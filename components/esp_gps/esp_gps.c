#include "esp_gps.h"
#include "freertos/semphr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "esp_log.h"

#define _mutex_lock(x)      while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS)
#define _mutex_unlock(x)    xSemaphoreGive(x)
#define _mutex_create()     xSemaphoreCreateMutex()
#define _mutex_destroy(x)   vSemaphoreDelete(x)

static const char *TAG = "ESP_GPS";

static const int STARTED_BIT = BIT0;
static const int STOPPED_BIT = BIT1;

typedef struct esp_gps_
{
    SemaphoreHandle_t lock;
    int uart_port_num;
    bool data_available;
    esp_gps_data_t data;
    char *buffer;
    bool run;
    EventGroupHandle_t state_event_bits;
} esp_gps_t;

static esp_err_t _send_cmd_and_get_response(esp_gps_handle_t esp_gps, const char *cmd, TickType_t timeout)
{
    while (1)
    {
        ESP_LOGI(TAG, "Write cmd: %s", cmd);
        uart_write_bytes(esp_gps->uart_port_num, cmd, strlen(cmd));
        int len = uart_read_bytes(esp_gps->uart_port_num, (uint8_t *)esp_gps->buffer, GPS_BUFFER_SIZE, timeout);
        if (len > 0) {
            esp_gps->buffer[len] = 0;
            ESP_LOGI(TAG, "Recv: %s", esp_gps->buffer);
            return ESP_OK;
        }
    }
    return ESP_FAIL;
}

static esp_err_t _send_cmd_and_expect(esp_gps_handle_t esp_gps, const char *cmd, const char *expect, TickType_t timeout)
{
    while (1)
    {
        if (_send_cmd_and_get_response(esp_gps, cmd, timeout) == ESP_OK) {
            if (strstr(esp_gps->buffer, expect) != NULL) {
                return ESP_OK;
            }
        }
    }
    return ESP_FAIL;
}

static int nmea_to_string(char *des, nmea_position *pos)
{
    int ret_len = 0;
    if (pos->cardinal == NMEA_CARDINAL_DIR_NORTH || pos->cardinal == NMEA_CARDINAL_DIR_EAST) {
        ret_len = sprintf(des, "%d.%d", pos->degrees, (int)((pos->minutes * 10000000) / 60));
    } else if (pos->cardinal == NMEA_CARDINAL_DIR_SOUTH || pos->cardinal == NMEA_CARDINAL_DIR_WEST) {
        ret_len = sprintf(des, "-%d.%d", pos->degrees, (int)((pos->minutes * 10000000) / 60));
    }
    return ret_len;
}

static void _gps_task(void *pv)
{
    esp_gps_handle_t esp_gps = (esp_gps_handle_t)pv;
    esp_err_t ret;
    // while (1)
    // {
    //     ret = _send_cmd_and_expect(esp_gps, GPS_ALL_GP_DATA_CMD, GPS_ALL_GP_DATA_ACCEPT,
    //                                GPS_DEFAULT_CMD_TIMEOUT);
    //     if (ret == ESP_OK) {
    //         break;
    //     }
    // }
    ESP_LOGI(TAG, "Init complete");
    _mutex_lock(esp_gps->lock);
    esp_gps->data_available = false;
    _mutex_unlock(esp_gps->lock);
    char fmt_buf[32];
    size_t total_bytes = 0;

    while (esp_gps->run)
    {
        int read_bytes = uart_read_bytes(esp_gps->uart_port_num,
                                         (uint8_t *)esp_gps->buffer + total_bytes,
                                         GPS_BUFFER_SIZE - total_bytes, 100 / portTICK_RATE_MS);
        if (read_bytes <= 0) {
            continue;
        }
        nmea_s *data;
        total_bytes += read_bytes;
        /* find start (a dollar sign) */
        char *start = memchr(esp_gps->buffer, '$', total_bytes);
        if (start == NULL) {
            total_bytes = 0;
            continue;
        }
        /* find end of line */
        char *end = memchr(start, '\r', total_bytes - (start - esp_gps->buffer));
        if (NULL == end || '\n' != *(++end)) {
            continue;
        }
        end[-1] = NMEA_END_CHAR_1;
        end[0] = NMEA_END_CHAR_2;
        data = nmea_parse(start, end - start + 1, 0);
        if (data != NULL) {
            if (data->errors != 0) {
                ESP_LOGD(TAG, "WARN: The sentence struct contains parse errors!\n");
            }

            if (data->type == NMEA_GPRMC) {
                ESP_LOGD(TAG, "GPRMC sentence\n");
                nmea_gprmc_s *pos = (nmea_gprmc_s *)data;
                char s_latitude[20];
                int s_latitude_len = 0;
                s_latitude_len = nmea_to_string(s_latitude, &pos->latitude);
                s_latitude[s_latitude_len] = '\0';

                char s_longitude[20];
                int s_longitude_len = 0;
                s_longitude_len = nmea_to_string(s_longitude, &pos->longitude);
                s_longitude[s_longitude_len] = '\0';
                pos->time.tm_hour += GPS_ORIGIN_GMT;

                if (s_longitude_len != 0 && s_latitude_len != 0) {
                    _mutex_lock(esp_gps->lock);
                    esp_gps->data_available = true;
                    strncpy(esp_gps->data.latitude, s_latitude, s_latitude_len + 1);
                    strncpy(esp_gps->data.longitude, s_longitude, s_longitude_len + 1);
                    esp_gps->data.time = pos->time;
                    esp_gps->data.speed = pos->speed;
                    esp_gps->data.course = pos->course;
                    esp_gps->data.valid = pos->valid;
                    _mutex_unlock(esp_gps->lock);
                }
            }
            nmea_free(data);
        }
        /* buffer empty? */
        if (end == esp_gps->buffer + total_bytes) {
            total_bytes = 0;
            continue;
        }
        /* copy rest of esp_gps->buffer to beginning */
        if (esp_gps->buffer != memmove(esp_gps->buffer, end, total_bytes - (end - esp_gps->buffer))) {
            total_bytes = 0;
            continue;
        }
        total_bytes -= end - esp_gps->buffer;
    }
    xEventGroupClearBits(esp_gps->state_event_bits, STARTED_BIT);
    xEventGroupSetBits(esp_gps->state_event_bits, STOPPED_BIT);
    vTaskDelete(NULL);
}

static esp_err_t esp_gps_wait_for_stopped(esp_gps_handle_t esp_gps, TickType_t ticks_to_wait)
{
    EventGroupHandle_t ev_bits = esp_gps->state_event_bits;
    EventBits_t uxBits = xEventGroupWaitBits(ev_bits, STOPPED_BIT, false, true, ticks_to_wait);
    if (uxBits & STOPPED_BIT) {
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t esp_gps_destroy(esp_gps_handle_t esp_gps)
{
    if (esp_gps == NULL) {
        ESP_LOGE(TAG, "esp_gps is not exist");
        return;
    }
    esp_gps->run = false;
    if (esp_gps_wait_for_stopped(esp_gps, portMAX_DELAY) == ESP_FAIL) {
        ESP_LOGE(TAG, "Can't destroy esp_gps handle");
        return ESP_FAIL;
    }
    uart_driver_delete(esp_gps->uart_port_num);
    if (esp_gps->lock) {
        _mutex_destroy(esp_gps->lock);
    }
    free(esp_gps->buffer);
    free(esp_gps);
    return ESP_OK;
}

esp_gps_handle_t esp_gps_init(esp_gps_config_t *config)
{
    esp_gps_handle_t esp_gps = calloc(1, sizeof(esp_gps_t));
    if (esp_gps == NULL) {
        return NULL;
    }
    int uart_baudrate = (config->uart_baudrate == 0) ? GPS_DEFAULT_BAUDRATE : config->uart_baudrate;
    esp_gps->uart_port_num = config->uart_port_num;
    esp_gps->lock = _mutex_create();
    if (esp_gps->lock == NULL) {
        ESP_LOGE(TAG, "Error create lock");
        goto _gps_init_failed;
    }

    uart_config_t uart_config = {
        .baud_rate = uart_baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    uart_param_config(esp_gps->uart_port_num, &uart_config);
    uart_set_pin(esp_gps->uart_port_num, config->uart_tx_pin, config->uart_rx_pin, -1, -1);
    uart_driver_install(esp_gps->uart_port_num, GPS_BUFFER_SIZE * 2, GPS_BUFFER_SIZE * 2, 0, NULL, 0);
    esp_gps->data_available = false;
    xTaskCreatePinnedToCore(_gps_task, "gps_task", 4 * 1024, esp_gps, 5, NULL, 1);

    esp_gps->buffer = (char *)malloc(GPS_BUFFER_SIZE + 1);
    if (esp_gps->buffer == NULL) {
        goto _gps_init_failed;
    }
    esp_gps->state_event_bits = xEventGroupCreate();
    xEventGroupClearBits(esp_gps->state_event_bits, STARTED_BIT);
    xEventGroupSetBits(esp_gps->state_event_bits, STOPPED_BIT);
    esp_gps->run = true;
    return esp_gps;
_gps_init_failed:
    if (esp_gps->lock) {
        _mutex_destroy(esp_gps->lock);
    }
    free(esp_gps);
    return NULL;
}

bool esp_gps_read_data(esp_gps_handle_t esp_gps, esp_gps_data_t *data)
{
    bool ret;
    _mutex_lock(esp_gps->lock);
    ret = esp_gps->data_available;
    if (esp_gps->data_available) {
        strcpy(data->latitude, esp_gps->data.latitude);
        strcpy(data->longitude, esp_gps->data.longitude);
        data->time = esp_gps->data.time;
        data->speed = esp_gps->data.speed;
        data->course = esp_gps->data.course;
        data->valid = esp_gps->data.valid;
        esp_gps->data_available = false;
    } else {
        strcpy(data->latitude, "10.775131");
        strcpy(data->longitude, "106.665061");
    }
    _mutex_unlock(esp_gps->lock);
    return true;
}

bool esp_gps_is_available(esp_gps_handle_t esp_gps)
{
    bool ret = false;
    _mutex_lock(esp_gps->lock);
    ret = esp_gps->data_available;
    _mutex_unlock(esp_gps->lock);
    return ret;
}
