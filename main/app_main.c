/* PPPoS Client Example with GSM (tested with Telit GL865-DUAL-V3)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_gps.h"
#include "esp_ppp.h"
#include "tcpip_adapter.h"
#include "esp_http_client.h"
#include "tft.h"
#include "esp_http_client.h"
#include "mbedtls/sha256.h"
#include "mbedtls/md.h"
#include "mbedtls/base64.h"

#include "lwip/err.h"
#include "lwip/apps/sntp.h"
#include "app_wifi.h"

#include <sys/time.h>
#include "here_tracking_oauth.h"
#include "json_utils.h"
#include "tftspi.h"
#include "tft.h"
#include "qrcode.h"
#include "driver/gpio.h"

#define SPI_BUS TFT_HSPI_HOST

static const char *TAG = "TRACKER_HERE";
// #define METHOD_PPP              1
#define MAX_HTTP_RECV_BUFFER    512
#define STATIC_MAP_TEMPLATE     "https://image.maps.api.here.com/mia/1.6/tiltmap?app_id=4SOW7wtvnVNhniJUSFRr&app_code=LAQ7xdmuHxFmTlPhb_8Heg&w=320&h=240&sb=mk&z=15&t=%d&u=100&c=%s,%s"
#define ROUTING_MAP_TEMPLATE    "https://image.maps.api.here.com/mia/1.6/routing?app_id=4SOW7wtvnVNhniJUSFRr&app_code=LAQ7xdmuHxFmTlPhb_8Heg&waypoint0=%s,%s&waypoint1=%s,%s&poix0=%s,%s;00a3f2;00a3f2;11;.&poix1=%s,%s;white;white;11;.&lc=1652B4&lw=6&t=%d&ppi=320&w=320&h=240"
#define REQUEST_QR_TEMPLATE     "http://192.168.8.135:3001/%s"

#define _mutex_lock(x)      while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS)
#define _mutex_unlock(x)    xSemaphoreGive(x)
#define _mutex_create()     xSemaphoreCreateMutex()
#define _mutex_destroy(x)   vSemaphoreDelete(x)

#define BUTTON_B_PIN        38
#define BUTTON_A_PIN        39
#define BUTTON_C_PIN        37

typedef enum {
    STATIC_MAP_VIEW = 0,
    ROUTING_MAP,
} tracker_state_t;

typedef struct {
    int state_version;
    const char *device_id;
    const char *device_secret;
    char *access_token;
    char target_lat[20];
    char target_long[20];
    tracker_state_t tracker_state;
    esp_gps_handle_t gps_handle;
    int map_type;
} tracker_handle_t;

tracker_handle_t tracker_handle = {
    .state_version = 0,
    .device_id = "567c0e98-6f60-4f24-a7dc-e5669636c649",
    .device_secret = "mqqk5t7AS4-KVS5aRaPQDpHmmj8ml8b-a8oMdmFkOOw",
    .access_token = NULL,
    .tracker_state = STATIC_MAP_VIEW,
    .map_type = 0,
};

esp_ppp_handle_t ppp_handle = NULL;
// static int state_version = 0;
static struct tm* tm_info;
static char tmp_buff[64];
static time_t time_now, time_last = 0;

// const char *device_id = "567c0e98-6f60-4f24-a7dc-e5669636c649";
// const char *device_secret = "mqqk5t7AS4-KVS5aRaPQDpHmmj8ml8b-a8oMdmFkOOw";

// char *access_token;

SemaphoreHandle_t lcd_lock;

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}


const char *url  = "https://tracking.api.here.com/v2/token";
char auth_header_here[512];
char http_buffer[4096];
int http_data_len = 0;

uint8_t http_map_buffer[40000];
uint16_t http_map_data_len;

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    esp_http_client_handle_t client = evt->client;
    uint32_t sz = 512;

    switch (evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
                // Write out data
                memcpy(http_buffer + http_data_len, evt->data, evt->data_len);
                http_data_len += evt->data_len;
                printf("%.*s\n", evt->data_len, (char*)evt->data);
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}

esp_err_t _http_event_token_handler(esp_http_client_event_t *evt)
{
    esp_http_client_handle_t client = evt->client;
    uint32_t sz = 512;

    switch (evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:


            here_tracking_oauth_create_header(tracker_handle.device_id,
                                              tracker_handle.device_secret,
                                              "tracking.api.here.com",
                                              0,
                                              auth_header_here,
                                              &sz);
            // ESP_LOGW(TAG, "Header=%s", auth_header);
            // ESP_LOGW(TAG, "data=%s", tracking_data);
            // esp_http_client_set_header(client, "x-here-timestamp", timestamp);
            esp_http_client_set_header(client, "Authorization", auth_header_here);
            ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED, timestamp: %ld", time(NULL));
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
                // Write out data
                memcpy(http_buffer + http_data_len, evt->data, evt->data_len);
                http_data_len += evt->data_len;
                printf("%.*s\n", evt->data_len, (char*)evt->data);
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}

esp_err_t _http_event_map_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        // ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
				memcpy(http_map_buffer + http_map_data_len, evt->data, evt->data_len);
                http_map_data_len += evt->data_len;
			}
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}

int filter_data_json(const char* in, const char* key, char* out)
{
    int count = 0;
    int len_in = strlen(in);
    int len_key = strlen(key);
    ESP_LOGI(TAG, "%s %d %s %d", in, len_in, key, len_key);
    for (int i = 0; i < len_in - len_key; i++) {
        bool f_error = false;
        for (int j = 0; j < len_key; j++) {
            if (in[i+j] != key[j]) {
                f_error = true;
                break;
            }
        }
        if (!f_error) {
            int idx = i + len_key + 2;
            while ((in[idx] >= '0' && in[idx] <= '9') || in[idx] == '.') {
                out[count] = in[idx];
                count++;
                idx++;
            }
            break;
        }
    }
    return count;
}

static bool here_request(esp_gps_data_t gps_data)
{
    bool ret = false;
    esp_http_client_config_t config = {
        .url = url,
        .event_handler = _http_event_handler,
        .buffer_size = 2048,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err;

    if (tracker_handle.access_token != NULL) {
        char *token = NULL;
        char *tracking_data = NULL;
        asprintf(&token, "Bearer %s", tracker_handle.access_token);
        esp_http_client_set_url(client, "https://tracking.api.here.com/v2/");
        esp_http_client_set_method(client, HTTP_METHOD_POST);
        esp_http_client_set_header(client, "Authorization", token);
    
        ESP_LOGD(TAG, "header=%s", token);
        int data_len = asprintf(&tracking_data, "[{"
            "\"timestamp\": %ld,"
            "\"position\": {"
                "\"lat\": %s,"
                "\"lng\": %s,"
                "\"alt\": 1,"
                "\"accuracy\": 10"
            "},"
            "\"payload\": {"
                "\"temperature\": 25"
            "},"
            "\"system\": {"
                "\"stateVersion\": %d"
            "}"
        "}]", time(NULL), gps_data.latitude, gps_data.longitude, tracker_handle.state_version);
        char len_str[10];
        sprintf(len_str, "%d", data_len);
        ESP_LOGD(TAG, "Write data = %s", tracking_data);
        esp_http_client_set_post_field(client, tracking_data, data_len);
        esp_http_client_set_header(client, "Content-Type", "application/json");
        esp_http_client_set_header(client, "Content-Length", len_str);
        http_data_len = 0;
            err = esp_http_client_perform(client);

            if (err == ESP_OK) {
                ESP_LOGI(TAG, "HTTPS Status = %d, content_length = %d",
                        esp_http_client_get_status_code(client),
                        esp_http_client_get_content_length(client));
                http_buffer[http_data_len] = '\0';
                char out_state_version[10];
                int filter_len_state_version = filter_data_json(http_buffer, "stateVersion", out_state_version);
                if (filter_len_state_version > 0) {
                    int server_state_version = atoi(out_state_version);
                    if (tracker_handle.state_version == 0) {
                        tracker_handle.state_version = server_state_version;
                    } else if (server_state_version > tracker_handle.state_version) {
                        tracker_handle.state_version = server_state_version;
                        int filter_len_lat = filter_data_json(http_buffer, "lat", tracker_handle.target_lat);
                        int filter_len_lng = filter_data_json(http_buffer, "lng", tracker_handle.target_long);
                        if (filter_len_lat > 0 && filter_len_lng > 0) {
                            tracker_handle.target_lat[filter_len_lat] = 0;
                            tracker_handle.target_long[filter_len_lng] = 0;
                            ESP_LOGI(TAG, "parse lat: %s, parse long: %s", tracker_handle.target_lat, 
                                        tracker_handle.target_long);
                            tracker_handle.tracker_state = ROUTING_MAP;
                        }
                    }
                }
                ret = true;
            } else {
                ESP_LOGE(TAG, "Error perform http request %s", esp_err_to_name(err));
            }
        free(tracking_data);
        free(token);
        // free(access_token);
    }
    esp_http_client_cleanup(client);
    return ret;
}

static void http_update_position(esp_gps_data_t gps_data)
{
    // esp_gps_data_t* gps_data = (esp_gps_data_t*)pv;
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    int _retry = 3;
    while (_retry) {
        ESP_LOGI(TAG, "try: %d", _retry);
        if (here_request(gps_data)) {
            break;
        }
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        _retry--;
    }
    ESP_LOGI(TAG, "Finish http example");
    // vTaskDelete(NULL);
}

bool network_is_availablde() {
#ifdef METHOD_PPP
    if (esp_ppp_is_connected(ppp_handle)) {
        return true;
    }
#else
    tcpip_adapter_ip_info_t ip;
    tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip);

    if (ip.ip.addr) {
        return true;
    }
#endif
    return false;
}

void display_map(esp_gps_data_t gps_data)
{
    char* tracking_data = NULL;
    int data_len = 0;
    if (tracker_handle.tracker_state == STATIC_MAP_VIEW) {
        data_len = asprintf(&tracking_data, STATIC_MAP_TEMPLATE, tracker_handle.map_type, gps_data.latitude, gps_data.longitude);
    } else {
        data_len = asprintf(&tracking_data, ROUTING_MAP_TEMPLATE, 
                    gps_data.latitude, gps_data.longitude, tracker_handle.target_lat, tracker_handle.target_long,
                    gps_data.latitude, gps_data.longitude, tracker_handle.target_lat, tracker_handle.target_long, tracker_handle.map_type);
    }
    if (data_len == 0) {
        return;
    }
    tracking_data[data_len] = 0;
    ESP_LOGI(TAG, "map req: %s", tracking_data);
    esp_http_client_config_t config = {
        .url = tracking_data,
        .event_handler = _http_event_map_handler,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    http_map_data_len = 0;
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
		TFT_jpg_image(CENTER, CENTER, 0, NULL, http_map_buffer, http_map_data_len); 
    } else {
        ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
    ESP_LOGI(TAG, "Display complete %d", http_map_data_len);
}


void drawBig(int size, int x0, int y0, color_t c)
{
    int newX = x0 * size;
    int newY = y0 * size;
    for (int i = newX; i < (newX + size); i++)
    {
        for (int j = newY; j < (newY + size); j++)
        {
            TFT_drawPixel(i, j, c, 1);
        }
    }
}

static void button_task(void *pv)
{
    gpio_pad_select_gpio(BUTTON_A_PIN);

    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BUTTON_A_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_A_PIN, GPIO_PULLUP_ONLY);

    gpio_pad_select_gpio(BUTTON_B_PIN);

    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BUTTON_B_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_B_PIN, GPIO_PULLUP_ONLY);

    gpio_pad_select_gpio(BUTTON_C_PIN);

    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BUTTON_C_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_C_PIN, GPIO_PULLUP_ONLY);

    while(1) {
        if (gpio_get_level(BUTTON_B_PIN) == 0) {
            while (gpio_get_level(BUTTON_B_PIN) == 0);
            if (tracker_handle.tracker_state == ROUTING_MAP) {
                tracker_handle.tracker_state = STATIC_MAP_VIEW;
                esp_gps_data_t gps_data;
                if (esp_gps_read_data(tracker_handle.gps_handle, &gps_data)) {
                    ESP_LOGI(TAG, "Longitude: %s\n", gps_data.longitude);
                    ESP_LOGI(TAG, "Latitude: %s\n", gps_data.latitude);
                    if (xSemaphoreTake(lcd_lock, 500 / portTICK_PERIOD_MS) == pdPASS) {
                        display_map(gps_data);
                        _mutex_unlock(lcd_lock);
                    }
                }
                continue;
            }
            _mutex_lock(lcd_lock);
            QRCode qrcode;
            int version = 11;
            uint8_t *qrcodeData = calloc(1, qrcode_getBufferSize(version));
            char* qr_show = NULL;
            int data_len = asprintf(&qr_show, REQUEST_QR_TEMPLATE, tracker_handle.device_id);
            qr_show[data_len] = 0;
            qrcode_initText(&qrcode, qrcodeData, version, 0, qr_show);
            TFT_fillScreen(TFT_BLACK);
            for (uint8_t y = 0; y < qrcode.size; y++)
            {
                for (uint8_t x = 0; x < qrcode.size; x++)
                {
                    drawBig(3, x + 22, y + 3, qrcode_getModule(&qrcode, x, y) ? TFT_WHITE : TFT_BLACK);
                }
            }
            TFT_setFont(COMIC24_FONT, NULL);
            TFT_print("Scan HERE", CENTER, 200);
            vTaskDelay(10000 / portTICK_PERIOD_MS);
            _mutex_unlock(lcd_lock);
        }
        if (gpio_get_level(BUTTON_A_PIN) == 0) {
            while (gpio_get_level(BUTTON_B_PIN) == 0);
            tracker_handle.map_type--;
            if (tracker_handle.map_type < 0) {
                tracker_handle.map_type = 14;
            }
            esp_gps_data_t gps_data;
            if (esp_gps_read_data(tracker_handle.gps_handle, &gps_data)) {
                ESP_LOGI(TAG, "Longitude: %s\n", gps_data.longitude);
                ESP_LOGI(TAG, "Latitude: %s\n", gps_data.latitude);
                if (xSemaphoreTake(lcd_lock, 500 / portTICK_PERIOD_MS) == pdPASS) {
                    display_map(gps_data);
                    _mutex_unlock(lcd_lock);
                }
            }
        }
        if (gpio_get_level(BUTTON_C_PIN) == 0) {
            while (gpio_get_level(BUTTON_C_PIN) == 0);
            tracker_handle.map_type++;
            if (tracker_handle.map_type > 14) {
                tracker_handle.map_type = 0;
            }
            esp_gps_data_t gps_data;
            if (esp_gps_read_data(tracker_handle.gps_handle, &gps_data)) {
                ESP_LOGI(TAG, "Longitude: %s\n", gps_data.longitude);
                ESP_LOGI(TAG, "Latitude: %s\n", gps_data.latitude);
                // if (xSemaphoreTake(lcd_lock, 500 / portTICK_PERIOD_MS) == pdPASS) {
                    display_map(gps_data);
                    // _mutex_unlock(lcd_lock);
                // }
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    tcpip_adapter_init();

    tft_disp_type = DEFAULT_DISP_TYPE;
	_width = DEFAULT_TFT_DISPLAY_WIDTH;  // smaller dimension
	_height = DEFAULT_TFT_DISPLAY_HEIGHT; // larger dimension
	max_rdclock = 8000000;
    TFT_PinsInit();
    spi_lobo_device_handle_t spi;
	
    spi_lobo_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,				// set SPI MISO pin
        .mosi_io_num=PIN_NUM_MOSI,				// set SPI MOSI pin
        .sclk_io_num=PIN_NUM_CLK,				// set SPI CLK pin
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
		.max_transfer_sz = 6*1024,
    };
    spi_lobo_device_interface_config_t devcfg={
        .clock_speed_hz=8000000,                // Initial clock out at 8 MHz
        .mode=0,                                // SPI mode 0
        .spics_io_num=-1,                       // we will use external CS pin
		.spics_ext_io_num=PIN_NUM_CS,           // external CS pin
		.flags=LB_SPI_DEVICE_HALFDUPLEX,        // ALWAYS SET  to HALF DUPLEX MODE!! for display spi
    };

	ret=spi_lobo_bus_add_device(SPI_BUS, &buscfg, &devcfg, &spi);
	disp_spi = spi;

	TFT_display_init();
	max_rdclock = find_rd_speed();
	font_rotate = 0;
	text_wrap = 0;
	font_transparent = 0;
	font_forceFixed = 0;
	gray_scale = 0;
    TFT_setGammaCurve(DEFAULT_GAMMA_CURVE);
	TFT_setRotation(LANDSCAPE);
	TFT_setFont(DEJAVU24_FONT, NULL);
	TFT_resetclipwin();

    TFT_print("HERE Tracker", CENTER, CENTER);

    vTaskDelay(3000 / portTICK_PERIOD_MS);
    
    lcd_lock = _mutex_create();

    esp_gps_config_t esp_gps_config = {
        .uart_baudrate = 9600,
        .uart_tx_pin = 19,
        .uart_rx_pin = 34,
        .uart_port_num = UART_NUM_2,
    };
    tracker_handle.gps_handle = esp_gps_init(&esp_gps_config);

#ifdef METHOD_PPP
    esp_ppp_cfg_t ppp_cfg = {
        .modem_conn = {
            .tx_pin = 17,
            .rx_pin = 16,
            .uart_num = 1,
            .reset_pin = 5,
        }
    };
    ppp_handle = esp_ppp_init(&ppp_cfg);
    while (!esp_ppp_is_connected(ppp_handle)) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
#else
    app_wifi_initialise();
    app_wifi_wait_connected();
#endif
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    initialize_sntp();
    xTaskCreate(button_task, "button_task", 4096, NULL, 5, NULL);
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }
    while (1) {
        esp_http_client_config_t config = {
            .url = url,
            .event_handler = _http_event_token_handler,
            .buffer_size = 2048,
        };
        esp_http_client_handle_t client = esp_http_client_init(&config);

        // esp_http_client_set_header(client, "Content-Type", "application/json");
        esp_http_client_set_method(client, HTTP_METHOD_POST);

        http_data_len = 0;
        esp_err_t err = esp_http_client_perform(client);

        tracker_handle.access_token = NULL;
        if (err == ESP_OK) {
            http_buffer[http_data_len] = 0;
            if (esp_http_client_get_status_code(client) == 200) {
                tracker_handle.access_token = json_get_token_value(http_buffer, "accessToken");
                if (tracker_handle.access_token) {
                    ESP_LOGI(TAG, "Access token = %s", tracker_handle.access_token);
                    break;
                }
            }

            ESP_LOGI(TAG, "HTTPS Status = %d, content_length = %d",
                    esp_http_client_get_status_code(client),
                    esp_http_client_get_content_length(client));
        } else {
            ESP_LOGE(TAG, "Error perform http request %s", esp_err_to_name(err));
        }
        esp_http_client_cleanup(client);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    char fmt_buf[32];
    while(1){
        esp_gps_data_t gps_data;
        bool ret = esp_gps_read_data(tracker_handle.gps_handle, &gps_data);
        if (ret == true) {
            ESP_LOGI(TAG, "Longitude: %s\n", gps_data.longitude);
            ESP_LOGI(TAG, "Latitude: %s\n", gps_data.latitude);
            strftime(fmt_buf, sizeof(fmt_buf), "%H:%M:%S", &gps_data.time);
            ESP_LOGI(TAG, "Time: %s\n", fmt_buf);
            if (network_is_availablde()) {
                http_update_position(gps_data);
            }
            if (xSemaphoreTake(lcd_lock, 500 / portTICK_PERIOD_MS) == pdPASS) {
                display_map(gps_data);
                _mutex_unlock(lcd_lock);
            }
        } else {
            ESP_LOGI(TAG, "Data not available!");
        }
        vTaskDelay(60000 / portTICK_PERIOD_MS);
    }
    esp_gps_destroy(tracker_handle.gps_handle);
}
