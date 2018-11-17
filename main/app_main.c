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

static const char *TAG = "TRACKER_HERE";
// #define METHOD_PPP              1
#define MAX_HTTP_RECV_BUFFER    512

esp_ppp_handle_t ppp_handle = NULL;

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

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    esp_http_client_handle_t client = evt->client;
    const char *device_id = "a2abff51-5313-440d-a05f-7f7fcb20a0f9";
    const char *device_secret = "RQJbb6Ij8CTj2pDLhA-IRKMMk16gom0JQvGoN-j2pe0";
    uint32_t sz = 512;

    switch (evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:


            here_tracking_oauth_create_header(device_id,
                                              device_secret,
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
                printf("%.*s", evt->data_len, (char*)evt->data);
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

static void here_request(esp_gps_data_t gps_data)
{

    esp_http_client_config_t config = {
        .url = url,
        .event_handler = _http_event_handler,
        .buffer_size = 2048,
    };
    esp_http_client_handle_t client;

    // esp_http_client_set_header(client, "Content-Type", "application/json");
    // esp_http_client_set_method(client, HTTP_METHOD_POST);

    http_data_len = 0;
    esp_err_t err;
    char *access_token = NULL;
    int _retry = 3;
    while (_retry) {
        ESP_LOGI(TAG, "retry time : %d", _retry);
        client = esp_http_client_init(&config);
        esp_http_client_set_method(client, HTTP_METHOD_POST);
        err = esp_http_client_perform(client);
        _retry--;
        if (err == ESP_OK) {
            http_buffer[http_data_len] = 0;
            if (esp_http_client_get_status_code(client) == 200) {
                access_token = json_get_token_value(http_buffer, "accessToken");
                if (access_token) {
                    ESP_LOGE(TAG, "Access token = %s", access_token);
                }
                break;
            }

            ESP_LOGI(TAG, "HTTPS Status = %d, content_length = %d",
                    esp_http_client_get_status_code(client),
                    esp_http_client_get_content_length(client));
        } else {
            ESP_LOGE(TAG, "Error perform http request %s", esp_err_to_name(err));
        }
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
    if (access_token) {
        char *token = NULL;
        char *tracking_data = NULL;
        asprintf(&token, "Bearer %s", access_token);
        esp_http_client_set_url(client, "https://tracking.api.here.com/v2/");
        esp_http_client_set_method(client, HTTP_METHOD_POST);
        esp_http_client_set_header(client, "Authorization", token);
    
        ESP_LOGW(TAG, "header=%s", token);
        int data_len = asprintf(&tracking_data, "[{"
            "\"timestamp\": %ld,"
            "\"position\": {"
                "\"lat\": %s,"
                "\"lng\": %s,"
                "\"alt\": 1,"
                "\"accuracy\": 10"
            "}"
        "}]", time(NULL), gps_data.latitude, gps_data.longitude);
        char len_str[10];
        sprintf(len_str, "%d", data_len);
        ESP_LOGI(TAG, "Write data = %s", tracking_data);
        esp_http_client_set_post_field(client, tracking_data, data_len);
        esp_http_client_set_header(client, "Content-Type", "application/json");
        esp_http_client_set_header(client, "Content-Length", len_str);
        http_data_len = 0;
        _retry = 3;
        while (_retry) {
            ESP_LOGI(TAG, "retry time: %d", _retry);
            _retry--;
            err = esp_http_client_perform(client);

            if (err == ESP_OK) {
        
                ESP_LOGI(TAG, "HTTPS Status = %d, content_length = %d",
                        esp_http_client_get_status_code(client),
                        esp_http_client_get_content_length(client));
                break;
            } else {
                ESP_LOGE(TAG, "Error perform http request %s", esp_err_to_name(err));
            }
        }
        free(tracking_data);
        free(token);
        free(access_token);
    }
    esp_http_client_cleanup(client);
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
    here_request(gps_data);
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

void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    tcpip_adapter_init();
    // TFT_PinsInit();
    // spi_lobo_device_handle_t spi;
	
    // spi_lobo_bus_config_t buscfg={
    //     .miso_io_num=PIN_NUM_MISO,				// set SPI MISO pin
    //     .mosi_io_num=PIN_NUM_MOSI,				// set SPI MOSI pin
    //     .sclk_io_num=PIN_NUM_CLK,				// set SPI CLK pin
    //     .quadwp_io_num=-1,
    //     .quadhd_io_num=-1,
	// 	.max_transfer_sz = 6*1024,
    // };
    // spi_lobo_device_interface_config_t devcfg={
    //     .clock_speed_hz=8000000,                // Initial clock out at 8 MHz
    //     .mode=0,                                // SPI mode 0
    //     .spics_io_num=-1,                       // we will use external CS pin
	// 	.spics_ext_io_num=PIN_NUM_CS,           // external CS pin
	// 	.flags=LB_SPI_DEVICE_HALFDUPLEX,        // ALWAYS SET  to HALF DUPLEX MODE!! for display spi
    // };
    // ret=spi_lobo_bus_add_device(SPI_BUS, &buscfg, &devcfg, &spi);
    // assert(ret==ESP_OK);
	// printf("SPI: display device added to spi bus (%d)\r\n", SPI_BUS);
	// disp_spi = spi;

	// // ==== Test select/deselect ====
	// ret = spi_lobo_device_select(spi, 1);
    // assert(ret==ESP_OK);
	// ret = spi_lobo_device_deselect(spi);
    // assert(ret==ESP_OK);

	// printf("SPI: attached display device, speed=%u\r\n", spi_lobo_get_speed(spi));
	// printf("SPI: bus uses native pins: %s\r\n", spi_lobo_uses_native_pins(spi) ? "true" : "false");
    // TFT_display_init();

    esp_gps_config_t esp_gps_config = {
        .uart_baudrate = 9600,
        .uart_tx_pin = 19,
        .uart_rx_pin = 18,
        .uart_port_num = UART_NUM_2,
    };
    esp_gps_handle_t gps_handle = esp_gps_init(&esp_gps_config);

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
    // xTaskCreate(&http_update_task, "http_update_task", 8192, NULL, 5, NULL);
    char fmt_buf[32];
    esp_http_client_config_t config = {
        .url = "https://image.maps.api.here.com/mia/1.6/routing?app_id=2pt1MIM3bIHER2qfIM3M&app_code=IfhWsZtLJEZKDArU9yxSDg&waypoint0=10.767317,106.641038&waypoint1=10.782447,106.660454&lc=1652B4&lw=6&t=0&ppi=320&w=320&h=240",
        .event_handler = _http_event_handler,
    };
    // esp_http_client_handle_t client = esp_http_client_init(&config);

    // // GET
    // esp_err_t err = esp_http_client_perform(client);
    // if (err == ESP_OK) {
    //     ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %d",
    //     esp_http_client_get_status_code(client),
    //     esp_http_client_get_content_length(client));
    //     uint8_t *buff = malloc(40000);
    //     esp_http_client_read(client, (char*)buff, 40000);
    //     TFT_jpg_image(0, 0, 1, NULL, buff, 0);
    //     free(buff);
    // } else {
    //     ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
    // }
    // esp_http_client_cleanup(client);
    while(1){
        esp_gps_data_t gps_data;
        bool ret = esp_gps_read_data(gps_handle, &gps_data);
        if (ret == true) {
            ESP_LOGI(TAG, "Longitude: %s\n", gps_data.longitude);
            ESP_LOGI(TAG, "Latitude: %s\n", gps_data.latitude);
            strftime(fmt_buf, sizeof(fmt_buf), "%H:%M:%S", &gps_data.time);
            ESP_LOGI(TAG, "Time: %s\n", fmt_buf);
            // time_t t = mktime(&gps_data.time);
            // struct timeval now = { .tv_sec = t };
            // settimeofday(&now, NULL);
            if (network_is_availablde()) {
                http_update_position(gps_data);
            }
            // http_update_position(gps_data);
        } else {
            ESP_LOGI(TAG, "Data not available!");
        }
        vTaskDelay(60000 / portTICK_PERIOD_MS);
    }
    esp_gps_destroy(gps_handle);
}
