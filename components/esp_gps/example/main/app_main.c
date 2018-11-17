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

static const char *TAG = "ESP_GPS";

void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    esp_gps_config_t esp_gps_config = {
        .uart_baudrate = 9600,
        .uart_tx_pin = 19,
        .uart_rx_pin = 18,
        .uart_port_num = UART_NUM_2,
    };
    esp_gps_handle_t gps_handle = esp_gps_init(&esp_gps_config);

    char fmt_buf[32];

    while(1){
        esp_gps_data_t gps_data;
        bool ret = esp_gps_read_data(gps_handle, &gps_data);
        if (ret == true) {
            ESP_LOGI(TAG, "Longitude: %s\n", gps_data.longitude);
            ESP_LOGI(TAG, "Latitude: %s\n", gps_data.latitude);
            strftime(fmt_buf, sizeof(fmt_buf), "%H:%M:%S", &gps_data.time);
            ESP_LOGI(TAG, "Time: %s\n", fmt_buf);
        } else {
            ESP_LOGI(TAG, "Data not available!");
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    esp_gps_destroy(gps_handle);
}
