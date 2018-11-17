#ifndef _ESP_GPS_H
#define _ESP_GPS_H

#include "esp_err.h"
#include "nmea.h"
#include "gpgll.h"
#include "gpgga.h"
#include "gprmc.h"
#include "parser.h"
#include <time.h>
#include "driver/uart.h"

#define GPS_DEFAULT_BAUDRATE        9600
#define GPS_BUFFER_SIZE             1024
#define GPS_DEFAULT_CMD_TIMEOUT     (2000 / portTICK_RATE_MS)
#define GPS_DEFAULT_CMD_RETRY       5
#define GPS_ALL_GP_DATA_CMD         "$PMTK353,1,0,0,0,0*2A\r\n"
#define GPS_ALL_GP_DATA_ACCEPT      "$PMTK001,353,3,1,0,0,0,0,1*35\r\n"
#define GPS_ORIGIN_GMT              +7

typedef struct
{
    const char *tag;
    int uart_tx_pin;
    int uart_rx_pin;
    int uart_port_num;
    int uart_baudrate;
} esp_gps_config_t;

typedef struct
{
    char longitude[20];
    char latitude[20];
    struct tm time;
    float speed;
    float course;
    int valid;
} esp_gps_data_t;

typedef struct esp_gps_ *esp_gps_handle_t;

esp_gps_handle_t esp_gps_init(esp_gps_config_t *config);

bool esp_gps_read_data(esp_gps_handle_t esp_gps, esp_gps_data_t *data);

esp_err_t esp_gps_destroy(esp_gps_handle_t self);

bool esp_gps_is_available(esp_gps_handle_t esp_gps);

#endif
