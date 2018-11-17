/*
 * This file is subject to the terms of the Nanochip License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *                             ./LICENSE
 */
#ifndef _ESP_PPP_H_
#define _ESP_PPP_H_

#include "rom/queue.h"
#include "esp_err.h"
#include <time.h>
#include <sys/time.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct esp_ppp_* esp_ppp_handle_t;

typedef enum {
    MODEM_TYPE_SIM800 = 1,
    MODEM_TYPE_SIM868,
} esp_ppp_modem_type_t;

typedef struct {
    int tx_pin;
    int rx_pin;
    int reset_pin;
    int ri_pin;
    int dtr_pin;
    int uart_num;
} esp_ppp_modem_conn_t;

typedef struct {
    esp_ppp_modem_conn_t modem_conn;
    esp_ppp_modem_type_t modem_type;
    const char *tag;
    int task_stack;
    int task_prio;
} esp_ppp_cfg_t;

esp_ppp_handle_t esp_ppp_init(esp_ppp_cfg_t* ppp_cfg);

bool esp_ppp_is_connected(esp_ppp_handle_t periph);
void esp_ppp_get_ip_address(esp_ppp_handle_t periph);
void esp_ppp_recv_sms();
void esp_ppp_send_sms();

#ifdef __cplusplus
}
#endif

#endif
