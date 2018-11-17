/*
 * This file is subject to the terms of the Nanochip License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *                             ./LICENSE
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "rom/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_ppp.h"
#include "sdkconfig.h"
#include "freertos/event_groups.h"

#include "driver/uart.h"
#include "netif/ppp/pppos.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "netif/ppp/pppapi.h"

static const char* TAG = "ESP_PPP";
static esp_ppp_handle_t g_ppp = NULL;

#define PPP_ERROR_CHECK(err, action) if (err) action

#define PPP_BUF_SIZE (1024)
#define PPP_DEFAULT_TIMEOUT_MS (3000)
#define PPP_DEFAULT_UART UART_NUM_1
#define PPP_DEFAULT_TX_PIN 16
#define PPP_DEFAULT_RX_PIN 17
#define PPP_DEFAULT_CMD_RETRY 3
#define PPP_DEFAULT_CMD_TIMEOUT (2000/portTICK_RATE_MS)
#define PPP_DEFAULT_TASK_STACK (4*1024)
#define PPP_DEFAULT_TASK_PRIO (5)

typedef enum {
    PPP_STATE_UNINIT = 0,
    PPP_STATE_INIT,
    PPP_STATE_CONNECT,
    PPP_STATE_IP,
    PPP_STATE_WAIT_TIMEOUT,
} ppp_state_t;

typedef struct esp_ppp_ {
    esp_ppp_modem_conn_t modem_conn;
    esp_ppp_modem_type_t modem_type;
    /* The PPP control block */
    ppp_pcb *ppp;

    /* The PPP IP interface */
    struct netif ppp_netif;
    int rx_timeout_tick;
    char *buffer;
    ppp_state_t state;
    EventGroupHandle_t state_event;
    bool run;
    bool connected;
    int task_stack;
    int task_prio;
} esp_ppp_t;

static const int STARTED_BIT = BIT0;
static const int STOPPED_BIT = BIT1;
static const int PPP_STOPPED_BIT = BIT2;

typedef struct {
    const char *name;
    const char *number;
    const char *apn;
    const char *user;
    const char *pass;
} apn_map_t;

//http://www.numberportabilitylookup.com/networks?s=
const apn_map_t apn_map[] = {
    { "Viettel", "45204", "e-internet", "", ""},
    { "Mobifone", "45201", "m-wap", "mms", "mms"},
    { "Mobifone", "45202", "m-wap", "mms", "mms"},
    { NULL, NULL, NULL, NULL, NULL },
};

static u32_t ppp_output_callback(ppp_pcb *pcb, u8_t *data, u32_t len, void *ctx)
{
    esp_ppp_handle_t esp_ppp = (esp_ppp_handle_t)ctx;
    ESP_LOGD(TAG, "==> PPP tx len %d", len);
    esp_ppp->rx_timeout_tick = esp_timer_get_time()/1000;
    return uart_write_bytes(esp_ppp->modem_conn.uart_num, (const char *)data, len);
}


/* PPP status callback example */
static void ppp_status_cb(ppp_pcb *pcb, int err_code, void *ctx)
{

    esp_ppp_handle_t esp_ppp = (esp_ppp_handle_t)ctx;
    struct netif *pppif = ppp_netif(pcb);

    if (err_code) {
        esp_ppp->state = PPP_STATE_WAIT_TIMEOUT;
        esp_ppp->connected = false;
    }
    ESP_LOGI(TAG, "status_cb: err_code = %d", err_code);
    switch (err_code) {
        case PPPERR_NONE: {
                ESP_LOGI(TAG, "status_cb: Connected");
#if PPP_IPV4_SUPPORT
                ESP_LOGI(TAG, "   our_ipaddr  = %s", ipaddr_ntoa(&pppif->ip_addr));
                ESP_LOGI(TAG, "   his_ipaddr  = %s", ipaddr_ntoa(&pppif->gw));
                ESP_LOGI(TAG, "   netmask     = %s", ipaddr_ntoa(&pppif->netmask));
#endif /* PPP_IPV4_SUPPORT */
#if PPP_IPV6_SUPPORT
                ESP_LOGI(TAG, "   our6_ipaddr = %s", ip6addr_ntoa(netif_ip6_addr(pppif, 0)));
#endif /* PPP_IPV6_SUPPORT */
                esp_ppp->connected = true;
                esp_ppp->rx_timeout_tick = esp_timer_get_time()/1000;
                break;
            }
        case PPPERR_PARAM:
            ESP_LOGE(TAG, "status_cb: Invalid parameter");
            break;
        case PPPERR_OPEN:
            ESP_LOGE(TAG, "status_cb: Unable to open PPP session");
            break;
        case PPPERR_DEVICE:
            ESP_LOGE(TAG, "status_cb: Invalid I/O device for PPP");
            break;
        case PPPERR_ALLOC:
            ESP_LOGE(TAG, "status_cb: Unable to allocate resources");
            break;
        case PPPERR_USER:
            ESP_LOGE(TAG, "status_cb: User interrupt");
            xEventGroupSetBits(esp_ppp->state_event, PPP_STOPPED_BIT);
            break;
        case PPPERR_CONNECT:
            ESP_LOGE(TAG, "status_cb: Connection lost");
            break;

        case PPPERR_AUTHFAIL:
            ESP_LOGE(TAG, "status_cb: Failed authentication challenge");
            break;

        case PPPERR_PROTOCOL:
            ESP_LOGE(TAG, "status_cb: Failed to meet protocol");
            break;

        case PPPERR_PEERDEAD:
            ESP_LOGE(TAG, "status_cb: Connection timeout");
            break;

        case PPPERR_IDLETIMEOUT:
            ESP_LOGE(TAG, "status_cb: Idle Timeout");
            break;

        case PPPERR_CONNECTTIME:
            ESP_LOGE(TAG, "status_cb: Max connect time reached");
            break;

        case PPPERR_LOOPBACK:
            ESP_LOGE(TAG, "status_cb: Loopback detected");
            break;

        default:
            ESP_LOGE(TAG, "status_cb: Unknown error code %d", err_code);
            break;

    }

    /*
     * This should be in the switch case, this is put outside of the switch
     * case for example readability.
     */

    if (err_code == PPPERR_NONE) {
        return;
    }

    /* ppp_close() was previously called, don't reconnect */
    if (err_code == PPPERR_USER) {
        /* ppp_free(); -- can be called here */
        return;
    }


    /*
     * Try to reconnect in 30 seconds, if you need a modem chatscript you have
     * to do a much better signaling here ;-)
     */
    //ppp_connect(pcb, 30);
    /* OR ppp_listen(pcb); */
}

// static void ppp_timer_handler(xTimerHandle tmr)
// {
//     esp_periph_handle_t periph = (esp_periph_handle_t) pvTimerGetTimerID(tmr);
//     esp_ppp_t *esp_ppp = esp_periph_get_data(periph);
// }

static esp_err_t _at_and_get_response(esp_ppp_t *esp_ppp, const char *cmd, TickType_t timeout, int retry)
{
    do {
        ESP_LOGI(TAG, "Write cmd: %s", cmd);
        uart_write_bytes(esp_ppp->modem_conn.uart_num, cmd, strlen(cmd));
        int len = uart_read_bytes(esp_ppp->modem_conn.uart_num, (uint8_t *)esp_ppp->buffer, PPP_BUF_SIZE, timeout);
        if (len > 0) {
            esp_ppp->buffer[len] = 0;
            ESP_LOGI(TAG, "Recv: %s", esp_ppp->buffer);
            return ESP_OK;
        }

        if (retry > 0) {
            retry --;
        }
    } while (retry);
    return ESP_FAIL;
}

static esp_err_t _at_and_expect(esp_ppp_t *esp_ppp, const char *cmd, const char *expect, TickType_t timeout, int retry)
{
    int _retry = retry;
    do {
        if (_at_and_get_response(esp_ppp, cmd, timeout, 1) == ESP_OK) {
            if (strstr(esp_ppp->buffer, expect) != NULL) {
                return ESP_OK;
            }
        }

        if (retry > 0) {
            retry --;
        }
    } while (retry);
    return ESP_FAIL;
}

static void _modem_reset(esp_ppp_t *esp_ppp)
{
    if (esp_ppp->modem_conn.reset_pin) {
        gpio_set_level(esp_ppp->modem_conn.reset_pin, 0);
        vTaskDelay(1200 / portTICK_PERIOD_MS);
        gpio_set_level(esp_ppp->modem_conn.reset_pin, 1);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

static esp_err_t _detect_apn(esp_ppp_t *esp_ppp, char *output)
{
    int i = 0;
    while (apn_map[i].number != NULL) {
        if (strstr(esp_ppp->buffer, apn_map[i].number) != NULL) {
            sprintf(output, "AT+CSTT=\"%s\",\"%s\",\"%s\"\r\n", apn_map[i].apn, apn_map[i].user, apn_map[i].pass);
            return ESP_OK;
        }
        i++;
    }
    return ESP_FAIL;
}

static void _ppp_stop_task(esp_ppp_t *esp_ppp)
{
    if (esp_ppp->run) {
        esp_ppp->run = false;
        xEventGroupWaitBits(esp_ppp->state_event, STOPPED_BIT, false, true, portMAX_DELAY);
    }
}


static void _ppp_task(void *args)
{
    esp_ppp_handle_t esp_ppp = (esp_ppp_handle_t)args;

    xEventGroupClearBits(esp_ppp->state_event, STOPPED_BIT);
    xEventGroupSetBits(esp_ppp->state_event, STARTED_BIT);

    while (esp_ppp->run) {

        if (esp_ppp->state == PPP_STATE_UNINIT) {
            esp_err_t err = ESP_OK;
            _modem_reset(esp_ppp);
            uart_set_baudrate(esp_ppp->modem_conn.uart_num, 115200);
            err |= _at_and_expect(esp_ppp, "AT\r\n", "OK", PPP_DEFAULT_CMD_TIMEOUT, PPP_DEFAULT_CMD_RETRY);
            PPP_ERROR_CHECK(err, continue);
            // err |= _at_and_expect(esp_ppp, "AT+IPR=460800\r\n", "OK", PPP_DEFAULT_CMD_TIMEOUT, PPP_DEFAULT_CMD_RETRY);
            // PPP_ERROR_CHECK(err, continue);
            // uart_set_baudrate(esp_ppp->modem_conn.uart_num, 460800);
            // err |= _at_and_expect(esp_ppp, "AT\r\n", "OK", PPP_DEFAULT_CMD_TIMEOUT, PPP_DEFAULT_CMD_RETRY);
            // PPP_ERROR_CHECK(err, continue);
            err |= _at_and_expect(esp_ppp, "ATZ\r\n", "OK", PPP_DEFAULT_CMD_TIMEOUT, PPP_DEFAULT_CMD_RETRY);
            PPP_ERROR_CHECK(err, continue);
            err |= _at_and_expect(esp_ppp, "ATE0\r\n", "OK", PPP_DEFAULT_CMD_TIMEOUT, PPP_DEFAULT_CMD_RETRY);
            PPP_ERROR_CHECK(err, continue);
            err |= _at_and_expect(esp_ppp, "AT+CPIN?\r\n", "CPIN: READY", PPP_DEFAULT_CMD_TIMEOUT, PPP_DEFAULT_CMD_RETRY);
            PPP_ERROR_CHECK(err, continue);
            err |= _at_and_expect(esp_ppp, "AT+COPS=0,2\r\n", "OK", PPP_DEFAULT_CMD_TIMEOUT, PPP_DEFAULT_CMD_RETRY);
            PPP_ERROR_CHECK(err, continue);
            if (_at_and_get_response(esp_ppp, "AT+COPS?\r\n", PPP_DEFAULT_CMD_TIMEOUT, PPP_DEFAULT_CMD_RETRY) == ESP_OK) {
                char set_apn_cmd[32];
                if (_detect_apn(esp_ppp, set_apn_cmd) == ESP_OK) {
                    err |= _at_and_expect(esp_ppp, set_apn_cmd, "OK", PPP_DEFAULT_CMD_TIMEOUT, PPP_DEFAULT_CMD_RETRY);
                    PPP_ERROR_CHECK(err, continue);
                }
            }

            err |= _at_and_expect(esp_ppp, "ATDT*99***1#\r\n", "CONNECT", PPP_DEFAULT_CMD_TIMEOUT, PPP_DEFAULT_CMD_RETRY);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Error init gms");
                esp_ppp->state = PPP_STATE_WAIT_TIMEOUT;

            } else {
                esp_ppp->state = PPP_STATE_INIT;
            }

        }

        if (esp_ppp->state == PPP_STATE_INIT) {
            if (esp_ppp->ppp == NULL) {
                esp_ppp->ppp = pppapi_pppos_create(&esp_ppp->ppp_netif, ppp_output_callback, ppp_status_cb, esp_ppp);
            }
            if (esp_ppp->ppp == NULL) {
                ESP_LOGE(TAG, "Error init pppos");
                esp_ppp->state = PPP_STATE_WAIT_TIMEOUT;
            } else {
                esp_ppp->state = PPP_STATE_CONNECT;
            }
        }

        if (esp_ppp->state == PPP_STATE_CONNECT) {
            pppapi_set_default(esp_ppp->ppp);
            ppp_set_usepeerdns(esp_ppp->ppp, 1);

            ESP_LOGI(TAG, "After pppapi_set_default");

            pppapi_set_auth(esp_ppp->ppp, PPPAUTHTYPE_PAP, "mms", "mms");

            ESP_LOGI(TAG, "After pppapi_set_auth");

            pppapi_connect(esp_ppp->ppp, 0);

            ESP_LOGI(TAG, "After pppapi_connect");
            esp_ppp->state = PPP_STATE_IP;
            esp_ppp->rx_timeout_tick = esp_timer_get_time()/1000;
        }

        if (esp_ppp->state == PPP_STATE_IP) {
            memset(esp_ppp->buffer, 0, PPP_BUF_SIZE);
            int len = uart_read_bytes(esp_ppp->modem_conn.uart_num, (uint8_t *)esp_ppp->buffer, PPP_BUF_SIZE, 10 / portTICK_RATE_MS);
            if (len > 0) {
                ESP_LOGD(TAG, "<== PPP rx len %d", len);
                pppos_input_tcpip(esp_ppp->ppp, (u8_t *)esp_ppp->buffer, len);

                // Timeout if send and did not received after 10seconds
                if (/*len == 1 ||*/ (esp_ppp->rx_timeout_tick && esp_timer_get_time()/1000 - esp_ppp->rx_timeout_tick > 10000)) {
                    ESP_LOGW(TAG, "Send data without receive in 10 seconds, reset modem, %x", esp_ppp->buffer[0]);
                    esp_ppp->rx_timeout_tick = 0;
                    esp_ppp->state = PPP_STATE_WAIT_TIMEOUT;
                }

            }
            // Timeout if not connected after 30seconds
            if (!esp_ppp->connected && (esp_ppp->rx_timeout_tick && esp_timer_get_time()/1000 - esp_ppp->rx_timeout_tick > 30000)) {
                ESP_LOGW(TAG, "PPP connection timeout");
                esp_ppp->rx_timeout_tick = 0;
                esp_ppp->state = PPP_STATE_WAIT_TIMEOUT;
            }
        }

        if (esp_ppp->state == PPP_STATE_WAIT_TIMEOUT) {
            ESP_LOGI(TAG, "Waiting for timeout 10secs");
            esp_ppp->connected = false;
            if (esp_ppp->ppp) {
                ESP_LOGI(TAG, "Closing ppp");
                pppapi_close(esp_ppp->ppp, 0);
                xEventGroupWaitBits(esp_ppp->state_event, PPP_STOPPED_BIT, false, true, 10000 / portTICK_RATE_MS);

            }
            vTaskDelay(10000 / portTICK_RATE_MS);
            esp_ppp->state = PPP_STATE_UNINIT;
        }
    }
    xEventGroupSetBits(esp_ppp->state_event, STOPPED_BIT);
    xEventGroupClearBits(esp_ppp->state_event, STARTED_BIT);
    vTaskDelete(NULL);
}

static esp_err_t esp_ppp_destroy(esp_ppp_handle_t esp_ppp)
{
    _ppp_stop_task(esp_ppp);

    if (esp_ppp->ppp) {
        pppapi_close(esp_ppp->ppp, 0);
        pppapi_free(esp_ppp->ppp);
        esp_ppp->ppp = NULL;
    }

    uart_driver_delete(esp_ppp->modem_conn.uart_num);
    vEventGroupDelete(esp_ppp->state_event);
    free(esp_ppp->buffer);
    free(esp_ppp);
    return ESP_OK;
}

esp_ppp_handle_t esp_ppp_init(esp_ppp_cfg_t* ppp_cfg)
{
    esp_ppp_handle_t esp_ppp = calloc(1, sizeof(esp_ppp_t));

    memcpy(&esp_ppp->modem_conn, &ppp_cfg->modem_conn, sizeof(esp_ppp_modem_conn_t));
    if (esp_ppp->modem_conn.uart_num < 0) {
        esp_ppp->modem_conn.uart_num = PPP_DEFAULT_UART;
    }

    if (esp_ppp->modem_conn.tx_pin < 0) {
        esp_ppp->modem_conn.tx_pin = PPP_DEFAULT_TX_PIN;
    }

    if (esp_ppp->modem_conn.rx_pin < 0) {
        esp_ppp->modem_conn.rx_pin = PPP_DEFAULT_RX_PIN;
    }
    esp_ppp->task_stack = ppp_cfg->task_stack;
    if (esp_ppp->task_stack <= 0) {
        esp_ppp->task_stack = PPP_DEFAULT_TASK_STACK;
    }
    esp_ppp->task_prio = ppp_cfg->task_prio;
    if (esp_ppp->task_prio <= 0) {
        esp_ppp->task_prio = PPP_DEFAULT_TASK_PRIO;
    }
    esp_ppp->state_event = xEventGroupCreate();
    esp_ppp->connected = false;
    xEventGroupClearBits(esp_ppp->state_event, STARTED_BIT);
    xEventGroupSetBits(esp_ppp->state_event, STOPPED_BIT);
    g_ppp = esp_ppp;
    esp_ppp->buffer = (char *) malloc(PPP_BUF_SIZE);

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    //Configure UART1 parameters
    uart_param_config(esp_ppp->modem_conn.uart_num, &uart_config);

    ESP_LOGI(TAG, "Configuring UART%d GPIOs: TX:%d RX:%d RTS:%d CTS: %d",
             esp_ppp->modem_conn.uart_num,
             esp_ppp->modem_conn.tx_pin, esp_ppp->modem_conn.rx_pin, -1, -1);
    uart_set_pin(esp_ppp->modem_conn.uart_num, esp_ppp->modem_conn.tx_pin, esp_ppp->modem_conn.rx_pin, -1, -1);
    uart_driver_install(esp_ppp->modem_conn.uart_num, PPP_BUF_SIZE * 2, PPP_BUF_SIZE * 2, 0, NULL, 0);

    // uart_isr_register(esp_ppp->modem_conn.uart_num, uart_int_handler, self, int intr_alloc_flags,  uart_isr_handle_t *handle);

    if (esp_ppp->modem_conn.reset_pin) {
        gpio_config_t io_conf;
        //disable interrupt
        io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
        //set as output mode
        io_conf.mode = GPIO_MODE_OUTPUT;
        //bit mask of the pins that you want to set,e.g.GPIO18/19
        io_conf.pin_bit_mask = (1ULL << esp_ppp->modem_conn.reset_pin);
        //disable pull-down mode
        io_conf.pull_down_en = 0;
        //disable pull-up mode
        io_conf.pull_up_en = 1;
        //configure GPIO with the given settings
        gpio_config(&io_conf);
    }

    _ppp_stop_task(esp_ppp);

    esp_ppp->run = true;

    if (xTaskCreate(_ppp_task, "ppp_task", esp_ppp->task_stack, esp_ppp, esp_ppp->task_prio, NULL) != pdTRUE) {
        ESP_LOGE(TAG, "Error create console task, memory exhausted?");
        return ESP_FAIL;
    }
    return esp_ppp;
}

bool esp_ppp_is_connected(esp_ppp_handle_t esp_ppp)
{
    if (esp_ppp == NULL) {
        esp_ppp = g_ppp;
    }
    return esp_ppp->connected;
}
// #endif
