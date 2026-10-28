/**
 * @file uart_link.c
 * @brief UART link to MCU_CONTROL.
 */

#include "uart_link.h"
#include "app_config.h"
#include "rover_proto.h"
#include "drive_command.h"
#include "telemetry_store.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "UART_LINK";

static rp_parser_t       s_parser;
static portMUX_TYPE      s_stats_lock = portMUX_INITIALIZER_UNLOCKED;
static uart_link_stats_t s_stats;

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static void handle_frame(const rp_frame_t *f)
{
    if (f->msg_id == RP_MSG_TLM_STATE && f->len == sizeof(rp_tlm_state_t)) {
        rp_tlm_state_t t;
        memcpy(&t, f->payload, sizeof(t));
        telemetry_store_update(&t);
        return;
    }
    taskENTER_CRITICAL(&s_stats_lock);
    s_stats.unknown_msgs++;
    taskEXIT_CRITICAL(&s_stats_lock);
}

static void uart_rx_task(void *arg)
{
    (void)arg;
    uint8_t buf[128];

    while (1) {
        int n = uart_read_bytes(LINK_UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(10));
        if (n <= 0) {
            continue;
        }
        uint32_t t = now_ms();
        for (int i = 0; i < n; i++) {
            if (rp_parser_feed(&s_parser, buf[i], t)) {
                handle_frame(&s_parser.frame);
            }
        }
        taskENTER_CRITICAL(&s_stats_lock);
        s_stats.rx_bytes  += (uint32_t)n;
        s_stats.frames_ok  = s_parser.frames_ok;
        s_stats.crc_errors = s_parser.crc_errors;
        s_stats.len_errors = s_parser.len_errors;
        s_stats.timeouts   = s_parser.timeouts;
        taskEXIT_CRITICAL(&s_stats_lock);
    }
}

static void drive_tx_task(void *arg)
{
    (void)arg;
    uint8_t frame[RP_MAX_FRAME];
    uint8_t seq = 0;
    uint32_t log_div = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        rp_cmd_drive_t cmd;
        drive_command_get_payload(&cmd);

        size_t n = rp_encode(RP_MSG_CMD_DRIVE, seq++, &cmd, sizeof(cmd), frame, sizeof(frame));
        if (n > 0 && uart_write_bytes(LINK_UART_PORT, (const char *)frame, n) == (int)n) {
            taskENTER_CRITICAL(&s_stats_lock);
            s_stats.tx_frames++;
            taskEXIT_CRITICAL(&s_stats_lock);
        }

        /* link health every 2 s: rx bytes = 0 means nothing arrives on the RX pin */
        if (++log_div >= 2000 / TASK_DRIVE_TX_PERIOD_MS) {
            log_div = 0;
            uart_link_stats_t s;
            uart_link_get_stats(&s);
            ESP_LOGI(TAG, "tx frames:%lu | rx bytes:%lu ok:%lu crc:%lu len:%lu",
                     (unsigned long)s.tx_frames, (unsigned long)s.rx_bytes,
                     (unsigned long)s.frames_ok, (unsigned long)s.crc_errors,
                     (unsigned long)s.len_errors);
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_DRIVE_TX_PERIOD_MS));
    }
}

esp_err_t uart_link_init(void)
{
    ESP_LOGI(TAG, "UART%d TX=GPIO%d RX=GPIO%d @ %d baud",
             LINK_UART_PORT, LINK_UART_TX_PIN, LINK_UART_RX_PIN, LINK_UART_BAUD_RATE);

    rp_parser_init(&s_parser);
    memset(&s_stats, 0, sizeof(s_stats));

    uart_config_t cfg = {
        .baud_rate  = LINK_UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t ret = uart_driver_install(LINK_UART_PORT, LINK_UART_RX_BUF_SIZE, LINK_UART_TX_BUF_SIZE, 0, NULL, 0);
    if (ret != ESP_OK) return ret;
    ret = uart_param_config(LINK_UART_PORT, &cfg);
    if (ret != ESP_OK) return ret;
    ret = uart_set_pin(LINK_UART_PORT, LINK_UART_TX_PIN, LINK_UART_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) return ret;

    if (xTaskCreatePinnedToCore(uart_rx_task, "uart_rx", TASK_UART_RX_STACK, NULL,
                                TASK_UART_RX_PRIO, NULL, TASK_UART_RX_CORE) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCore(drive_tx_task, "drive_tx", TASK_DRIVE_TX_STACK, NULL,
                                TASK_DRIVE_TX_PRIO, NULL, TASK_DRIVE_TX_CORE) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void uart_link_get_stats(uart_link_stats_t *out)
{
    taskENTER_CRITICAL(&s_stats_lock);
    *out = s_stats;
    taskEXIT_CRITICAL(&s_stats_lock);
}
