/**
 * @file uart_link.c
 * @brief UART link to MCU_COMM.
 */

#include "uart_link.h"
#include "app_config.h"
#include "rover_proto.h"
#include "motion_control.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "UART_LINK";

static SemaphoreHandle_t s_tx_mutex = NULL;
static uint8_t           s_tx_seq = 0;
static rp_parser_t       s_parser;
static portMUX_TYPE      s_stats_lock = portMUX_INITIALIZER_UNLOCKED;
static uart_link_stats_t s_stats;

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static void handle_frame(const rp_frame_t *f)
{
    switch (f->msg_id) {
    case RP_MSG_CMD_DRIVE: {
        if (f->len != sizeof(rp_cmd_drive_t)) {
            break;
        }
        rp_cmd_drive_t d;
        memcpy(&d, f->payload, sizeof(d));
        motion_cmd_t cmd = {
            .speed_pct = d.speed_x10 / 10.0f,
            .steer_deg = d.steer_x10 / 10.0f,
            .estop     = (d.flags & RP_DRIVE_FLAG_ESTOP) != 0,
        };
        motion_control_submit(&cmd);
        return;
    }
    case RP_MSG_CMD_CONFIG:
        /* Reserved for runtime tuning */
        return;
    default:
        break;
    }

    taskENTER_CRITICAL(&s_stats_lock);
    s_stats.unknown_msgs++;
    taskEXIT_CRITICAL(&s_stats_lock);
}

static void uart_rx_task(void *arg)
{
    (void)arg;
    uint8_t buf[128];

    ESP_LOGI(TAG, "RX task started");
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

esp_err_t uart_link_init(void)
{
    ESP_LOGI(TAG, "UART%d TX=GPIO%d RX=GPIO%d @ %d baud",
             LINK_UART_PORT, LINK_UART_TX_PIN, LINK_UART_RX_PIN, LINK_UART_BAUD_RATE);

    rp_parser_init(&s_parser);
    memset(&s_stats, 0, sizeof(s_stats));

    s_tx_mutex = xSemaphoreCreateMutex();
    if (s_tx_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    uart_config_t cfg = {
        .baud_rate  = LINK_UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_driver_install(LINK_UART_PORT, LINK_UART_RX_BUF_SIZE, LINK_UART_TX_BUF_SIZE, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = uart_param_config(LINK_UART_PORT, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = uart_set_pin(LINK_UART_PORT, LINK_UART_TX_PIN, LINK_UART_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(ret));
        return ret;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(uart_rx_task, "uart_rx", TASK_UART_RX_STACK, NULL,
                                            TASK_UART_RX_PRIO, NULL, TASK_UART_RX_CORE);
    return (ok == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t uart_link_send(uint8_t msg_id, const void *payload, uint8_t len)
{
    uint8_t frame[RP_MAX_FRAME];

    if (s_tx_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        taskENTER_CRITICAL(&s_stats_lock);
        s_stats.tx_drops++;
        taskEXIT_CRITICAL(&s_stats_lock);
        return ESP_ERR_TIMEOUT;
    }

    size_t n = rp_encode(msg_id, s_tx_seq++, payload, len, frame, sizeof(frame));
    int written = (n > 0) ? uart_write_bytes(LINK_UART_PORT, (const char *)frame, n) : -1;
    xSemaphoreGive(s_tx_mutex);

    taskENTER_CRITICAL(&s_stats_lock);
    if (written == (int)n && n > 0) s_stats.tx_frames++;
    else                            s_stats.tx_drops++;
    taskEXIT_CRITICAL(&s_stats_lock);

    return (written == (int)n && n > 0) ? ESP_OK : ESP_FAIL;
}

void uart_link_get_stats(uart_link_stats_t *out)
{
    taskENTER_CRITICAL(&s_stats_lock);
    *out = s_stats;
    taskEXIT_CRITICAL(&s_stats_lock);
}
