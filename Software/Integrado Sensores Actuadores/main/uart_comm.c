/**
 * @file uart_comm.c
 * @brief UART telemetry communication implementation.
 */

#include "uart_comm.h"
#include "project_config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "UART_COMM";

static uint16_t compute_checksum(const uint8_t *data, size_t length)
{
    uint16_t sum = 0;
    for (size_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return sum;
}

esp_err_t uart_comm_init(void)
{
    ESP_LOGI(TAG, "Initializing UART Port %d (TX=GPIO%d, RX=GPIO%d, Baud=%d)...",
             TELEMETRY_UART_PORT, TELEMETRY_UART_TX_PIN, TELEMETRY_UART_RX_PIN, TELEMETRY_UART_BAUD_RATE);

    uart_config_t uart_cfg = {
        .baud_rate  = TELEMETRY_UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_param_config(TELEMETRY_UART_PORT, &uart_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(TELEMETRY_UART_PORT,
                       TELEMETRY_UART_TX_PIN,
                       TELEMETRY_UART_RX_PIN,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_driver_install(TELEMETRY_UART_PORT, TELEMETRY_UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "UART driver installed successfully.");
    return ESP_OK;
}

esp_err_t uart_comm_send_telemetry(telemetry_packet_t *packet)
{
    if (!packet) return ESP_ERR_INVALID_ARG;

    // Set framing delimiters
    packet->header[0] = TELEMETRY_FRAME_START_0;
    packet->header[1] = TELEMETRY_FRAME_START_1;
    packet->tail[0]   = TELEMETRY_FRAME_END_0;
    packet->tail[1]   = TELEMETRY_FRAME_END_1;

    // Calculate checksum over data payload (excluding header, checksum, and tail)
    const uint8_t *payload_ptr = (const uint8_t *)&packet->timestamp_ms;
    size_t payload_len = sizeof(telemetry_packet_t) - sizeof(packet->header) - sizeof(packet->checksum) - sizeof(packet->tail);
    packet->checksum = compute_checksum(payload_ptr, payload_len);

    int bytes_sent = uart_write_bytes(TELEMETRY_UART_PORT, (const char *)packet, sizeof(telemetry_packet_t));
    if (bytes_sent != sizeof(telemetry_packet_t)) {
        ESP_LOGE(TAG, "Failed to send full telemetry frame (%d/%u bytes)", bytes_sent, (unsigned int)sizeof(telemetry_packet_t));
        return ESP_FAIL;
    }

    return ESP_OK;
}
