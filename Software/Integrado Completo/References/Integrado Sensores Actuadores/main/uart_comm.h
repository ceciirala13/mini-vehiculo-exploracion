/**
 * @file uart_comm.h
 * @brief UART telemetry communication to companion microcontroller.
 */

#pragma once

#include "esp_err.h"
#include "project_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize UART port and driver with configured TX/RX pins and baud rate.
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t uart_comm_init(void);

/**
 * @brief Calculate CRC/checksum and transmit telemetry packet over UART.
 * @param packet Pointer to packet payload.
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t uart_comm_send_telemetry(telemetry_packet_t *packet);

#ifdef __cplusplus
}
#endif
