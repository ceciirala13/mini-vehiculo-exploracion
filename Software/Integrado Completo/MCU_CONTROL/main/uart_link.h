/**
 * @file uart_link.h
 * @brief UART link to MCU_COMM: non-blocking RX parser task + framed TX.
 *
 * RX: the IDF UART driver buffers bytes in its ISR-fed ring buffer; a dedicated
 *     task drains it in chunks and feeds the rover_proto parser. Valid
 *     CMD_DRIVE frames are forwarded to motion_control_submit().
 * TX: frames are written into the driver TX ring buffer under a mutex.
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t rx_bytes;      /* raw bytes seen on RX, valid or not */
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t len_errors;
    uint32_t timeouts;
    uint32_t unknown_msgs;
    uint32_t tx_frames;
    uint32_t tx_drops;
} uart_link_stats_t;

/**
 * @brief Install UART driver and start the RX task.
 */
esp_err_t uart_link_init(void);

/**
 * @brief Encode and transmit one frame (thread-safe, never blocks > 10 ms).
 */
esp_err_t uart_link_send(uint8_t msg_id, const void *payload, uint8_t len);

void uart_link_get_stats(uart_link_stats_t *out);

#ifdef __cplusplus
}
#endif
