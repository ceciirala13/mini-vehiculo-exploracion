/**
 * @file uart_link.h
 * @brief UART link to MCU_CONTROL.
 *
 * RX task: drains the IDF UART ring buffer in chunks, feeds the rover_proto
 *          parser and stores TLM_STATE frames in telemetry_store.
 * TX task: sends the current drive command (drive_command.c) as CMD_DRIVE at a
 *          fixed 20 Hz, independent of how often the browser sends; this frame
 *          is also the heartbeat that keeps MCU_CONTROL out of FAILSAFE.
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
} uart_link_stats_t;

esp_err_t uart_link_init(void);

void uart_link_get_stats(uart_link_stats_t *out);

#ifdef __cplusplus
}
#endif
