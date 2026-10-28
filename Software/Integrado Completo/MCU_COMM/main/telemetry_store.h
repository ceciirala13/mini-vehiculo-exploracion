/**
 * @file telemetry_store.h
 * @brief Latest MCU_CONTROL telemetry + JSON rendering for the web clients.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "rover_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

void telemetry_store_init(void);

/** Called by the UART RX task for every valid TLM_STATE frame. */
void telemetry_store_update(const rp_tlm_state_t *t);

/**
 * @brief Copy the latest telemetry.
 * @param age_ms Time since it was received (UINT32_MAX if never).
 * @return true if MCU_CONTROL is online (age <= CTRL_LINK_TIMEOUT_MS).
 */
bool telemetry_store_get(rp_tlm_state_t *out, uint32_t *age_ms);

/**
 * @brief Render the full GUI telemetry JSON (CTRL telemetry, link stats,
 *        effective drive command, camera status).
 * @return Length written (excluding NUL), 0 if the buffer is too small.
 */
size_t telemetry_store_to_json(char *buf, size_t size);

#ifdef __cplusplus
}
#endif
