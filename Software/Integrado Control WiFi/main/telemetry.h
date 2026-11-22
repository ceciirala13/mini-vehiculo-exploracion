/**
 * @file telemetry.h
 * @brief Rover telemetry snapshot (sensors + motion) and JSON rendering for the web clients.
 *
 * On the single-board rover the data is read directly from vehicle_state and
 * motion_control; there is no link to go stale.
 */

#pragma once

#include <stddef.h>
#include "rover_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Build a fresh snapshot from vehicle_state + motion_control. Safe from any task. */
void telemetry_build(rover_telemetry_t *out);

/**
 * @brief Render the full GUI telemetry JSON (sensors, motion, effective drive
 *        command, system and camera status).
 * @return Length written (excluding NUL), 0 if the buffer is too small.
 */
size_t telemetry_to_json(char *buf, size_t size);

#ifdef __cplusplus
}
#endif
