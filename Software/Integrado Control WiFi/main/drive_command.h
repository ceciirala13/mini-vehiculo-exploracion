/**
 * @file drive_command.h
 * @brief Latest drive command from web/Python clients, GUI watchdog and the
 *        bridge task that feeds motion_control.
 *
 * Clients (WebSocket, /api/control) call drive_command_set() continuously
 * (the GUI sends at 20 Hz). If nothing arrives for GUI_TIMEOUT_MS the
 * effective command becomes neutral (speed 0, steering 0).
 *
 * The bridge task submits the effective command to motion_control at a fixed
 * 20 Hz, whatever the clients do. motion_control goes to FAILSAFE if the
 * bridge ever stalls.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float    speed_pct;
    float    steer_deg;
    bool     estop;         /* e-stop latched (held for DRIVE_ESTOP_HOLD_MS) */
    bool     client_active; /* a client sent a command within GUI_TIMEOUT_MS */
    uint32_t age_ms;        /* time since the last client command */
} drive_command_state_t;

void drive_command_init(void);

/** Update from a client. Values are clamped to app_config.h limits. */
void drive_command_set(float speed_pct, float steer_deg);

/** Emergency stop: sends the E-STOP flag and neutral for DRIVE_ESTOP_HOLD_MS. */
void drive_command_estop(void);

/** Effective command (fail-safe applied). */
void drive_command_get(drive_command_state_t *out);

/** Start the fixed-rate bridge task (drive_command -> motion_control_submit). */
esp_err_t drive_command_start_bridge(void);

#ifdef __cplusplus
}
#endif
