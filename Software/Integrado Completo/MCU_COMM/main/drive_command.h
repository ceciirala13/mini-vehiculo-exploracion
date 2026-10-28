/**
 * @file drive_command.h
 * @brief Latest drive command from web/Python clients + second-level fail-safe.
 *
 * Clients (WebSocket, /api/control) call drive_command_set() continuously
 * (the GUI sends at 20 Hz). If nothing arrives for GUI_TIMEOUT_MS the command
 * read by the UART TX task becomes neutral (speed 0, steering 0).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "rover_proto.h"

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

/** Same, already encoded as a protocol payload. */
void drive_command_get_payload(rp_cmd_drive_t *out);

#ifdef __cplusplus
}
#endif
