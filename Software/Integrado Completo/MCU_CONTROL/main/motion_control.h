/**
 * @file motion_control.h
 * @brief Motion controller: the single owner of the motors and steering servo.
 *
 * Pipeline executed every TASK_MOTION_PERIOD_MS:
 *
 *   latest command (mailbox)
 *     -> safety state machine (DISARMED / ARMED / FAILSAFE / FAULT)
 *     -> slew-rate limiter on vehicle speed and steering (smooth accel/decel)
 *     -> kinematics (Ackermann + electronic differential)
 *     -> dead-band / minimum duty compensation
 *     -> wheel-to-H-bridge mapping and polarity
 *     -> dc_motors / servo
 *
 * Each stage is a separate static function in motion_control.c so it can be
 * changed without touching the others. Tunables live in app_config.h.
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include "rover_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Command coming from any source (UART link, test code...). */
typedef struct {
    float speed_pct;        /* -100..100 % (+ = forward) */
    float steer_deg;        /* + = right */
    bool  estop;
} motion_cmd_t;

typedef struct {
    rp_motion_state_t state;
    bool  link_ok;          /* fresh command available (< LINK_TIMEOUT_MS) */
    bool  fault;            /* DRV8873 nFAULT asserted */
    float speed_pct;        /* ramped vehicle speed */
    float steer_deg;        /* ramped steering */
    float left_pct;         /* applied left wheel duty (after compensation) */
    float right_pct;        /* applied right wheel duty */
} motion_status_t;

/**
 * @brief Create the command mailbox. Call after dc_motors_init() and servo_init().
 */
esp_err_t motion_control_init(void);

/**
 * @brief Start the periodic motion task (pinned per app_config.h).
 */
esp_err_t motion_control_start(void);

/**
 * @brief Submit the newest command (overwrites any pending one). Safe from any task.
 */
void motion_control_submit(const motion_cmd_t *cmd);

/**
 * @brief Thread-safe copy of the controller status.
 */
void motion_control_get_status(motion_status_t *out);

#ifdef __cplusplus
}
#endif
