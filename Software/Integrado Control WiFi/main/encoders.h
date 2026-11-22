/**
 * @file encoders.h
 * @brief Quadrature wheel encoders on the S3 pulse counter (PCNT).
 *
 * Hall encoders, 11 PPR on the motor shaft, 1:45 gearbox. Both edges of both
 * phases are counted (x4), so one wheel revolution = 11 * 4 * 45 = 1980 counts.
 * Counting is done in hardware; the CPU only reads the totals.
 *
 * encoders_update() must be called at a fixed rate by a single task (the
 * motion task does it every TASK_MOTION_PERIOD_MS). Speeds are computed over
 * the last ENC_SPEED_WINDOW_SAMPLES samples. encoders_get() is safe from any task.
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool    ok;                 /* both PCNT units running */
    int32_t count_left;         /* accumulated counts since boot, + = forward */
    int32_t count_right;
    float   rpm_left;           /* wheel (gearbox output) speed, + = forward */
    float   rpm_right;
    float   mps_left;           /* wheel surface speed, m/s */
    float   mps_right;
    float   dist_m;             /* distance travelled since boot (average of both wheels), m */
} encoders_state_t;

/**
 * @brief Configure the GPIOs and both PCNT units and start counting.
 *        Call early: it also stops GPIO43 being driven as UART0 TX.
 */
esp_err_t encoders_init(void);

/** Sample the counters and update speeds/distance (one task, fixed rate). */
void encoders_update(void);

/** Thread-safe copy of the latest values. */
void encoders_get(encoders_state_t *out);

/** ENC_DIAG only: log edges seen on each diagnostic pin and the level of all encoder pins. */
void encoders_diag_log(void);

#ifdef __cplusplus
}
#endif
