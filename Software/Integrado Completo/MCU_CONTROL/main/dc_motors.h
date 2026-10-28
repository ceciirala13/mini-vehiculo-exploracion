/**
 * @file dc_motors.h
 * @brief Low-level MCPWM driver for two DC motors on DRV8873H H-bridges.
 *
 * This layer only knows "M1" and "M2" and signed duty in percent.
 * Wheel mapping, kinematics and ramps live in motion_control.c.
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DC_MOTOR_1 = 0,
    DC_MOTOR_2 = 1,
} dc_motor_id_t;

/**
 * @brief Initialize MCPWM timer/operators/generators and GPIOs.
 *        Outputs start DISABLED (DISABLE pins high) with 0 duty.
 */
esp_err_t dc_motors_init(void);

/**
 * @brief Set signed duty for both motors.
 * @param duty_m1 -100..100 % (+ = IN1 PWM, - = IN2 PWM), clamped.
 * @param duty_m2 -100..100 %
 */
esp_err_t dc_motors_set_speed(float duty_m1, float duty_m2);

/**
 * @brief Both inputs low on both bridges (0 duty).
 */
esp_err_t dc_motors_stop(void);

/**
 * @brief Enable/disable the H-bridge outputs via the DISABLE pins.
 *        Disabled = outputs Hi-Z (motors coast). Duty is reset to 0 when disabling.
 */
void dc_motors_enable(bool enable);

bool dc_motors_is_enabled(void);

/**
 * @brief true if nFAULT of the given motor is asserted (low).
 */
bool dc_motors_fault(dc_motor_id_t motor);

/**
 * @brief true if either nFAULT is asserted.
 */
bool dc_motors_check_fault(void);

/**
 * @brief Last applied duty values (%).
 */
void dc_motors_get_speeds(float *duty_m1, float *duty_m2);

#ifdef __cplusplus
}
#endif
