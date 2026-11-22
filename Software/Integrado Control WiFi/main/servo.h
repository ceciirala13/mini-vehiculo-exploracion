/**
 * @file servo.h
 * @brief LEDC PWM driver for the steering servo (continuous angle).
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize LEDC timer/channel and center the steering.
 */
esp_err_t servo_init(void);

/**
 * @brief Set raw servo angle (0..180 deg, clamped).
 */
esp_err_t servo_set_angle_deg(float angle_deg);

/**
 * @brief Set steering angle relative to straight ahead.
 * @param steer_deg -SERVO_MAX_STEER_DEG..+SERVO_MAX_STEER_DEG (+ = right), clamped.
 *        Applies SERVO_CENTER_DEG trim and SERVO_INVERT.
 */
esp_err_t steering_set_deg(float steer_deg);

/**
 * @brief Last applied steering angle (deg, + = right).
 */
float steering_get_deg(void);

#ifdef __cplusplus
}
#endif
