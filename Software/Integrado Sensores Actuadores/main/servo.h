/**
 * @file servo.h
 * @brief LEDC PWM controller for servo motor (0°, 90°, 180°).
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SERVO_POS_0_DEG   = 0,
    SERVO_POS_90_DEG  = 90,
    SERVO_POS_180_DEG = 180
} servo_angle_t;

/**
 * @brief Initialize LEDC timer and channel for servo motor control.
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t servo_init(void);

/**
 * @brief Set servo position to specific angle (0, 90, 180 degrees).
 * @param angle Target angle enum.
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t servo_set_angle(servo_angle_t angle);

/**
 * @brief Get the last commanded servo angle.
 * @return Commanded angle in degrees.
 */
uint16_t servo_get_current_angle(void);

#ifdef __cplusplus
}
#endif
