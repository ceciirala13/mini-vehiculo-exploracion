/**
 * @file dc_motors.h
 * @brief MCPWM control for dual DC motors with direction, speed, and nFAULT detection.
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize MCPWM timers, operators, comparators, generators, and nFAULT GPIOs.
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t dc_motors_init(void);

/**
 * @brief Set speed for Motor 1 and Motor 2.
 * @param speed_m1 Speed percentage (-100.0f to 100.0f). Positive=FWD, Negative=REV, 0=Brake.
 * @param speed_m2 Speed percentage (-100.0f to 100.0f). Positive=FWD, Negative=REV, 0=Brake.
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t dc_motors_set_speed(float speed_m1, float speed_m2);

/**
 * @brief Stop both motors immediately (Brake).
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t dc_motors_stop(void);

/**
 * @brief Check if a fault is reported on either motor driver (nFAULT low).
 * @return true if fault is detected, false if operating normally.
 */
bool dc_motors_check_fault(void);

/**
 * @brief Get current speed setpoints.
 */
void dc_motors_get_speeds(float *speed_m1, float *speed_m2);

#ifdef __cplusplus
}
#endif
