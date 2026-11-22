/**
 * @file bme690.h
 * @brief Bosch BME690 Environmental Sensor driver wrapper (I2C Bus 1).
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float    temperature_c;
    float    humidity_pct;
    float    pressure_hpa;
    uint32_t gas_resistance_ohm;
    uint8_t  gas_valid;
    uint8_t  heater_stable;
} bme690_sensor_data_t;

/**
 * @brief Initialize dedicated I2C master bus 1, configure Bosch BME690 sensor,
 * oversampling, and gas heater profile.
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t bme690_sensor_init(void);

/**
 * @brief Trigger measurement in forced mode and read compensated environmental data.
 * @param[out] data Output data structure.
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t bme690_sensor_read(bme690_sensor_data_t *data);

#ifdef __cplusplus
}
#endif
