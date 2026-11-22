/**
 * @file mpu6050.h
 * @brief MPU6050 6-axis IMU driver wrapper using ESP-IDF v5 driver/i2c_master.h on dedicated I2C Bus 0.
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} mpu6050_data_t;

/**
 * @brief Initialize dedicated I2C master bus 0 and configure MPU6050.
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t mpu6050_sensor_init(void);

/**
 * @brief Read calibrated/scaled accelerometer and gyroscope data.
 * @param[out] data Structure to receive sensor measurements.
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t mpu6050_sensor_read(mpu6050_data_t *data);

#ifdef __cplusplus
}
#endif
