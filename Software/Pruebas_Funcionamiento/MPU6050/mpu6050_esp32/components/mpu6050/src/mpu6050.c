/********************************************************************************************
 * Project: MPU6050 ESP32 Sensor Interface
 * Author: Muhammad Idrees (Updated for ESP-IDF v5.x i2c_master driver)
 * 
 * Description:
 * This source file implements the functions required to interface with the MPU6050 sensor
 * using the modern ESP-IDF driver/i2c_master.h driver.
 ********************************************************************************************/

#include "mpu6050.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ACCEL_SCALE 16384.0f // for ±2g range
#define GYRO_SCALE 131.0f    // for ±250°/s range
#define GRAVITY 9.8f         // m/s²

static float accel_bias[3] = {1.12f, -0.30f, -0.55f}; // Biases for accel X, Y, Z
static float gyro_bias[3] = {-0.27f, 0.24f, 0.16f};   // Biases for gyro X, Y, Z

esp_err_t mpu6050_init(i2c_master_dev_handle_t dev_handle) {
    // Wake up the MPU6050: write 0x00 to PWR_MGMT_1 (0x6B) to clear sleep bit
    uint8_t wake_cmd[2] = {MPU6050_REG_PWR_MGMT_1, 0x00};
    esp_err_t ret = i2c_master_transmit(dev_handle, wake_cmd, sizeof(wake_cmd), 1000);
    if (ret != ESP_OK) {
        return ret;
    }

    return ESP_OK;
}

esp_err_t mpu6050_read_raw_data(i2c_master_dev_handle_t dev_handle,
                               int16_t *accel_x, int16_t *accel_y, int16_t *accel_z,
                               int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
    uint8_t reg_addr = MPU6050_REG_ACCEL_XOUT_H;
    uint8_t data[14];
    esp_err_t ret;

    // Transmit the starting register address, then read 14 contiguous bytes
    ret = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, sizeof(data), 1000);
    if (ret != ESP_OK) {
        return ret;
    }

    *accel_x = (int16_t)((data[0] << 8) | data[1]);
    *accel_y = (int16_t)((data[2] << 8) | data[3]);
    *accel_z = (int16_t)((data[4] << 8) | data[5]);
    *gyro_x  = (int16_t)((data[8] << 8) | data[9]);
    *gyro_y  = (int16_t)((data[10] << 8) | data[11]);
    *gyro_z  = (int16_t)((data[12] << 8) | data[13]);

    return ESP_OK;
}

void mpu6050_convert_accel(int16_t raw_x, int16_t raw_y, int16_t raw_z, float *accel_x, float *accel_y, float *accel_z) {
    *accel_x = ((raw_x / ACCEL_SCALE) * GRAVITY) - accel_bias[0];
    *accel_y = ((raw_y / ACCEL_SCALE) * GRAVITY) - accel_bias[1];
    *accel_z = ((raw_z / ACCEL_SCALE) * GRAVITY) - accel_bias[2];
}

void mpu6050_convert_gyro(int16_t raw_x, int16_t raw_y, int16_t raw_z, float *gyro_x, float *gyro_y, float *gyro_z) {
    *gyro_x = (raw_x / GYRO_SCALE) - gyro_bias[0];
    *gyro_y = (raw_y / GYRO_SCALE) - gyro_bias[1];
    *gyro_z = (raw_z / GYRO_SCALE) - gyro_bias[2];
}

void mpu6050_calibrate(i2c_master_dev_handle_t dev_handle, float *accel_bias_out, float *gyro_bias_out) {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    float accel_x_g, accel_y_g, accel_z_g;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;
    float accel_x_sum = 0.0f, accel_y_sum = 0.0f, accel_z_sum = 0.0f;
    float gyro_x_sum = 0.0f, gyro_y_sum = 0.0f, gyro_z_sum = 0.0f;
    int samples = 100;

    for (int i = 0; i < samples; i++) {
        if (mpu6050_read_raw_data(dev_handle, &accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z) == ESP_OK) {
            mpu6050_convert_accel(accel_x, accel_y, accel_z, &accel_x_g, &accel_y_g, &accel_z_g);
            mpu6050_convert_gyro(gyro_x, gyro_y, gyro_z, &gyro_x_dps, &gyro_y_dps, &gyro_z_dps);

            accel_x_sum += accel_x_g;
            accel_y_sum += accel_y_g;
            accel_z_sum += accel_z_g;
            gyro_x_sum += gyro_x_dps;
            gyro_y_sum += gyro_y_dps;
            gyro_z_sum += gyro_z_dps;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // Compute biases
    accel_bias_out[0] = accel_x_sum / samples;
    accel_bias_out[1] = accel_y_sum / samples;
    accel_bias_out[2] = (accel_z_sum / samples) - GRAVITY; // Gravity should be accounted for on Z-axis

    gyro_bias_out[0] = gyro_x_sum / samples;
    gyro_bias_out[1] = gyro_y_sum / samples;
    gyro_bias_out[2] = gyro_z_sum / samples;
}
