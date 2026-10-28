/********************************************************************************************
 * Project: MPU6050 ESP32-S3 Sensor Interface
 * Author: Muhammad Idrees (Updated for ESP32-S3 & ESP-IDF v5.x i2c_master driver)
 * 
 * Description:
 * Initializes the ESP32-S3 I2C bus using GPIO 40 (SDA) and GPIO 42 (SCL) with the
 * modern driver/i2c_master.h driver, reads MPU6050 data and calculates roll, pitch, and yaw.
 ********************************************************************************************/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "mpu6050.h"
#include "roll_pitch.h"
#include "quaternions.h"

#define TAG "MAIN"

#define I2C_MASTER_SDA_IO    40  // SDA 
#define I2C_MASTER_SCL_IO    41  // SCL 
#define I2C_MASTER_FREQ_HZ   400000

void app_main(void) {
    esp_err_t ret;
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    float accel_x_g, accel_y_g, accel_z_g;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;
    float accel_bias[3] = {0.00f, 0.00f, 0.00f};
    float gyro_bias[3] = {0.00f, 0.00f, 0.00f};

    ESP_LOGI(TAG, "Initializing I2C Master Bus on SDA=GPIO%d, SCL=GPIO%d...", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);

    // 1. Initialize I2C Master Bus
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ret = i2c_new_master_bus(&bus_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        return;
    }

    // 2. Add MPU6050 device to the bus
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    i2c_master_dev_handle_t mpu6050_handle;
    ret = i2c_master_bus_add_device(bus_handle, &dev_config, &mpu6050_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add MPU6050 device to bus: %s", esp_err_to_name(ret));
        return;
    }

    // 3. Initialize MPU6050
    ret = mpu6050_init(mpu6050_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 Initialization failed: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "Check connections: VCC->3.3V, GND->GND, SDA->GPIO%d, SCL->GPIO%d, AD0->GND", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
        return;
    }
    ESP_LOGI(TAG, "MPU6050 successfully initialized!");

    // 4. Calibrate the MPU6050
    ESP_LOGI(TAG, "Calibrating MPU6050... Keep the sensor still.");
    mpu6050_calibrate(mpu6050_handle, accel_bias, gyro_bias);
    ESP_LOGI(TAG, "Calibration complete.");

    // Initialize roll and pitch calculations
    roll_pitch_init();

    // Initialize Quaternion
    Quaternion q;
    quaternion_init(&q);

    while (1) {
        // Read raw data
        ret = mpu6050_read_raw_data(mpu6050_handle, &accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "MPU6050 read failed: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Convert raw data to physical values
        mpu6050_convert_accel(accel_x, accel_y, accel_z, &accel_x_g, &accel_y_g, &accel_z_g);
        mpu6050_convert_gyro(gyro_x, gyro_y, gyro_z, &gyro_x_dps, &gyro_y_dps, &gyro_z_dps);

        // Update roll and pitch
        roll_pitch_update(accel_x_g, accel_y_g, accel_z_g, gyro_x_dps, gyro_y_dps, gyro_z_dps);

        // Update Quaternion
        quaternion_update(&q, gyro_x_dps, gyro_y_dps, gyro_z_dps, accel_x_g, accel_y_g, accel_z_g, 0.01f);

        // Print results
        printf("Accel: X=%0.2f m/s^2, Y=%0.2f m/s^2, Z=%0.2f m/s^2 | Gyro: X=%0.2f, Y=%0.2f, Z=%0.2f deg/s\n",
               accel_x_g, accel_y_g, accel_z_g, gyro_x_dps, gyro_y_dps, gyro_z_dps);
        printf("Angles -> Roll: %0.2f°, Pitch: %0.2f° | Quaternion -> Roll: %0.2f°, Pitch: %0.2f°, Yaw: %0.2f°\n",
               roll_get(), pitch_get(), quaternion_get_roll(&q), quaternion_get_pitch(&q), quaternion_get_yaw(&q));

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
