/**
 * @file mpu6050.c
 * @brief MPU6050 driver implementation using ESP-IDF v5 driver/i2c_master.h on dedicated I2C Bus 0.
 */

#include "mpu6050.h"
#include "project_config.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "MPU6050";

#define REG_PWR_MGMT_1      0x6B
#define REG_ACCEL_XOUT_H    0x3B
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C

#define ACCEL_SCALE         16384.0f  // ±2g
#define GYRO_SCALE          131.0f    // ±250 deg/s

static i2c_master_bus_handle_t s_bus_handle = NULL;
static i2c_master_dev_handle_t s_dev_handle = NULL;
static bool s_initialized = false;

// Calibration bias offsets (can be calibrated dynamically if desired)
static float s_accel_bias[3] = {0.0f, 0.0f, 0.0f};
static float s_gyro_bias[3]  = {0.0f, 0.0f, 0.0f};

#include <math.h>
#include "esp_timer.h"

// Quaternion orientation structure
typedef struct {
    float w;
    float x;
    float y;
    float z;
} imu_quaternion_t;

#define BETA_COEFF 0.1f // Filter convergence rate

static imu_quaternion_t s_q = { .w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f };
static int64_t s_last_read_time_us = 0;

static float fast_inv_sqrt(float x) {
    return 1.0f / sqrtf(x);
}

static float wrap_angle(float angle) {
    if (angle > 180.0f) {
        return angle - 360.0f;
    } else if (angle < -180.0f) {
        return angle + 360.0f;
    }
    return angle;
}

static void quaternion_update(imu_quaternion_t *q, float gx_dps, float gy_dps, float gz_dps,
                              float ax, float ay, float az, float dt)
{
    // Convert gyro to radians/sec
    float gx = gx_dps * (M_PI / 180.0f);
    float gy = gy_dps * (M_PI / 180.0f);
    float gz = gz_dps * (M_PI / 180.0f);

    float norm = ax * ax + ay * ay + az * az;
    if (norm > 0.0001f) {
        norm = fast_inv_sqrt(norm);
        ax *= norm;
        ay *= norm;
        az *= norm;
    }

    // Rate of change of quaternion from gyroscope
    float qDot1 = 0.5f * (-q->x * gx - q->y * gy - q->z * gz);
    float qDot2 = 0.5f * ( q->w * gx + q->y * gz - q->z * gy);
    float qDot3 = 0.5f * ( q->w * gy - q->x * gz + q->z * gx);
    float qDot4 = 0.5f * ( q->w * gz + q->x * gy - q->y * gx);

    if (ax != 0.0f || ay != 0.0f || az != 0.0f) {
        float f1 = 2.0f * (q->x * q->z - q->w * q->y) - ax;
        float f2 = 2.0f * (q->w * q->x + q->y * q->z) - ay;
        float f3 = 2.0f * (0.5f - q->x * q->x - q->y * q->y) - az;

        float j11 = -2.0f * q->y;
        float j12 =  2.0f * q->z;
        float j13 = -2.0f * q->w;
        float j14 =  2.0f * q->x;
        float j21 =  2.0f * q->x;
        float j22 =  2.0f * q->w;
        float j23 =  2.0f * q->z;
        float j24 =  2.0f * q->y;
        float j31 =  0.0f;
        float j32 = -4.0f * q->x;
        float j33 = -4.0f * q->y;
        float j34 =  0.0f;

        float step1 = j11 * f1 + j21 * f2 + j31 * f3;
        float step2 = j12 * f1 + j22 * f2 + j32 * f3;
        float step3 = j13 * f1 + j23 * f2 + j33 * f3;
        float step4 = j14 * f1 + j24 * f2 + j34 * f3;

        float step_norm = step1 * step1 + step2 * step2 + step3 * step3 + step4 * step4;
        if (step_norm > 0.0001f) {
            step_norm = fast_inv_sqrt(step_norm);
            step1 *= step_norm;
            step2 *= step_norm;
            step3 *= step_norm;
            step4 *= step_norm;

            qDot1 -= BETA_COEFF * step1;
            qDot2 -= BETA_COEFF * step2;
            qDot3 -= BETA_COEFF * step3;
            qDot4 -= BETA_COEFF * step4;
        }
    }

    q->w += qDot1 * dt;
    q->x += qDot2 * dt;
    q->y += qDot3 * dt;
    q->z += qDot4 * dt;

    float q_norm = q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z;
    if (q_norm > 0.0001f) {
        q_norm = fast_inv_sqrt(q_norm);
        q->w *= q_norm;
        q->x *= q_norm;
        q->y *= q_norm;
        q->z *= q_norm;
    }
}

static void compute_euler_angles(const imu_quaternion_t *q, float *roll, float *pitch, float *yaw)
{
    *roll = atan2f(2.0f * (q->w * q->x + q->y * q->z), 1.0f - 2.0f * (q->x * q->x + q->y * q->y)) * 180.0f / M_PI;
    *roll = wrap_angle(*roll);

    float sinp = 2.0f * (q->w * q->y - q->z * q->x);
    if (fabsf(sinp) >= 1.0f) {
        sinp = copysignf(1.0f, sinp);
    }
    *pitch = asinf(sinp) * 180.0f / M_PI;
    *pitch = wrap_angle(*pitch);

    *yaw = atan2f(2.0f * (q->w * q->z + q->x * q->y), 1.0f - 2.0f * (q->y * q->y + q->z * q->z)) * 180.0f / M_PI;
    *yaw = wrap_angle(*yaw);
}

esp_err_t mpu6050_sensor_init(void)
{
    ESP_LOGI(TAG, "Initializing MPU6050 on I2C Port %d (SDA=GPIO%d, SCL=GPIO%d)...",
             MPU6050_I2C_PORT, MPU6050_PIN_SDA, MPU6050_PIN_SCL);

    // 1. Initialize dedicated I2C master bus
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = (i2c_port_num_t)MPU6050_I2C_PORT,
        .sda_io_num        = MPU6050_PIN_SDA,
        .scl_io_num        = MPU6050_PIN_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus %d: %s", MPU6050_I2C_PORT, esp_err_to_name(ret));
        return ret;
    }

    // 2. Attach MPU6050 device to the bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MPU6050_I2C_ADDR,
        .scl_speed_hz    = MPU6050_I2C_FREQ_HZ,
    };

    ret = i2c_master_bus_add_device(s_bus_handle, &dev_cfg, &s_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add MPU6050 device: %s", esp_err_to_name(ret));
        return ret;
    }

    // 3. Wake up sensor: write 0x00 to PWR_MGMT_1
    uint8_t wake_cmd[2] = {REG_PWR_MGMT_1, 0x00};
    ret = i2c_master_transmit(s_dev_handle, wake_cmd, sizeof(wake_cmd), 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake MPU6050: %s (check wiring & AD0)", esp_err_to_name(ret));
        return ret;
    }

    // 4. Quick Calibration (50 samples while vehicle is stationary)
    ESP_LOGI(TAG, "Calibrating MPU6050 gyro and accel bias... Keep vehicle stationary.");
    float gx_sum = 0.0f, gy_sum = 0.0f, gz_sum = 0.0f;
    float ax_sum = 0.0f, ay_sum = 0.0f, az_sum = 0.0f;
    int samples = 50;
    uint8_t reg_addr = REG_ACCEL_XOUT_H;
    uint8_t raw_buf[14];

    for (int i = 0; i < samples; i++) {
        if (i2c_master_transmit_receive(s_dev_handle, &reg_addr, 1, raw_buf, sizeof(raw_buf), 500) == ESP_OK) {
            int16_t raw_ax = (int16_t)((raw_buf[0] << 8) | raw_buf[1]);
            int16_t raw_ay = (int16_t)((raw_buf[2] << 8) | raw_buf[3]);
            int16_t raw_az = (int16_t)((raw_buf[4] << 8) | raw_buf[5]);
            int16_t raw_gx = (int16_t)((raw_buf[8] << 8) | raw_buf[9]);
            int16_t raw_gy = (int16_t)((raw_buf[10] << 8) | raw_buf[11]);
            int16_t raw_gz = (int16_t)((raw_buf[12] << 8) | raw_buf[13]);

            ax_sum += (raw_ax / ACCEL_SCALE);
            ay_sum += (raw_ay / ACCEL_SCALE);
            az_sum += (raw_az / ACCEL_SCALE);
            gx_sum += (raw_gx / GYRO_SCALE);
            gy_sum += (raw_gy / GYRO_SCALE);
            gz_sum += (raw_gz / GYRO_SCALE);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    s_accel_bias[0] = ax_sum / samples;
    s_accel_bias[1] = ay_sum / samples;
    s_accel_bias[2] = (az_sum / samples) - 1.0f; // 1.0g gravity along Z
    s_gyro_bias[0]  = gx_sum / samples;
    s_gyro_bias[1]  = gy_sum / samples;
    s_gyro_bias[2]  = gz_sum / samples;

    ESP_LOGI(TAG, "Calibration offsets -> Accel:[%.2f, %.2f, %.2f]g | Gyro:[%.2f, %.2f, %.2f]dps",
             s_accel_bias[0], s_accel_bias[1], s_accel_bias[2],
             s_gyro_bias[0], s_gyro_bias[1], s_gyro_bias[2]);

    s_last_read_time_us = esp_timer_get_time();
    s_initialized = true;
    ESP_LOGI(TAG, "MPU6050 successfully initialized and calibrated.");
    return ESP_OK;
}

esp_err_t mpu6050_sensor_read(mpu6050_data_t *data)
{
    if (!s_initialized || !data) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t reg_addr = REG_ACCEL_XOUT_H;
    uint8_t raw_buf[14];

    // Transmit register address then read 14 burst bytes: Accel (6), Temp (2), Gyro (6)
    esp_err_t ret = i2c_master_transmit_receive(s_dev_handle, &reg_addr, 1, raw_buf, sizeof(raw_buf), 1000);
    if (ret != ESP_OK) {
        return ret;
    }

    int64_t now_us = esp_timer_get_time();
    float dt = (float)(now_us - s_last_read_time_us) / 1000000.0f;
    if (dt <= 0.0f || dt > 1.0f) {
        dt = 0.1f; // default to 100ms if anomalous interval
    }
    s_last_read_time_us = now_us;

    int16_t raw_ax = (int16_t)((raw_buf[0] << 8) | raw_buf[1]);
    int16_t raw_ay = (int16_t)((raw_buf[2] << 8) | raw_buf[3]);
    int16_t raw_az = (int16_t)((raw_buf[4] << 8) | raw_buf[5]);

    int16_t raw_gx = (int16_t)((raw_buf[8] << 8) | raw_buf[9]);
    int16_t raw_gy = (int16_t)((raw_buf[10] << 8) | raw_buf[11]);
    int16_t raw_gz = (int16_t)((raw_buf[12] << 8) | raw_buf[13]);

    data->accel_x_g  = (raw_ax / ACCEL_SCALE) - s_accel_bias[0];
    data->accel_y_g  = (raw_ay / ACCEL_SCALE) - s_accel_bias[1];
    data->accel_z_g  = (raw_az / ACCEL_SCALE) - s_accel_bias[2];

    data->gyro_x_dps = (raw_gx / GYRO_SCALE) - s_gyro_bias[0];
    data->gyro_y_dps = (raw_gy / GYRO_SCALE) - s_gyro_bias[1];
    data->gyro_z_dps = (raw_gz / GYRO_SCALE) - s_gyro_bias[2];

    // Update quaternion attitude filter
    quaternion_update(&s_q, data->gyro_x_dps, data->gyro_y_dps, data->gyro_z_dps,
                      data->accel_x_g, data->accel_y_g, data->accel_z_g, dt);

    // Compute Euler angles (Roll, Pitch, Yaw)
    compute_euler_angles(&s_q, &data->roll_deg, &data->pitch_deg, &data->yaw_deg);

    return ESP_OK;
}
