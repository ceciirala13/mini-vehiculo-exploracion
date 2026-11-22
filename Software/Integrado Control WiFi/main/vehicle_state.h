/**
 * @file vehicle_state.h
 * @brief Thread-safe snapshot of the latest sensor readings.
 *
 * Writers: imu_task, env_task. Reader: telemetry_task.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "mpu6050.h"
#include "bme690.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    mpu6050_data_t       imu;
    bool                 imu_ok;        /* last IMU read succeeded */
    bme690_sensor_data_t env;
    bool                 env_ok;        /* at least one valid env reading, last read succeeded */
} vehicle_state_t;

void vehicle_state_init(void);

void vehicle_state_set_imu(const mpu6050_data_t *imu, bool ok);
void vehicle_state_set_env(const bme690_sensor_data_t *env, bool ok);

void vehicle_state_get_copy(vehicle_state_t *out);

#ifdef __cplusplus
}
#endif
