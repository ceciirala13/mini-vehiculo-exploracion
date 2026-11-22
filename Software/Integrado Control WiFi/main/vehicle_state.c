/**
 * @file vehicle_state.c
 * @brief Thread-safe sensor snapshot.
 */

#include "vehicle_state.h"
#include "freertos/FreeRTOS.h"
#include <string.h>

static portMUX_TYPE    s_lock = portMUX_INITIALIZER_UNLOCKED;
static vehicle_state_t s_state;

void vehicle_state_init(void)
{
    taskENTER_CRITICAL(&s_lock);
    memset(&s_state, 0, sizeof(s_state));
    s_state.imu.accel_z_g = 1.0f;
    taskEXIT_CRITICAL(&s_lock);
}

void vehicle_state_set_imu(const mpu6050_data_t *imu, bool ok)
{
    taskENTER_CRITICAL(&s_lock);
    if (ok) {
        s_state.imu = *imu;
    }
    s_state.imu_ok = ok;
    taskEXIT_CRITICAL(&s_lock);
}

void vehicle_state_set_env(const bme690_sensor_data_t *env, bool ok)
{
    taskENTER_CRITICAL(&s_lock);
    if (ok) {
        s_state.env = *env;     /* keep last good values on a failed read */
    }
    s_state.env_ok = ok;
    taskEXIT_CRITICAL(&s_lock);
}

void vehicle_state_get_copy(vehicle_state_t *out)
{
    taskENTER_CRITICAL(&s_lock);
    *out = s_state;
    taskEXIT_CRITICAL(&s_lock);
}
