/**
 * @file main.c
 * @brief MCU_CONTROL: system init and FreeRTOS tasks.
 *
 * Target: ESP32-S3 (custom board)   SDK: ESP-IDF v6.x
 *
 * Tasks (see app_config.h for core/priority/period):
 *   uart_rx    - parse frames from MCU_COMM, submit drive commands   (uart_link.c)
 *   motion     - fail-safe state machine, ramps, kinematics, outputs (motion_control.c)
 *   imu        - MPU6050 + orientation filter, 50 Hz
 *   env        - BME690 forced measurement, 1 Hz
 *   telemetry  - TLM_STATE frame to MCU_COMM, 20 Hz
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "app_config.h"
#include "rover_proto.h"
#include "bme690.h"
#include "mpu6050.h"
#include "servo.h"
#include "dc_motors.h"
#include "motion_control.h"
#include "uart_link.h"
#include "vehicle_state.h"

static const char *TAG = "MAIN_APP";

static bool s_mpu_present = false;
static bool s_bme_present = false;

static int16_t to_x10(float v)
{
    float s = v * 10.0f;
    if (s > 32767.0f)  s = 32767.0f;
    if (s < -32768.0f) s = -32768.0f;
    return (int16_t)(s >= 0.0f ? s + 0.5f : s - 0.5f);
}

/* =========================================================================
 * Sensor tasks
 * ========================================================================= */
static void imu_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t err_count = 0;

    while (1) {
        mpu6050_data_t imu = {0};
        bool ok = s_mpu_present && (mpu6050_sensor_read(&imu) == ESP_OK);
        vehicle_state_set_imu(&imu, ok);

        if (s_mpu_present && !ok && (++err_count % 50) == 1) {
            ESP_LOGW(TAG, "MPU6050 read error (count %lu)", (unsigned long)err_count);
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_IMU_PERIOD_MS));
    }
}

static void env_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        if (s_bme_present) {
            bme690_sensor_data_t env = {0};
            bool ok = (bme690_sensor_read(&env) == ESP_OK);   /* blocks ~160 ms (heater) */
            vehicle_state_set_env(&env, ok);
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_ENV_PERIOD_MS));
    }
}

/* =========================================================================
 * Telemetry
 * ========================================================================= */
static void build_telemetry(rp_tlm_state_t *t)
{
    vehicle_state_t vs;
    motion_status_t ms;
    uart_link_stats_t ls;
    vehicle_state_get_copy(&vs);
    motion_control_get_status(&ms);
    uart_link_get_stats(&ls);

    memset(t, 0, sizeof(*t));
    t->uptime_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    t->ax_g      = vs.imu.accel_x_g;
    t->ay_g      = vs.imu.accel_y_g;
    t->az_g      = vs.imu.accel_z_g;
    t->gx_dps    = vs.imu.gyro_x_dps;
    t->gy_dps    = vs.imu.gyro_y_dps;
    t->gz_dps    = vs.imu.gyro_z_dps;
    t->roll_deg  = vs.imu.roll_deg;
    t->pitch_deg = vs.imu.pitch_deg;
    t->yaw_deg   = vs.imu.yaw_deg;

    t->temp_c    = vs.env.temperature_c;
    t->hum_pct   = vs.env.humidity_pct;
    t->press_hpa = vs.env.pressure_hpa;
    t->gas_ohm   = vs.env.gas_resistance_ohm;

    t->motor_left_x10  = to_x10(ms.left_pct);
    t->motor_right_x10 = to_x10(ms.right_pct);
    t->steer_x10       = to_x10(ms.steer_deg);

    uint8_t st = 0;
    if (vs.imu_ok)                         st |= RP_ST_MPU_OK;
    if (vs.env_ok)                         st |= RP_ST_BME_OK;
    if (vs.env_ok && vs.env.gas_valid)     st |= RP_ST_GAS_VALID;
    if (ms.fault)                          st |= RP_ST_MOTOR_FAULT;
    if (ms.state == RP_MOTION_FAILSAFE)    st |= RP_ST_FAILSAFE;
    if (ms.state == RP_MOTION_ARMED)       st |= RP_ST_ARMED;
    if (ms.link_ok)                        st |= RP_ST_LINK_OK;
    t->status       = st;
    t->motion_state = (uint8_t)ms.state;

    t->rx_ok      = (uint16_t)ls.frames_ok;
    t->rx_crc_err = (uint16_t)ls.crc_errors;
    t->proto_version = ROVER_PROTO_VERSION;
}

static void telemetry_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t log_div = 0;

    while (1) {
        rp_tlm_state_t t;
        uart_link_stats_t ls;
        build_telemetry(&t);
        uart_link_send(RP_MSG_TLM_STATE, &t, sizeof(t));

        /* human-readable log every 2 s */
        if (++log_div >= 2000 / TASK_TLM_PERIOD_MS) {
            log_div = 0;
            uart_link_get_stats(&ls);
            ESP_LOGI("TELEMETRY",
                     "st=%u flags=0x%02X | L:%.1f%% R:%.1f%% steer:%.1f | T:%.1fC H:%.1f%% P:%.1fhPa | "
                     "R:%.1f P:%.1f Y:%.1f | rx bytes:%lu ok:%u crc:%u | tx:%lu",
                     t.motion_state, t.status,
                     t.motor_left_x10 / 10.0f, t.motor_right_x10 / 10.0f, t.steer_x10 / 10.0f,
                     t.temp_c, t.hum_pct, t.press_hpa,
                     t.roll_deg, t.pitch_deg, t.yaw_deg,
                     (unsigned long)ls.rx_bytes, t.rx_ok, t.rx_crc_err,
                     (unsigned long)ls.tx_frames);
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_TLM_PERIOD_MS));
    }
}

/* =========================================================================
 * Entry point
 * ========================================================================= */
void app_main(void)
{
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "  MINI VEHICULO EXPLORACION - MCU_CONTROL (S3)   ");
    ESP_LOGI(TAG, "  Protocol v%d                                   ", ROVER_PROTO_VERSION);
    ESP_LOGI(TAG, "=================================================");

    vehicle_state_init();

    /* Actuators first, so outputs are in a safe state (disabled, centered) ASAP */
    ESP_ERROR_CHECK(dc_motors_init());
    ESP_ERROR_CHECK(servo_init());
    ESP_ERROR_CHECK(motion_control_init());

    /* Link to MCU_COMM */
    ESP_ERROR_CHECK(uart_link_init());

    /* Sensors: a missing sensor is reported but not fatal */
    s_mpu_present = (mpu6050_sensor_init() == ESP_OK);
    if (!s_mpu_present) {
        ESP_LOGE(TAG, "MPU6050 init failed! Check wiring on SDA=GPIO%d, SCL=GPIO%d", MPU6050_PIN_SDA, MPU6050_PIN_SCL);
    }
    s_bme_present = (bme690_sensor_init() == ESP_OK);
    if (!s_bme_present) {
        ESP_LOGE(TAG, "BME690 init failed! Check wiring on SDA=GPIO%d, SCL=GPIO%d", BME690_PIN_SDA, BME690_PIN_SCL);
    }

    ESP_ERROR_CHECK(motion_control_start());
    xTaskCreatePinnedToCore(imu_task, "imu", TASK_IMU_STACK, NULL, TASK_IMU_PRIO, NULL, TASK_IMU_CORE);
    xTaskCreatePinnedToCore(env_task, "env", TASK_ENV_STACK, NULL, TASK_ENV_PRIO, NULL, TASK_ENV_CORE);
    xTaskCreatePinnedToCore(telemetry_task, "telemetry", TASK_TLM_STACK, NULL, TASK_TLM_PRIO, NULL, TASK_TLM_CORE);

    ESP_LOGI(TAG, "Initialization complete. Motors DISARMED until a neutral command arrives.");
}
