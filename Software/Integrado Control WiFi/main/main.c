/**
 * @file main.c
 * @brief Single-board rover (ESP32-S3 N8R8): sensors, motion, camera, Wi-Fi and web GUI.
 *
 * Target: ESP32-S3 (custom board)   SDK: ESP-IDF v6.x
 *
 * Data flow:
 *   Browser/Python --WebSocket/HTTP--> drive_command --(20 Hz bridge)--> motion_control --> motors/servo
 *   vehicle_state + motion status --> telemetry --(20 Hz JSON)--> WebSocket clients
 *   ArduCAM --> camera_service (double buffer, PSRAM) --> /stream (:81) and /capture (:80)
 *
 * Tasks (see app_config.h for core/priority/period):
 *   core 1: motion (50 Hz), imu (50 Hz), env (1 Hz)
 *   core 0: Wi-Fi/lwIP, httpd x2, ws_tlm (20 Hz), drive_bridge (20 Hz), cam, status log
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"

#include "app_config.h"
#include "rover_types.h"
#include "bme690.h"
#include "mpu6050.h"
#include "servo.h"
#include "dc_motors.h"
#include "motion_control.h"
#include "encoders.h"
#include "vehicle_state.h"
#include "camera_service.h"
#include "drive_command.h"
#include "telemetry.h"
#include "web_server.h"
#include "wifi_ap.h"

static const char *TAG = "MAIN_APP";

static bool s_mpu_present = false;
static bool s_bme_present = false;

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
 * Human-readable status line on the console
 * ========================================================================= */
static void status_log_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        rover_telemetry_t t;
        drive_command_state_t cmd;
        telemetry_build(&t);
        drive_command_get(&cmd);

        ESP_LOGI("TELEMETRY",
                 "st=%u flags=0x%02X | cmd %.1f%% %.1fdeg%s%s | L:%.1f%% R:%.1f%% steer:%.1f | "
                 "enc L:%.1frpm(%ld) R:%.1frpm(%ld) %.2fm | "
                 "T:%.1fC H:%.1f%% P:%.1fhPa | R:%.1f P:%.1f Y:%.1f | sta:%d cam:%.1ffps | heap:%u psram:%u",
                 t.motion_state, t.status,
                 cmd.speed_pct, cmd.steer_deg, cmd.client_active ? "" : " (no client)", cmd.estop ? " ESTOP" : "",
                 t.motor_left_pct, t.motor_right_pct, t.steer_deg,
                 t.wheel_rpm_left, (long)t.enc_count_left, t.wheel_rpm_right, (long)t.enc_count_right, t.odom_m,
                 t.temp_c, t.hum_pct, t.press_hpa,
                 t.roll_deg, t.pitch_deg, t.yaw_deg,
                 wifi_ap_station_count(), camera_service_get_fps(),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

        encoders_diag_log();                /* no-op unless ENC_DIAG is 1 */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_LOG_PERIOD_MS));
    }
}

/* =========================================================================
 * Entry point
 * ========================================================================= */
void app_main(void)
{
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "  MINI VEHICULO EXPLORACION - SINGLE BOARD (S3)  ");
    ESP_LOGI(TAG, "  Firmware v%d                                   ", ROVER_FW_VERSION);
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "PSRAM: %u KB free", (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));

    /* 1. Actuators first, so outputs are in a safe state (disabled, centered) ASAP */
    vehicle_state_init();
    ESP_ERROR_CHECK(dc_motors_init());
    ESP_ERROR_CHECK(servo_init());
    ESP_ERROR_CHECK(motion_control_init());

    /* Encoders right away: this also takes GPIO43 back from UART0 TX (ROM boot log) */
    if (encoders_init() != ESP_OK) {
        ESP_LOGE(TAG, "Encoders not available. Check A/B = GPIO%d/%d (left) and GPIO%d/%d (right)",
                 ENC_LEFT_PIN_A, ENC_LEFT_PIN_B, ENC_RIGHT_PIN_A, ENC_RIGHT_PIN_B);
    }

    /* 2. NVS (required by Wi-Fi) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 3. Sensors: a missing sensor is reported but not fatal */
#if MPU6050_ENABLED
    s_mpu_present = (mpu6050_sensor_init() == ESP_OK);
    if (!s_mpu_present) {
        ESP_LOGE(TAG, "MPU6050 init failed! Check wiring on SDA=GPIO%d, SCL=GPIO%d", MPU6050_PIN_SDA, MPU6050_PIN_SCL);
    }
#else
    ESP_LOGW(TAG, "MPU6050 disabled in app_config.h (I2C%d is used by the camera)", MPU6050_I2C_PORT);
#endif
    s_bme_present = (bme690_sensor_init() == ESP_OK);
    if (!s_bme_present) {
        ESP_LOGE(TAG, "BME690 init failed! Check wiring on SDA=GPIO%d, SCL=GPIO%d", BME690_PIN_SDA, BME690_PIN_SCL);
    }

    /* 4. Real-time tasks (core 1). The motors stay DISARMED until a neutral command arrives. */
    drive_command_init();
    ESP_ERROR_CHECK(motion_control_start());
    xTaskCreatePinnedToCore(imu_task, "imu", TASK_IMU_STACK, NULL, TASK_IMU_PRIO, NULL, TASK_IMU_CORE);
    xTaskCreatePinnedToCore(env_task, "env", TASK_ENV_STACK, NULL, TASK_ENV_PRIO, NULL, TASK_ENV_CORE);
    ESP_ERROR_CHECK(drive_command_start_bridge());

    /* 5. Camera: not fatal, the GUI and driving work without it */
    if (camera_service_init() != ESP_OK) {
        ESP_LOGW(TAG, "Continuing without camera. Check CS/MOSI/SCK/MISO = GPIO%d/%d/%d/%d, SDA/SCL = GPIO%d/%d",
                 CAM_PIN_CS, CAM_PIN_MOSI, CAM_PIN_SCK, CAM_PIN_MISO, CAM_PIN_SDA, CAM_PIN_SCL);
    }

    /* 6. Network */
    ESP_ERROR_CHECK(wifi_ap_start());
    ESP_ERROR_CHECK(web_server_start());

    xTaskCreatePinnedToCore(status_log_task, "status_log", TASK_LOG_STACK, NULL, TASK_LOG_PRIO, NULL, TASK_LOG_CORE);

    ESP_LOGI(TAG, "Ready. Connect to '%s' and open http://192.168.4.1  (internal heap: %u, PSRAM: %u bytes free)",
             WIFI_AP_SSID, (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "Motors DISARMED until a neutral command arrives.");
}
