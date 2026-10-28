/**
 * @file main.c
 * @brief System initialization, FreeRTOS tasks, and MODE_TESTING execution.
 * 
 * Target: ESP32-S3 WROOM
 * SDK: ESP-IDF v5.x
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "project_config.h"
#include "bme690.h"
#include "mpu6050.h"
#include "servo.h"
#include "dc_motors.h"
#include "uart_comm.h"

static const char *TAG = "MAIN_APP";

// Global shared telemetry structure & mutex
static telemetry_packet_t s_telemetry_data;
static SemaphoreHandle_t  s_telemetry_mutex = NULL;

#if (CURRENT_OPERATION_MODE == MODE_TESTING)

/**
 * @brief Task: Periodically read MPU6050 and BME690 sensors.
 */
static void sensor_read_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Sensor read task started.");
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100); // 10 Hz read rate

    uint32_t bme_tick_divider = 0;

    while (1) {
        mpu6050_data_t mpu_val = {0};
        esp_err_t mpu_err = mpu6050_sensor_read(&mpu_val);

        bme690_sensor_data_t bme_val = {0};
        esp_err_t bme_err = ESP_ERR_NOT_FINISHED;

        // BME690 forced mode gas measurement takes ~160ms, read every ~1s (10 ticks)
        if (++bme_tick_divider >= 10) {
            bme_tick_divider = 0;
            bme_err = bme690_sensor_read(&bme_val);
        }

        // Update protected telemetry structure
        if (xSemaphoreTake(s_telemetry_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            s_telemetry_data.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

            if (mpu_err == ESP_OK) {
                s_telemetry_data.accel_x_g  = mpu_val.accel_x_g;
                s_telemetry_data.accel_y_g  = mpu_val.accel_y_g;
                s_telemetry_data.accel_z_g  = mpu_val.accel_z_g;
                s_telemetry_data.gyro_x_dps = mpu_val.gyro_x_dps;
                s_telemetry_data.gyro_y_dps = mpu_val.gyro_y_dps;
                s_telemetry_data.gyro_z_dps = mpu_val.gyro_z_dps;
                s_telemetry_data.roll_deg   = mpu_val.roll_deg;
                s_telemetry_data.pitch_deg  = mpu_val.pitch_deg;
                s_telemetry_data.yaw_deg    = mpu_val.yaw_deg;
                s_telemetry_data.mpu_status = 1;
            } else {
                s_telemetry_data.mpu_status = 0;
            }

            if (bme_err == ESP_OK) {
                s_telemetry_data.temperature_c      = bme_val.temperature_c;
                s_telemetry_data.pressure_hpa       = bme_val.pressure_hpa;
                s_telemetry_data.humidity_pct       = bme_val.humidity_pct;
                s_telemetry_data.gas_resistance_ohm = bme_val.gas_resistance_ohm;
                s_telemetry_data.bme_status         = 1;
            }

            s_telemetry_data.servo_angle_deg  = servo_get_current_angle();
            s_telemetry_data.motor_fault_flag = dc_motors_check_fault() ? 1 : 0;
            dc_motors_get_speeds(&s_telemetry_data.motor_speed_m1, &s_telemetry_data.motor_speed_m2);

            xSemaphoreGive(s_telemetry_mutex);
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}

/**
 * @brief Task: Test actuators by sweeping the servo (0°, 90°, 180°) and driving motors.
 */
static void actuator_test_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Actuator test task started.");

    while (1) {
        // Step 1: Servo 0°, Motors Forward
        ESP_LOGI(TAG, "[TEST STEP 1] Servo -> 0° | Motors -> FWD (40%)");
        servo_set_angle(SERVO_POS_0_DEG);
        dc_motors_set_speed(MOTOR_SPEED_DEFAULT_FWD, MOTOR_SPEED_DEFAULT_FWD);
        vTaskDelay(pdMS_TO_TICKS(2000));

        // Step 2: Servo 90°, Motors Stop / Brake
        ESP_LOGI(TAG, "[TEST STEP 2] Servo -> 90° | Motors -> STOP (0%)");
        servo_set_angle(SERVO_POS_90_DEG);
        dc_motors_stop();
        vTaskDelay(pdMS_TO_TICKS(1500));

        // Step 3: Servo 180°, Motors Reverse
        ESP_LOGI(TAG, "[TEST STEP 3] Servo -> 180° | Motors -> REV (-40%)");
        servo_set_angle(SERVO_POS_180_DEG);
        dc_motors_set_speed(MOTOR_SPEED_DEFAULT_REV, MOTOR_SPEED_DEFAULT_REV);
        vTaskDelay(pdMS_TO_TICKS(2000));

        // Step 4: Servo 90°, Motors Stop
        ESP_LOGI(TAG, "[TEST STEP 4] Servo -> 90° | Motors -> STOP (0%)");
        servo_set_angle(SERVO_POS_90_DEG);
        dc_motors_stop();
        vTaskDelay(pdMS_TO_TICKS(1500));

        // Safety Fault Check
        if (dc_motors_check_fault()) {
            ESP_LOGE(TAG, "[SAFETY ALERT] DRV8873H Fault detected! Stopping motors.");
            dc_motors_stop();
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
}

/**
 * @brief Task: Periodically serialize and transmit telemetry data over UART.
 */
static void telemetry_tx_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Telemetry transmission task started.");
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(200); // 5 Hz telemetry transmission

    telemetry_packet_t packet_to_send;

    while (1) {
        if (xSemaphoreTake(s_telemetry_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            memcpy(&packet_to_send, &s_telemetry_data, sizeof(telemetry_packet_t));
            xSemaphoreGive(s_telemetry_mutex);

            // Transmit binary packet frame over UART
            uart_comm_send_telemetry(&packet_to_send);

            // Periodic human-readable log (every 1 second)
            static uint32_t log_cnt = 0;
            if (++log_cnt >= 5) {
                log_cnt = 0;
                ESP_LOGI("TELEMETRY", "T:%.1f°C | H:%.1f%% | P:%.1fhPa | Accel:[%.2f,%.2f,%.2f]g | Gyro:[%.1f,%.1f,%.1f]dps | Angles:[R:%.1f°,P:%.1f°,Y:%.1f°] | Servo:%u° | M1:%.0f%% M2:%.0f%%",
                         packet_to_send.temperature_c,
                         packet_to_send.humidity_pct,
                         packet_to_send.pressure_hpa,
                         packet_to_send.accel_x_g, packet_to_send.accel_y_g, packet_to_send.accel_z_g,
                         packet_to_send.gyro_x_dps, packet_to_send.gyro_y_dps, packet_to_send.gyro_z_dps,
                         packet_to_send.roll_deg, packet_to_send.pitch_deg, packet_to_send.yaw_deg,
                         packet_to_send.servo_angle_deg,
                         packet_to_send.motor_speed_m1, packet_to_send.motor_speed_m2);
            }
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}

#elif (CURRENT_OPERATION_MODE == MODE_INTEGRATED_WIFI_CAM)
// Architectural stub for future integrated SoftAP + Camera server mode
static void wifi_cam_mode_stub(void)
{
    ESP_LOGW(TAG, "Running under MODE_INTEGRATED_WIFI_CAM (Stub implementation).");
}
#elif (CURRENT_OPERATION_MODE == MODE_EXTERNAL_UART_WIFI_CAM)
// Architectural stub for future external Wi-Fi Camera communication mode
static void external_uart_wifi_cam_mode_stub(void)
{
    ESP_LOGW(TAG, "Running under MODE_EXTERNAL_UART_WIFI_CAM (Stub implementation).");
}
#endif

void app_main(void)
{
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "    MINI VEHICULO EXPLORACION - FIRMWARE v1.0    ");
    ESP_LOGI(TAG, "    Target: ESP32-S3 WROOM | Mode: MODE_TESTING   ");
    ESP_LOGI(TAG, "=================================================");

    // 1. Initialize telemetry mutex
    s_telemetry_mutex = xSemaphoreCreateMutex();
    assert(s_telemetry_mutex != NULL);
    memset(&s_telemetry_data, 0, sizeof(telemetry_packet_t));

    // 2. Initialize Hardware Peripherals
    ESP_LOGI(TAG, "[INIT] Initializing Peripherals...");
    
    // UART Communication
    ESP_ERROR_CHECK(uart_comm_init());

    // Actuators
    ESP_ERROR_CHECK(servo_init());
    ESP_ERROR_CHECK(dc_motors_init());

    // Dedicated I2C Bus 0: MPU6050
    esp_err_t mpu_ret = mpu6050_sensor_init();
    if (mpu_ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 init failed! Check wiring on SDA=GPIO%d, SCL=GPIO%d", MPU6050_PIN_SDA, MPU6050_PIN_SCL);
    }

    // Dedicated I2C Bus 1: BME690
    esp_err_t bme_ret = bme690_sensor_init();
    if (bme_ret != ESP_OK) {
        ESP_LOGE(TAG, "BME690 init failed! Check wiring on SDA=GPIO%d, SCL=GPIO%d", BME690_PIN_SDA, BME690_PIN_SCL);
    }

#if (CURRENT_OPERATION_MODE == MODE_TESTING)
    ESP_LOGI(TAG, "[EXEC] Spawning FreeRTOS tasks for MODE_TESTING...");

    // Create Sensor Read Task (Priority 5, 4KB stack)
    xTaskCreatePinnedToCore(sensor_read_task, "sensor_read_task", 4096, NULL, 5, NULL, 1);

    // Create Actuator Test Task (Priority 4, 4KB stack)
    xTaskCreatePinnedToCore(actuator_test_task, "actuator_test_task", 4096, NULL, 4, NULL, 1);

    // Create Telemetry TX Task (Priority 3, 4KB stack)
    xTaskCreatePinnedToCore(telemetry_tx_task, "telemetry_tx_task", 4096, NULL, 3, NULL, 0);

#elif (CURRENT_OPERATION_MODE == MODE_INTEGRATED_WIFI_CAM)
    wifi_cam_mode_stub();
#elif (CURRENT_OPERATION_MODE == MODE_EXTERNAL_UART_WIFI_CAM)
    external_uart_wifi_cam_mode_stub();
#endif

    ESP_LOGI(TAG, "System initialization complete. Scheduler running.");
}
