/**
 * @file project_config.h
 * @brief Master configuration, centralized pin assignments, and operation mode macros.
 * 
 * Target: ESP32-S3 (ESP32-S3-WROOM)
 * SDK: ESP-IDF v5.x
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * 1. OPERATING MODES SELECTION
 * ========================================================================= */
#define MODE_TESTING                    1
#define MODE_INTEGRATED_WIFI_CAM        2
#define MODE_EXTERNAL_UART_WIFI_CAM     3

/** Select current active operating mode */
#define CURRENT_OPERATION_MODE          MODE_EXTERNAL_UART_WIFI_CAM

/* =========================================================================
 * 2. I2C BUS CONFIGURATION (TWO INDEPENDENT BUSES)
 * ========================================================================= */

/* --- Bus 0: MPU6050 6-Axis IMU --- */
#define MPU6050_I2C_PORT                0
#define MPU6050_PIN_SDA                 GPIO_NUM_40
#define MPU6050_PIN_SCL                 GPIO_NUM_41
#define MPU6050_I2C_FREQ_HZ             400000
#define MPU6050_I2C_ADDR                0x68

/* --- Bus 1: BME690 Environmental Sensor --- */
#define BME690_I2C_PORT                 1
#define BME690_PIN_SDA                  GPIO_NUM_5
#define BME690_PIN_SCL                  GPIO_NUM_6
#define BME690_I2C_FREQ_HZ              400000
#define BME690_I2C_ADDR                 0x76  /* SDO to GND -> 0x76, SDO to VDD -> 0x77 */

/* BME690 Gas heater setup */
#define BME690_GAS_HEATER_TEMP_C        320
#define BME690_GAS_HEATER_DUR_MS        150
#define BME690_AMB_TEMP_INIT_C          25

/* =========================================================================
 * 3. SERVO MOTOR (LEDC PWM) CONFIGURATION
 * ========================================================================= */
#define SERVO_PIN_PWM                   GPIO_NUM_39
#define SERVO_LEDC_TIMER                LEDC_TIMER_0
#define SERVO_LEDC_MODE                 LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_CHANNEL              LEDC_CHANNEL_0
#define SERVO_LEDC_DUTY_RES             LEDC_TIMER_13_BIT
#define SERVO_LEDC_FREQ_HZ              50

/* 13-bit (8192 counts) @ 50 Hz (20ms period):
 * 0.5ms (0 deg)   -> (0.5 / 20.0) * 8192 ≈ 205
 * 1.5ms (90 deg)  -> (1.5 / 20.0) * 8192 ≈ 614
 * 2.5ms (180 deg) -> (2.5 / 20.0) * 8192 ≈ 1024
 */
#define SERVO_DUTY_0_DEG                205
#define SERVO_DUTY_90_DEG               614
#define SERVO_DUTY_180_DEG              1024

/* =========================================================================
 * 4. DC MOTORS (MCPWM DRV8873H / H-BRIDGE) CONFIGURATION
 * ========================================================================= */
#define MOTOR_PWM_FREQ_HZ               20000

/* Motor 1 */
#define M1_PIN_IN1                      GPIO_NUM_7
#define M1_PIN_IN2                      GPIO_NUM_8
#define M1_PIN_DISABLE                  GPIO_NUM_9
#define M1_PIN_NFAULT                   GPIO_NUM_21

/* Motor 2 */
#define M2_PIN_IN1                      GPIO_NUM_14
#define M2_PIN_IN2                      GPIO_NUM_15
#define M2_PIN_DISABLE                  GPIO_NUM_16
#define M2_PIN_NFAULT                   GPIO_NUM_38

/* Speed Presets */
#define MOTOR_SPEED_STOP                0.0f
#define MOTOR_SPEED_DEFAULT_FWD         80.0f
#define MOTOR_SPEED_DEFAULT_REV         -80.0f

/* =========================================================================
 * 5. UART TELEMETRY CONFIGURATION
 * ========================================================================= */
#define TELEMETRY_UART_PORT             UART_NUM_1
#define TELEMETRY_UART_TX_PIN           GPIO_NUM_17
#define TELEMETRY_UART_RX_PIN           GPIO_NUM_18
#define TELEMETRY_UART_BAUD_RATE        115200
#define TELEMETRY_UART_BUF_SIZE         512

/* Frame Delimiters */
#define TELEMETRY_FRAME_START_0         0xAA
#define TELEMETRY_FRAME_START_1         0x55
#define TELEMETRY_FRAME_END_0           0x0D
#define TELEMETRY_FRAME_END_1           0x0A

/* =========================================================================
 * 6. TELEMETRY PACKET STRUCTURE
 * ========================================================================= */
#pragma pack(push, 1)
typedef struct {
    uint8_t  header[2];            /* 0xAA, 0x55 */
    uint32_t timestamp_ms;
    
    /* BME690 Environmental Data */
    float    temperature_c;
    float    pressure_hpa;
    float    humidity_pct;
    uint32_t gas_resistance_ohm;
    uint8_t  bme_status;           /* 0 = Err, 1 = OK */

    /* MPU6050 Inertial & Orientation Data */
    float    accel_x_g;
    float    accel_y_g;
    float    accel_z_g;
    float    gyro_x_dps;
    float    gyro_y_dps;
    float    gyro_z_dps;
    float    roll_deg;             /* Tilt angle Roll in degrees */
    float    pitch_deg;            /* Tilt angle Pitch in degrees */
    float    yaw_deg;              /* Heading angle Yaw in degrees */
    uint8_t  mpu_status;           /* 0 = Err, 1 = OK */

    /* Actuator & Safety Status */
    float    motor_speed_m1;
    float    motor_speed_m2;
    uint16_t servo_angle_deg;
    uint8_t  motor_fault_flag;     /* 1 if nFAULT active */

    uint16_t checksum;
    uint8_t  tail[2];              /* 0x0D, 0x0A */
} telemetry_packet_t;
#pragma pack(pop)

#ifdef __cplusplus
}
#endif
