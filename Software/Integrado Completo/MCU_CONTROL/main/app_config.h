/**
 * @file app_config.h
 * @brief MCU_CONTROL master configuration: pins, task layout, motion tunables.
 *
 * Target: ESP32-S3 (custom board)   SDK: ESP-IDF v6.x
 *
 * Pin map taken from "Integrado Sensores Actuadores/main/project_config.h"
 * (verified working with the sensors and raw motor drive).
 */

#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * 1. I2C BUSES (two independent buses)
 * ========================================================================= */

/* --- Bus 0: MPU6050 6-axis IMU --- */
#define MPU6050_I2C_PORT                0
#define MPU6050_PIN_SDA                 GPIO_NUM_40
#define MPU6050_PIN_SCL                 GPIO_NUM_41
#define MPU6050_I2C_FREQ_HZ             400000
#define MPU6050_I2C_ADDR                0x68

/* --- Bus 1: BME690 environmental sensor --- */
#define BME690_I2C_PORT                 1
#define BME690_PIN_SDA                  GPIO_NUM_5
#define BME690_PIN_SCL                  GPIO_NUM_6
#define BME690_I2C_FREQ_HZ              400000
#define BME690_I2C_ADDR                 0x76  /* SDO to GND -> 0x76, SDO to VDD -> 0x77 */

/* BME690 gas heater setup */
#define BME690_GAS_HEATER_TEMP_C        320
#define BME690_GAS_HEATER_DUR_MS        150
#define BME690_AMB_TEMP_INIT_C          25

/* =========================================================================
 * 2. STEERING SERVO (LEDC PWM, 50 Hz)
 * ========================================================================= */
#define SERVO_PIN_PWM                   GPIO_NUM_39
#define SERVO_LEDC_TIMER                LEDC_TIMER_0
#define SERVO_LEDC_MODE                 LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_CHANNEL              LEDC_CHANNEL_0
#define SERVO_LEDC_DUTY_RES             LEDC_TIMER_13_BIT
#define SERVO_LEDC_FREQ_HZ              50

#define SERVO_MIN_PULSE_US              500     /* 0 deg   */
#define SERVO_MAX_PULSE_US              2500    /* 180 deg */

/* Steering geometry: steering angle (deg, + = right) -> servo angle */
#define SERVO_CENTER_DEG                90.0f   /* servo angle with wheels straight (trim here) */
#define SERVO_MAX_STEER_DEG             45.0f   /* mechanical steering limit, each side */
#define SERVO_INVERT                    0       /* 1 if + steering turns the wheels left */

/* =========================================================================
 * 3. DC MOTORS (MCPWM -> 2x DRV8873H)
 * ========================================================================= */
#define MOTOR_PWM_FREQ_HZ               20000

/* Motor 1 */
#define M1_PIN_IN1                      GPIO_NUM_7
#define M1_PIN_IN2                      GPIO_NUM_8
#define M1_PIN_DISABLE                  GPIO_NUM_9      /* active HIGH = outputs Hi-Z */
#define M1_PIN_NFAULT                   GPIO_NUM_21     /* active LOW */

/* Motor 2 */
#define M2_PIN_IN1                      GPIO_NUM_14
#define M2_PIN_IN2                      GPIO_NUM_15
#define M2_PIN_DISABLE                  GPIO_NUM_16
#define M2_PIN_NFAULT                   GPIO_NUM_38

/* =========================================================================
 * 4. MOTION CONTROL TUNABLES (see motion_control.c / kinematics.c)
 * ========================================================================= */

/* Wheel mapping: which H-bridge drives which rear wheel, and polarity */
#define MOTOR_LEFT_IS_M1                1       /* 1: M1 = left rear, M2 = right rear */
#define MOTOR_LEFT_INVERT               0       /* 1 if + duty makes the left wheel go backwards */
#define MOTOR_RIGHT_INVERT              0

/* Ackermann geometry (meters) - MEASURE ON THE ROVER */
#define VEHICLE_WHEELBASE_M             0.20f   /* front axle to rear axle */
#define VEHICLE_TRACK_M                 0.16f   /* distance between rear wheels */

/* Electronic differential gain: 0.0 = both rear wheels same speed (pure Ackermann),
 * 1.0 = full geometric differential. */
#define EDIFF_GAIN                      1.0f

/* Command limits */
#define MOTION_MAX_SPEED_PCT            100.0f  /* global cap on |duty| */

/* Slew-rate limits (smooth acceleration / deceleration) */
#define MOTION_ACCEL_PCT_PER_S          150.0f  /* |speed| increasing */
#define MOTION_DECEL_PCT_PER_S          300.0f  /* |speed| decreasing (normal) */
#define MOTION_ESTOP_DECEL_PCT_PER_S    1000.0f /* failsafe / e-stop: 100% -> 0 in 100 ms */
#define MOTION_STEER_DEG_PER_S          360.0f  /* servo slew */

/* Motor dead-band compensation: |cmd| below DEADBAND -> 0,
 * above it the output is remapped to MIN_DUTY..100 so small commands still move. */
#define MOTOR_DEADBAND_PCT              2.0f
#define MOTOR_MIN_DUTY_PCT              12.0f

/* Arming: a command is "neutral" if |speed| is below this */
#define MOTION_NEUTRAL_SPEED_PCT        1.0f

/* Fail-safe: stop if no valid command frame for this long */
#define LINK_TIMEOUT_MS                 500

/* =========================================================================
 * 5. UART LINK TO MCU_COMM
 * ========================================================================= */
#define LINK_UART_PORT                  UART_NUM_1
/* Board routes the link to the TXD0/RXD0 pads, so the console must be on USB-Serial-JTAG */
#define LINK_UART_TX_PIN                GPIO_NUM_43     /* TXD0, module pin 37 -> ESP32 (COMM) RX GPIO16 */
#define LINK_UART_RX_PIN                GPIO_NUM_44     /* RXD0, module pin 36 <- ESP32 (COMM) TX GPIO17 */
#define LINK_UART_BAUD_RATE             115200
#define LINK_UART_RX_BUF_SIZE           2048
#define LINK_UART_TX_BUF_SIZE           1024

/* =========================================================================
 * 6. TASK LAYOUT (core, priority, period)
 * ========================================================================= */
#define TASK_UART_RX_CORE               0
#define TASK_UART_RX_PRIO               10
#define TASK_UART_RX_STACK              4096

#define TASK_MOTION_CORE                1
#define TASK_MOTION_PRIO                9
#define TASK_MOTION_STACK               4096
#define TASK_MOTION_PERIOD_MS           20      /* 50 Hz */

#define TASK_IMU_CORE                   1
#define TASK_IMU_PRIO                   6
#define TASK_IMU_STACK                  4096
#define TASK_IMU_PERIOD_MS              20      /* 50 Hz */

#define TASK_ENV_CORE                   1
#define TASK_ENV_PRIO                   4
#define TASK_ENV_STACK                  4096
#define TASK_ENV_PERIOD_MS              1000    /* forced mode + heater ~160 ms */

#define TASK_TLM_CORE                   0
#define TASK_TLM_PRIO                   5
#define TASK_TLM_STACK                  4096
#define TASK_TLM_PERIOD_MS              50      /* 20 Hz */

#ifdef __cplusplus
}
#endif
