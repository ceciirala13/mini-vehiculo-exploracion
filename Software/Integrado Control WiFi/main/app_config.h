/**
 * @file app_config.h
 * @brief Single-board rover configuration: pins, Wi-Fi, camera, task layout, motion tunables.
 *
 * Target: ESP32-S3-WROOM-1-N8R8 (8 MB flash, 8 MB octal PSRAM), custom board
 * SDK:    ESP-IDF v6.x
 *
 * This board does everything: sensors, motors, steering, ArduCAM camera,
 * Wi-Fi access point and web GUI.
 *
 * Core split:
 *   core 0 - Wi-Fi / lwIP, both HTTP servers, WebSocket telemetry, camera, drive bridge
 *   core 1 - motion control, IMU, BME690
 * so network and camera load never delays the 50 Hz motor loop.
 *
 * Never use GPIO26-32 (flash) or GPIO35-37 (octal PSRAM) on this module.
 */

#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * 1. I2C BUSES (the S3 has two hardware controllers)
 * ========================================================================= */

/* --- Bus 0: MPU6050 6-axis IMU ---
 * To lend I2C0 to the camera instead: MPU6050_ENABLED 0 and CAM_I2C_PORT 0. */
#define MPU6050_ENABLED                 1
#define MPU6050_I2C_PORT                0
#define MPU6050_PIN_SDA                 GPIO_NUM_40
#define MPU6050_PIN_SCL                 GPIO_NUM_41
#define MPU6050_I2C_FREQ_HZ             400000
#define MPU6050_I2C_ADDR                0x68

/* Mounting: rotate the sensor axes 90 deg about z before the orientation filter.
 * Robot frame: x = width, y = length, z = normal to the robot plane.
 *   1: new x = -old y, new y = old x   (old x is now y, old y is now -x)
 *   0: sensor axes used as they are */
#define IMU_ROTATE_Z_90                 1
/* Flip the sign of the horizon angles if one moves the wrong way (1 or -1) */
#define IMU_ROLL_SIGN                   1
#define IMU_PITCH_SIGN                  1

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

/* Motor 2 */
#define M2_PIN_IN1                      GPIO_NUM_14
#define M2_PIN_IN2                      GPIO_NUM_15
#define M2_PIN_DISABLE                  GPIO_NUM_16
/* The DRV8873 nFAULT outputs are no longer read (GPIO21/38 now go to the encoders).
 * The drivers still protect themselves (over-current, over-temperature), the
 * firmware just isn't told. */

/* =========================================================================
 * 3b. WHEEL ENCODERS (Hall, quadrature, PCNT)
 *     11 PPR on the motor shaft, 1:45 gearbox, built-in pull-ups.
 *     POWER THE ENCODERS FROM 3.3 V: their pull-ups go to their supply.
 * ========================================================================= */
#define ENC_LEFT_PIN_A                  GPIO_NUM_44     /* RXD0, module pin 36 */
#define ENC_LEFT_PIN_B                  GPIO_NUM_43     /* TXD0, module pin 37.
                                                         * The ROM drives it as UART0 TX for ~100 ms at
                                                         * every reset: use a 1 kOhm series resistor. */
#define ENC_RIGHT_PIN_A                 GPIO_NUM_21     /* old M1 nFAULT */
#define ENC_RIGHT_PIN_B                 GPIO_NUM_38     /* old M2 nFAULT */

#define ENC_LEFT_INVERT                 1       /* counted backwards when driving forward (bench test) */
#define ENC_RIGHT_INVERT                1       /* counted backwards when driving forward (bench test) */

#define ENC_MOTOR_PPR                   11      /* pulses per motor revolution, per phase */
#define ENC_GEAR_RATIO                  45.0f   /* motor revolutions per wheel revolution */
#define ENC_COUNTS_PER_WHEEL_REV        (ENC_MOTOR_PPR * 4 * ENC_GEAR_RATIO)   /* x4 decoding = 1980 */
#define WHEEL_DIAMETER_M                0.065f  /* MEASURE ON THE ROVER */

#define ENC_GLITCH_NS                   1000    /* ignore pulses shorter than 1 us (motor noise) */
#define ENC_SPEED_WINDOW_SAMPLES        5       /* speed over 5 motion periods = 100 ms */
#define ENC_PCNT_HIGH_LIMIT             30000   /* hardware counter range, accumulated in software */
#define ENC_PCNT_LOW_LIMIT              -30000

/* 1 = count every edge of ENC_DIAG_PIN_1 and ENC_DIAG_PIN_2 separately on the two
 * spare PCNT units and log them every 2 s with the level of all four encoder pins.
 * A phase whose edge count stays at 0 while the wheel turns is not reaching the S3. */
#define ENC_DIAG                        0
#define ENC_DIAG_PIN_1                  ENC_LEFT_PIN_A
#define ENC_DIAG_PIN_2                  ENC_LEFT_PIN_B

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

/* Electronic differential in turns: the OUTER wheel keeps the commanded speed and
 * the INNER wheel slows down by the geometric ratio (R - T/2) / (R + T/2).
 * EDIFF_GAIN: 0.0 = both wheels same speed, 1.0 = full geometric ratio.
 * EDIFF_MIN_INNER_RATIO: the inner wheel never goes below this fraction of the
 * outer one (at full lock the geometry alone would give ~0.43). */
#define EDIFF_GAIN                      1.0f
#define EDIFF_MIN_INNER_RATIO           0.6f

/* Closed-loop wheel speed (encoders): each wheel gets feed-forward duty = its
 * command, plus a PI correction so the measured speed follows the command.
 * This keeps both wheels at the same speed going straight and makes the slow
 * inner wheel actually turn in curves. 0 = open loop (duty only). */
#define SPEED_CTRL_ENABLED              1
#define WHEEL_MAX_RPM                   90.0f   /* wheel rpm meant by a 100 % command. Set it a bit below what
                                                 * the SLOWER wheel reaches at 100 % duty on the ground. */
#define SPEED_CTRL_KP                   0.5f    /* % duty per % speed error */
#define SPEED_CTRL_KI                   2.0f    /* % duty per (% speed error x s) */
#define SPEED_CTRL_MAX_CORR_PCT         30.0f   /* max correction added to the feed-forward duty */

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

/* Fail-safe: FAILSAFE if the drive bridge has not submitted a command for this long
 * (only happens if the bridge task stalls; a silent GUI gives neutral, see GUI_TIMEOUT_MS) */
#define LINK_TIMEOUT_MS                 500

/* =========================================================================
 * 5. WI-FI SOFT-AP
 * ========================================================================= */
#define WIFI_AP_SSID                    "ROVER_EXPLORER_AP"
#define WIFI_AP_PASS                    "rover1234"     /* >= 8 chars, "" = open network */
#define WIFI_AP_CHANNEL                 1
#define WIFI_AP_MAX_CONN                4
/* AP IP is the IDF default: 192.168.4.1 */

/* =========================================================================
 * 6. ARDUCAM MINI 2MP PLUS (OV2640) - original schematic pinout
 * ========================================================================= */
/* SPI: GPIO10-13 are the S3's FSPI IO_MUX pins, so SPI2_HOST uses them directly */
#define CAM_SPI_HOST                    SPI2_HOST
#define CAM_PIN_CS                      GPIO_NUM_10     /* CS_cam   */
#define CAM_PIN_MOSI                    GPIO_NUM_11     /* MOSI_cam */
#define CAM_PIN_SCK                     GPIO_NUM_12     /* SCK_cam  */
#define CAM_PIN_MISO                    GPIO_NUM_13     /* MISO_cam */
#define CAM_SPI_FREQ_HZ                 8000000         /* ArduChip limit */

/* SCCB: hardware I2C port, or -1 for software (bit-banged) SCCB.
 * Must not be a port used by an enabled sensor (MPU6050 = 0, BME690 = 1). */
#define CAM_I2C_PORT                    -1              /* software: both controllers are taken */
#define CAM_PIN_SDA                     GPIO_NUM_47     /* SDA_cam */
#define CAM_PIN_SCL                     GPIO_NUM_48     /* SCL_cam */
#define CAM_I2C_FREQ_HZ                 100000

#if MPU6050_ENABLED && (CAM_I2C_PORT == MPU6050_I2C_PORT)
#error "CAM_I2C_PORT collides with the MPU6050 bus: disable the MPU6050 or set CAM_I2C_PORT to -1"
#endif
#if CAM_I2C_PORT == BME690_I2C_PORT
#error "CAM_I2C_PORT collides with the BME690 bus: set CAM_I2C_PORT to -1"
#endif

/* Frame buffers (x2). With PSRAM they go there and up to 1600x1200 is allowed.
 * If PSRAM is missing, camera_service falls back to internal RAM and VGA. */
#define CAM_FRAME_BUF_SIZE_PSRAM        (512 * 1024)
#define CAM_MAX_RES_PSRAM               8               /* OV2640_RES_1600x1200 */
#define CAM_FRAME_BUF_SIZE_INTERNAL     (40 * 1024)
#define CAM_MAX_RES_INTERNAL            4               /* OV2640_RES_640x480 */
#define CAM_DEFAULT_RES                 2               /* OV2640_RES_320x240 */
#define CAM_MIN_FRAME_INTERVAL_MS       40              /* cap at ~25 fps */

/* 1 = no camera: step through the six camera lines forever so they can be
 * checked with a multimeter at the camera pins (see camera_service.c).
 * Driving and the GUI keep working. Set back to 0 for normal use. */
#define CAM_WIRING_TEST                 0

/* =========================================================================
 * 7. CONTROL INPUT / SAFETY
 * ========================================================================= */
/* No control message from any web/Python client for this long -> neutral */
#define GUI_TIMEOUT_MS                  500
#define DRIVE_MAX_SPEED_PCT             100.0f
#define DRIVE_MAX_STEER_DEG             45.0f

/* =========================================================================
 * 8. WEB SERVER
 * ========================================================================= */
#define WEB_HTTP_PORT                   80      /* GUI, WebSocket, REST, /capture */
#define WEB_STREAM_PORT                 81      /* MJPEG /stream (separate server) */
#define WEB_MAX_WS_CLIENTS              4
#define WEB_SERVER_CORE                 0

/* =========================================================================
 * 9. TASK LAYOUT (core, priority, period)
 * ========================================================================= */
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

/* Web command -> motion_control, fixed rate */
#define TASK_DRIVE_BRIDGE_CORE          0
#define TASK_DRIVE_BRIDGE_PRIO          8
#define TASK_DRIVE_BRIDGE_STACK         3072
#define TASK_DRIVE_BRIDGE_PERIOD_MS     50      /* 20 Hz */

#define TASK_WS_TLM_CORE                0
#define TASK_WS_TLM_PRIO                5
#define TASK_WS_TLM_STACK               4096
#define TASK_WS_TLM_PERIOD_MS           50      /* 20 Hz telemetry push to browsers */

#define TASK_CAM_CORE                   0
#define TASK_CAM_PRIO                   4
#define TASK_CAM_STACK                  4096

#define TASK_LOG_CORE                   0
#define TASK_LOG_PRIO                   2
#define TASK_LOG_STACK                  4096
#define TASK_LOG_PERIOD_MS              2000    /* human-readable status line */

#ifdef __cplusplus
}
#endif
