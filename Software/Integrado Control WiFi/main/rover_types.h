/**
 * @file rover_types.h
 * @brief Shared types: motion state and the telemetry snapshot.
 *
 * Plain in-memory types used by motion_control, telemetry and the web server.
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Firmware/API version reported in /status and the telemetry JSON ("proto") */
#define ROVER_FW_VERSION            2

/* rover_telemetry_t.status bits */
#define RP_ST_MPU_OK                0x01
#define RP_ST_BME_OK                0x02
#define RP_ST_ENC_OK                0x04    /* wheel encoders running (was MOTOR_FAULT) */
#define RP_ST_FAILSAFE              0x08
#define RP_ST_ARMED                 0x10
#define RP_ST_GAS_VALID             0x20
#define RP_ST_LINK_OK               0x40    /* motion_control is receiving fresh commands */

/* Safety state machine (motion_control.c) */
typedef enum {
    RP_MOTION_DISARMED = 0,
    RP_MOTION_ARMED    = 1,
    RP_MOTION_FAILSAFE = 2,
} rp_motion_state_t;

/* Snapshot of everything the GUI shows, built by telemetry.c */
typedef struct {
    uint32_t uptime_ms;

    /* MPU6050 */
    float    ax_g, ay_g, az_g;
    float    gx_dps, gy_dps, gz_dps;
    float    roll_deg, pitch_deg, yaw_deg;

    /* BME690 */
    float    temp_c;
    float    hum_pct;
    float    press_hpa;
    uint32_t gas_ohm;

    /* Actuators (actual, after ramps and compensation) */
    float    motor_left_pct;
    float    motor_right_pct;
    float    steer_deg;

    /* Wheel encoders (+ = forward) */
    int32_t  enc_count_left;
    int32_t  enc_count_right;
    float    wheel_rpm_left;
    float    wheel_rpm_right;
    float    wheel_mps_left;
    float    wheel_mps_right;
    float    odom_m;            /* distance since boot, average of both wheels */

    uint8_t  status;            /* RP_ST_* */
    uint8_t  motion_state;      /* rp_motion_state_t */
} rover_telemetry_t;

#ifdef __cplusplus
}
#endif
