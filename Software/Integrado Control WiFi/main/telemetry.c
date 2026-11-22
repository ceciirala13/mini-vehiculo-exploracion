/**
 * @file telemetry.c
 * @brief Telemetry snapshot + JSON rendering.
 */

#include "telemetry.h"
#include "app_config.h"
#include "camera_service.h"
#include "drive_command.h"
#include "encoders.h"
#include "motion_control.h"
#include "vehicle_state.h"
#include "wifi_ap.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>

static const char *state_name(uint8_t s)
{
    switch (s) {
    case RP_MOTION_DISARMED: return "DISARMED";
    case RP_MOTION_ARMED:    return "ARMED";
    case RP_MOTION_FAILSAFE: return "FAILSAFE";
    default:                 return "UNKNOWN";
    }
}

static const char *b(bool v)
{
    return v ? "true" : "false";
}

void telemetry_build(rover_telemetry_t *t)
{
    vehicle_state_t vs;
    motion_status_t ms;
    encoders_state_t enc;
    vehicle_state_get_copy(&vs);
    motion_control_get_status(&ms);
    encoders_get(&enc);

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

    t->motor_left_pct  = ms.left_pct;
    t->motor_right_pct = ms.right_pct;
    t->steer_deg       = ms.steer_deg;

    t->enc_count_left  = enc.count_left;
    t->enc_count_right = enc.count_right;
    t->wheel_rpm_left  = enc.rpm_left;
    t->wheel_rpm_right = enc.rpm_right;
    t->wheel_mps_left  = enc.mps_left;
    t->wheel_mps_right = enc.mps_right;
    t->odom_m          = enc.dist_m;

    uint8_t st = 0;
    if (vs.imu_ok)                         st |= RP_ST_MPU_OK;
    if (vs.env_ok)                         st |= RP_ST_BME_OK;
    if (vs.env_ok && vs.env.gas_valid)     st |= RP_ST_GAS_VALID;
    if (enc.ok)                            st |= RP_ST_ENC_OK;
    if (ms.state == RP_MOTION_FAILSAFE)    st |= RP_ST_FAILSAFE;
    if (ms.state == RP_MOTION_ARMED)       st |= RP_ST_ARMED;
    if (ms.link_ok)                        st |= RP_ST_LINK_OK;
    t->status       = st;
    t->motion_state = (uint8_t)ms.state;
}

size_t telemetry_to_json(char *buf, size_t size)
{
    rover_telemetry_t t;
    telemetry_build(&t);

    drive_command_state_t cmd;
    drive_command_get(&cmd);

    const bool bme_ok   = (t.status & RP_ST_BME_OK) != 0;
    const bool mpu_ok   = (t.status & RP_ST_MPU_OK) != 0;
    const bool gas_ok   = (t.status & RP_ST_GAS_VALID) != 0;
    const bool enc_ok   = (t.status & RP_ST_ENC_OK) != 0;
    const bool failsafe = (t.status & RP_ST_FAILSAFE) != 0;
    const bool armed    = (t.status & RP_ST_ARMED) != 0;
    const bool cmd_ok   = (t.status & RP_ST_LINK_OK) != 0;

    int n = snprintf(buf, size,
        "{"
        "\"rover_connected\":true,\"uptime_ms\":%lu,\"proto\":%u,"
        "\"temp\":%.2f,\"hum\":%.2f,\"press\":%.2f,\"gas\":%.2f,\"gas_valid\":%s,"
        "\"bme_ok\":%s,\"mpu_ok\":%s,"
        "\"imu\":{\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
        "\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f},"
        "\"motors\":{\"left\":%.1f,\"right\":%.1f},\"servo\":%.1f,"
        "\"enc\":{\"ok\":%s,\"left_rpm\":%.1f,\"right_rpm\":%.1f,\"left_mps\":%.3f,\"right_mps\":%.3f,"
        "\"left_count\":%ld,\"right_count\":%ld,\"dist_m\":%.3f},"
        "\"failsafe\":%s,\"armed\":%s,\"state\":\"%s\","
        "\"cmd\":{\"speed\":%.1f,\"angle\":%.1f,\"estop\":%s,\"client_active\":%s,\"age_ms\":%ld,\"bridge_ok\":%s},"
        "\"sys\":{\"heap\":%u,\"min_heap\":%u,\"psram\":%u,\"stations\":%d},"
        "\"cam\":{\"ok\":%s,\"fps\":%.1f,\"size\":%u,\"res\":%d,\"max_res\":%d}"
        "}",
        (unsigned long)t.uptime_ms, (unsigned)ROVER_FW_VERSION,
        t.temp_c, t.hum_pct, t.press_hpa, t.gas_ohm / 1000.0f, b(gas_ok),
        b(bme_ok), b(mpu_ok),
        t.ax_g, t.ay_g, t.az_g, t.gx_dps, t.gy_dps, t.gz_dps,
        t.roll_deg, t.pitch_deg, t.yaw_deg,
        t.motor_left_pct, t.motor_right_pct, t.steer_deg,
        b(enc_ok), t.wheel_rpm_left, t.wheel_rpm_right, t.wheel_mps_left, t.wheel_mps_right,
        (long)t.enc_count_left, (long)t.enc_count_right, t.odom_m,
        b(failsafe), b(armed), state_name(t.motion_state),
        cmd.speed_pct, cmd.steer_deg, b(cmd.estop), b(cmd.client_active),
        (cmd.age_ms == UINT32_MAX) ? -1L : (long)cmd.age_ms, b(cmd_ok),
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
        (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
        wifi_ap_station_count(),
        b(camera_service_available()), camera_service_get_fps(),
        (unsigned)camera_service_get_last_size(), camera_service_get_resolution(),
        camera_service_get_max_resolution());

    return (n > 0 && (size_t)n < size) ? (size_t)n : 0;
}
