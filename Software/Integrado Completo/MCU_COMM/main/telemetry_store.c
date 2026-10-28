/**
 * @file telemetry_store.c
 * @brief Latest MCU_CONTROL telemetry + JSON rendering.
 */

#include "telemetry_store.h"
#include "app_config.h"
#include "uart_link.h"
#include "drive_command.h"
#include "camera_service.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>

static portMUX_TYPE   s_lock = portMUX_INITIALIZER_UNLOCKED;
static rp_tlm_state_t s_tlm;
static uint32_t       s_rx_ms = 0;
static bool           s_ever = false;

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static const char *state_name(uint8_t s)
{
    switch (s) {
    case RP_MOTION_DISARMED: return "DISARMED";
    case RP_MOTION_ARMED:    return "ARMED";
    case RP_MOTION_FAILSAFE: return "FAILSAFE";
    case RP_MOTION_FAULT:    return "FAULT";
    default:                 return "UNKNOWN";
    }
}

void telemetry_store_init(void)
{
    taskENTER_CRITICAL(&s_lock);
    memset(&s_tlm, 0, sizeof(s_tlm));
    s_tlm.az_g = 1.0f;
    s_ever = false;
    taskEXIT_CRITICAL(&s_lock);
}

void telemetry_store_update(const rp_tlm_state_t *t)
{
    taskENTER_CRITICAL(&s_lock);
    s_tlm = *t;
    s_rx_ms = now_ms();
    s_ever = true;
    taskEXIT_CRITICAL(&s_lock);
}

bool telemetry_store_get(rp_tlm_state_t *out, uint32_t *age_ms)
{
    uint32_t t = now_ms();
    taskENTER_CRITICAL(&s_lock);
    *out = s_tlm;
    uint32_t age = s_ever ? (t - s_rx_ms) : UINT32_MAX;
    taskEXIT_CRITICAL(&s_lock);
    if (age_ms) *age_ms = age;
    return age <= CTRL_LINK_TIMEOUT_MS;
}

size_t telemetry_store_to_json(char *buf, size_t size)
{
    rp_tlm_state_t t;
    uint32_t age;
    bool online = telemetry_store_get(&t, &age);

    uart_link_stats_t ls;
    uart_link_get_stats(&ls);

    drive_command_state_t cmd;
    drive_command_get(&cmd);

    const bool bme_ok   = (t.status & RP_ST_BME_OK) != 0;
    const bool mpu_ok   = (t.status & RP_ST_MPU_OK) != 0;
    const bool gas_ok   = (t.status & RP_ST_GAS_VALID) != 0;
    const bool fault    = (t.status & RP_ST_MOTOR_FAULT) != 0;
    const bool failsafe = (t.status & RP_ST_FAILSAFE) != 0;
    const bool armed    = (t.status & RP_ST_ARMED) != 0;

    int n = snprintf(buf, size,
        "{"
        "\"rover_connected\":%s,\"ctrl_age_ms\":%ld,\"ctrl_uptime_ms\":%lu,\"proto\":%u,"
        "\"temp\":%.2f,\"hum\":%.2f,\"press\":%.2f,\"gas\":%.2f,\"gas_valid\":%s,"
        "\"bme_ok\":%s,\"mpu_ok\":%s,"
        "\"imu\":{\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
        "\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f},"
        "\"motors\":{\"left\":%.1f,\"right\":%.1f},\"servo\":%.1f,"
        "\"fault\":%s,\"failsafe\":%s,\"armed\":%s,\"state\":\"%s\","
        "\"cmd\":{\"speed\":%.1f,\"angle\":%.1f,\"estop\":%s,\"client_active\":%s},"
        "\"link\":{\"ctrl_rx_ok\":%u,\"ctrl_crc_err\":%u,\"comm_rx_ok\":%lu,\"comm_crc_err\":%lu,"
        "\"comm_timeouts\":%lu,\"comm_tx\":%lu},"
        "\"cam\":{\"ok\":%s,\"fps\":%.1f,\"size\":%u,\"res\":%d}"
        "}",
        online ? "true" : "false",
        (age == UINT32_MAX) ? -1L : (long)age,
        (unsigned long)t.uptime_ms, t.proto_version,
        t.temp_c, t.hum_pct, t.press_hpa, t.gas_ohm / 1000.0f, gas_ok ? "true" : "false",
        bme_ok ? "true" : "false", mpu_ok ? "true" : "false",
        t.ax_g, t.ay_g, t.az_g, t.gx_dps, t.gy_dps, t.gz_dps,
        t.roll_deg, t.pitch_deg, t.yaw_deg,
        t.motor_left_x10 / 10.0f, t.motor_right_x10 / 10.0f, t.steer_x10 / 10.0f,
        fault ? "true" : "false", failsafe ? "true" : "false", armed ? "true" : "false",
        online ? state_name(t.motion_state) : "OFFLINE",
        cmd.speed_pct, cmd.steer_deg, cmd.estop ? "true" : "false", cmd.client_active ? "true" : "false",
        t.rx_ok, t.rx_crc_err, (unsigned long)ls.frames_ok, (unsigned long)ls.crc_errors,
        (unsigned long)ls.timeouts, (unsigned long)ls.tx_frames,
        camera_service_available() ? "true" : "false", camera_service_get_fps(),
        (unsigned)camera_service_get_last_size(), camera_service_get_resolution());

    return (n > 0 && (size_t)n < size) ? (size_t)n : 0;
}
