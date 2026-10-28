/**
 * @file drive_command.c
 * @brief Latest drive command + GUI watchdog.
 */

#include "drive_command.h"
#include "app_config.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"

#define DRIVE_ESTOP_HOLD_MS     1000

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static float    s_speed = 0.0f;
static float    s_steer = 0.0f;
static uint32_t s_last_set_ms = 0;
static uint32_t s_estop_until_ms = 0;
static bool     s_ever_set = false;

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

void drive_command_init(void)
{
    taskENTER_CRITICAL(&s_lock);
    s_speed = 0.0f;
    s_steer = 0.0f;
    s_ever_set = false;
    taskEXIT_CRITICAL(&s_lock);
}

void drive_command_set(float speed_pct, float steer_deg)
{
    /* NaN guard: (x != x) is true only for NaN */
    if (speed_pct != speed_pct) speed_pct = 0.0f;
    if (steer_deg != steer_deg) steer_deg = 0.0f;

    taskENTER_CRITICAL(&s_lock);
    s_speed = clampf(speed_pct, -DRIVE_MAX_SPEED_PCT, DRIVE_MAX_SPEED_PCT);
    s_steer = clampf(steer_deg, -DRIVE_MAX_STEER_DEG, DRIVE_MAX_STEER_DEG);
    s_last_set_ms = now_ms();
    s_ever_set = true;
    taskEXIT_CRITICAL(&s_lock);
}

void drive_command_estop(void)
{
    taskENTER_CRITICAL(&s_lock);
    s_speed = 0.0f;
    s_steer = 0.0f;
    s_estop_until_ms = now_ms() + DRIVE_ESTOP_HOLD_MS;
    s_last_set_ms = now_ms();
    s_ever_set = true;
    taskEXIT_CRITICAL(&s_lock);
}

void drive_command_get(drive_command_state_t *out)
{
    uint32_t t = now_ms();

    taskENTER_CRITICAL(&s_lock);
    uint32_t age = s_ever_set ? (t - s_last_set_ms) : UINT32_MAX;
    bool active = s_ever_set && age <= GUI_TIMEOUT_MS;
    bool estop = (int32_t)(s_estop_until_ms - t) > 0;

    out->client_active = active;
    out->age_ms = age;
    out->estop = estop;
    out->speed_pct = (active && !estop) ? s_speed : 0.0f;
    out->steer_deg = (active && !estop) ? s_steer : 0.0f;
    taskEXIT_CRITICAL(&s_lock);
}

void drive_command_get_payload(rp_cmd_drive_t *out)
{
    drive_command_state_t st;
    drive_command_get(&st);
    out->speed_x10 = (int16_t)(st.speed_pct * 10.0f);
    out->steer_x10 = (int16_t)(st.steer_deg * 10.0f);
    out->flags = st.estop ? RP_DRIVE_FLAG_ESTOP : 0;
}
