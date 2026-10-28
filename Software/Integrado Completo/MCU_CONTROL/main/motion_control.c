/**
 * @file motion_control.c
 * @brief Motion controller implementation (see motion_control.h for the pipeline).
 */

#include "motion_control.h"
#include "app_config.h"
#include "kinematics.h"
#include "dc_motors.h"
#include "servo.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "MOTION";

typedef struct {
    motion_cmd_t cmd;
    uint32_t     rx_ms;
} mailbox_item_t;

static QueueHandle_t   s_mailbox = NULL;      /* length 1, xQueueOverwrite */
static portMUX_TYPE    s_status_lock = portMUX_INITIALIZER_UNLOCKED;
static motion_status_t s_status;

/* Controller internal state (only touched by the motion task) */
static rp_motion_state_t s_state = RP_MOTION_DISARMED;
static float s_speed_pct = 0.0f;   /* ramped */
static float s_steer_deg = 0.0f;   /* ramped */

static const kinematics_cfg_t s_kin_cfg = {
    .wheelbase_m   = VEHICLE_WHEELBASE_M,
    .track_m       = VEHICLE_TRACK_M,
    .ediff_gain    = EDIFF_GAIN,
    .max_speed_pct = MOTION_MAX_SPEED_PCT,
    .max_steer_deg = SERVO_MAX_STEER_DEG,
};

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static const char *state_name(rp_motion_state_t s)
{
    switch (s) {
    case RP_MOTION_DISARMED: return "DISARMED";
    case RP_MOTION_ARMED:    return "ARMED";
    case RP_MOTION_FAILSAFE: return "FAILSAFE";
    case RP_MOTION_FAULT:    return "FAULT";
    default:                 return "?";
    }
}

/* =========================================================================
 * Stage 1: input - newest command and its freshness
 * ========================================================================= */
static bool read_command(motion_cmd_t *cmd, bool *link_ok)
{
    mailbox_item_t item;
    if (xQueuePeek(s_mailbox, &item, 0) != pdTRUE) {
        *link_ok = false;
        memset(cmd, 0, sizeof(*cmd));
        return false;
    }
    *cmd = item.cmd;
    *link_ok = (uint32_t)(now_ms() - item.rx_ms) <= LINK_TIMEOUT_MS;
    return true;
}

static bool is_neutral(const motion_cmd_t *cmd)
{
    return fabsf(cmd->speed_pct) < MOTION_NEUTRAL_SPEED_PCT;
}

/* =========================================================================
 * Stage 2: safety state machine -> returns the target to track
 * ========================================================================= */
static void set_state(rp_motion_state_t next, const char *reason)
{
    if (next == s_state) return;
    ESP_LOGW(TAG, "State %s -> %s (%s)", state_name(s_state), state_name(next), reason);
    s_state = next;
}

static void update_state(const motion_cmd_t *cmd, bool link_ok, bool fault,
                         float *target_speed, float *target_steer)
{
    bool can_arm = link_ok && !cmd->estop && is_neutral(cmd) && !fault;

    if (fault && s_state != RP_MOTION_FAULT) {
        set_state(RP_MOTION_FAULT, "DRV8873 nFAULT");
    }

    switch (s_state) {
    case RP_MOTION_DISARMED:
        if (can_arm) set_state(RP_MOTION_ARMED, "neutral command received");
        break;
    case RP_MOTION_ARMED:
        if (!link_ok)          set_state(RP_MOTION_FAILSAFE, "link timeout");
        else if (cmd->estop)   set_state(RP_MOTION_FAILSAFE, "e-stop");
        break;
    case RP_MOTION_FAILSAFE:
        /* re-arm only once stopped and the operator is back at neutral */
        if (can_arm && s_speed_pct == 0.0f) set_state(RP_MOTION_ARMED, "link restored, neutral");
        break;
    case RP_MOTION_FAULT:
        if (can_arm) set_state(RP_MOTION_ARMED, "fault cleared, neutral");
        break;
    }

    if (s_state == RP_MOTION_ARMED) {
        *target_speed = cmd->speed_pct;
        *target_steer = cmd->steer_deg;
    } else {
        *target_speed = 0.0f;
        *target_steer = 0.0f;
    }
}

/* =========================================================================
 * Stage 3: slew-rate limiter
 * ========================================================================= */
static float slew(float cur, float target, float up_rate, float down_rate, float dt)
{
    /* "up" = magnitude growing in the same direction; anything else is braking */
    bool growing = (fabsf(target) > fabsf(cur)) && (target * cur >= 0.0f);
    float max_step = (growing ? up_rate : down_rate) * dt;
    float diff = target - cur;
    if (diff > max_step)  diff = max_step;
    if (diff < -max_step) diff = -max_step;
    float next = cur + diff;
    /* don't overshoot through zero with the braking rate in one step */
    if (!growing && cur * next < 0.0f) next = 0.0f;
    return next;
}

static void apply_ramps(float target_speed, float target_steer, float dt)
{
    if (s_state == RP_MOTION_FAULT) {
        s_speed_pct = 0.0f;                 /* hardware fault: no ramp */
    } else {
        float decel = (s_state == RP_MOTION_ARMED) ? MOTION_DECEL_PCT_PER_S
                                                   : MOTION_ESTOP_DECEL_PCT_PER_S;
        s_speed_pct = slew(s_speed_pct, target_speed, MOTION_ACCEL_PCT_PER_S, decel, dt);
    }
    s_steer_deg = slew(s_steer_deg, target_steer, MOTION_STEER_DEG_PER_S, MOTION_STEER_DEG_PER_S, dt);
}

/* =========================================================================
 * Stage 4: dead-band / minimum duty compensation
 * ========================================================================= */
static float deadband_comp(float duty)
{
    float mag = fabsf(duty);
    if (mag < MOTOR_DEADBAND_PCT) {
        return 0.0f;
    }
    float out = MOTOR_MIN_DUTY_PCT +
                (mag - MOTOR_DEADBAND_PCT) * (100.0f - MOTOR_MIN_DUTY_PCT) / (100.0f - MOTOR_DEADBAND_PCT);
    return copysignf(fminf(out, 100.0f), duty);
}

/* =========================================================================
 * Stage 5: output - wheel mapping, polarity, enable, hardware
 * ========================================================================= */
static void drive_outputs(const wheel_cmd_t *w)
{
    bool outputs_on = (s_state == RP_MOTION_ARMED) ||
                      (s_state == RP_MOTION_FAILSAFE && s_speed_pct != 0.0f);

    float left  = outputs_on ? deadband_comp(w->left_pct)  : 0.0f;
    float right = outputs_on ? deadband_comp(w->right_pct) : 0.0f;

#if MOTOR_LEFT_INVERT
    left = -left;
#endif
#if MOTOR_RIGHT_INVERT
    right = -right;
#endif

    if (outputs_on && !dc_motors_is_enabled()) {
        dc_motors_enable(true);
    } else if (!outputs_on && dc_motors_is_enabled()) {
        dc_motors_enable(false);            /* also sets duty to 0 -> coast */
    }

    if (outputs_on) {
#if MOTOR_LEFT_IS_M1
        dc_motors_set_speed(left, right);
#else
        dc_motors_set_speed(right, left);
#endif
    }

    if (fabsf(w->steer_deg - steering_get_deg()) > 0.05f) {
        steering_set_deg(w->steer_deg);
    }

    taskENTER_CRITICAL(&s_status_lock);
    s_status.left_pct  = outputs_on ? left  : 0.0f;
    s_status.right_pct = outputs_on ? right : 0.0f;
    taskEXIT_CRITICAL(&s_status_lock);
}

/* =========================================================================
 * Task
 * ========================================================================= */
static void motion_task(void *arg)
{
    (void)arg;
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    const float dt = TASK_MOTION_PERIOD_MS / 1000.0f;
    TickType_t last_wake = xTaskGetTickCount();

    ESP_LOGI(TAG, "Motion task running at %d Hz (link timeout %d ms)",
             1000 / TASK_MOTION_PERIOD_MS, LINK_TIMEOUT_MS);

    while (1) {
        motion_cmd_t cmd;
        bool link_ok = false;
        read_command(&cmd, &link_ok);
        bool fault = dc_motors_check_fault();

        float target_speed, target_steer;
        update_state(&cmd, link_ok, fault, &target_speed, &target_steer);
        apply_ramps(target_speed, target_steer, dt);

        wheel_cmd_t wheels;
        kinematics_ackermann(s_speed_pct, s_steer_deg, &s_kin_cfg, &wheels);
        drive_outputs(&wheels);

        taskENTER_CRITICAL(&s_status_lock);
        s_status.state     = s_state;
        s_status.link_ok   = link_ok;
        s_status.fault     = fault;
        s_status.speed_pct = s_speed_pct;
        s_status.steer_deg = s_steer_deg;
        taskEXIT_CRITICAL(&s_status_lock);

        esp_task_wdt_reset();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_MOTION_PERIOD_MS));
    }
}

esp_err_t motion_control_init(void)
{
    s_mailbox = xQueueCreate(1, sizeof(mailbox_item_t));
    if (s_mailbox == NULL) {
        return ESP_ERR_NO_MEM;
    }
    memset(&s_status, 0, sizeof(s_status));
    s_status.state = RP_MOTION_DISARMED;
    dc_motors_enable(false);
    return ESP_OK;
}

esp_err_t motion_control_start(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(motion_task, "motion", TASK_MOTION_STACK, NULL,
                                            TASK_MOTION_PRIO, NULL, TASK_MOTION_CORE);
    return (ok == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}

void motion_control_submit(const motion_cmd_t *cmd)
{
    if (s_mailbox == NULL || cmd == NULL) return;
    mailbox_item_t item = { .cmd = *cmd, .rx_ms = now_ms() };
    xQueueOverwrite(s_mailbox, &item);
}

void motion_control_get_status(motion_status_t *out)
{
    taskENTER_CRITICAL(&s_status_lock);
    *out = s_status;
    taskEXIT_CRITICAL(&s_status_lock);
}
