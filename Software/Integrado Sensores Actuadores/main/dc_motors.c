/**
 * @file dc_motors.c
 * @brief MCPWM control implementation for dual DC motors with DRV8873H driver logic.
 */

#include "dc_motors.h"
#include "project_config.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "DC_MOTORS";

typedef struct {
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t  oper_m1;
    mcpwm_oper_handle_t  oper_m2;
    mcpwm_cmpr_handle_t  cmpr_in1_m1;
    mcpwm_cmpr_handle_t  cmpr_in2_m1;
    mcpwm_cmpr_handle_t  cmpr_in1_m2;
    mcpwm_cmpr_handle_t  cmpr_in2_m2;
    mcpwm_gen_handle_t   gen_in1_m1;
    mcpwm_gen_handle_t   gen_in2_m1;
    mcpwm_gen_handle_t   gen_in1_m2;
    mcpwm_gen_handle_t   gen_in2_m2;
    uint32_t             period_ticks;
    float                cur_speed_m1;
    float                cur_speed_m2;
} dc_motors_ctx_t;

static dc_motors_ctx_t s_ctx;

esp_err_t dc_motors_init(void)
{
    ESP_LOGI(TAG, "Initializing MCPWM Dual Motor Control...");

    // 1. Configure DISABLE GPIO outputs (Active-LOW: 0 = Enable H-Bridge)
    gpio_config_t dis_io_conf = {
        .pin_bit_mask = (1ULL << M1_PIN_DISABLE) | (1ULL << M2_PIN_DISABLE),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&dis_io_conf));
    gpio_set_level(M1_PIN_DISABLE, 0);
    gpio_set_level(M2_PIN_DISABLE, 0);

    // Wait t(WAKE) stabilization time (10ms)
    vTaskDelay(pdMS_TO_TICKS(10));

    // 2. Configure nFAULT GPIO inputs with pull-up
    gpio_config_t fault_io_conf = {
        .pin_bit_mask = (1ULL << M1_PIN_NFAULT) | (1ULL << M2_PIN_NFAULT),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&fault_io_conf));

    // 2. Configure MCPWM Timer
    s_ctx.period_ticks = 1000000 / MOTOR_PWM_FREQ_HZ;
    mcpwm_timer_config_t timer_cfg = {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks  = s_ctx.period_ticks,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &s_ctx.timer));

    // 3. Configure Operators for Motor 1 & Motor 2
    mcpwm_operator_config_t oper_cfg = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &s_ctx.oper_m1));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(s_ctx.oper_m1, s_ctx.timer));

    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &s_ctx.oper_m2));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(s_ctx.oper_m2, s_ctx.timer));

    // 4. Configure Comparators
    mcpwm_comparator_config_t cmpr_cfg = {
        .flags = { .update_cmp_on_tez = true },
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(s_ctx.oper_m1, &cmpr_cfg, &s_ctx.cmpr_in1_m1));
    ESP_ERROR_CHECK(mcpwm_new_comparator(s_ctx.oper_m1, &cmpr_cfg, &s_ctx.cmpr_in2_m1));
    ESP_ERROR_CHECK(mcpwm_new_comparator(s_ctx.oper_m2, &cmpr_cfg, &s_ctx.cmpr_in1_m2));
    ESP_ERROR_CHECK(mcpwm_new_comparator(s_ctx.oper_m2, &cmpr_cfg, &s_ctx.cmpr_in2_m2));

    // Set initial comparator compare values to 0
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_ctx.cmpr_in1_m1, 0));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_ctx.cmpr_in2_m1, 0));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_ctx.cmpr_in1_m2, 0));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_ctx.cmpr_in2_m2, 0));

    // 5. Configure Generators
    mcpwm_generator_config_t gen_cfg = { .gen_gpio_num = M1_PIN_IN1 };
    ESP_ERROR_CHECK(mcpwm_new_generator(s_ctx.oper_m1, &gen_cfg, &s_ctx.gen_in1_m1));

    gen_cfg.gen_gpio_num = M1_PIN_IN2;
    ESP_ERROR_CHECK(mcpwm_new_generator(s_ctx.oper_m1, &gen_cfg, &s_ctx.gen_in2_m1));

    gen_cfg.gen_gpio_num = M2_PIN_IN1;
    ESP_ERROR_CHECK(mcpwm_new_generator(s_ctx.oper_m2, &gen_cfg, &s_ctx.gen_in1_m2));

    gen_cfg.gen_gpio_num = M2_PIN_IN2;
    ESP_ERROR_CHECK(mcpwm_new_generator(s_ctx.oper_m2, &gen_cfg, &s_ctx.gen_in2_m2));

    // 6. Action configs for generators
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        s_ctx.gen_in1_m1, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        s_ctx.gen_in1_m1, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_ctx.cmpr_in1_m1, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        s_ctx.gen_in2_m1, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        s_ctx.gen_in2_m1, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_ctx.cmpr_in2_m1, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        s_ctx.gen_in1_m2, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        s_ctx.gen_in1_m2, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_ctx.cmpr_in1_m2, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        s_ctx.gen_in2_m2, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        s_ctx.gen_in2_m2, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_ctx.cmpr_in2_m2, MCPWM_GEN_ACTION_LOW)));

    // 7. Start Timer
    ESP_ERROR_CHECK(mcpwm_timer_enable(s_ctx.timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(s_ctx.timer, MCPWM_TIMER_START_NO_STOP));

    s_ctx.cur_speed_m1 = 0.0f;
    s_ctx.cur_speed_m2 = 0.0f;

    ESP_LOGI(TAG, "MCPWM Dual Motor Driver successfully initialized.");
    return ESP_OK;
}

static void apply_motor_channel(mcpwm_cmpr_handle_t cmpr_in1, mcpwm_cmpr_handle_t cmpr_in2, float speed, uint32_t period_ticks)
{
    if (speed > 100.0f) speed = 100.0f;
    if (speed < -100.0f) speed = -100.0f;

    uint32_t ticks = (uint32_t)((fabsf(speed) / 100.0f) * period_ticks);

    if (speed > 0.0f) {
        // Forward: IN1 has PWM duty, IN2 remains 0
        mcpwm_comparator_set_compare_value(cmpr_in1, ticks);
        mcpwm_comparator_set_compare_value(cmpr_in2, 0);
    } else if (speed < 0.0f) {
        // Reverse: IN1 remains 0, IN2 has PWM duty
        mcpwm_comparator_set_compare_value(cmpr_in1, 0);
        mcpwm_comparator_set_compare_value(cmpr_in2, ticks);
    } else {
        // Stop / Brake: Both low
        mcpwm_comparator_set_compare_value(cmpr_in1, 0);
        mcpwm_comparator_set_compare_value(cmpr_in2, 0);
    }
}

esp_err_t dc_motors_set_speed(float speed_m1, float speed_m2)
{
    apply_motor_channel(s_ctx.cmpr_in1_m1, s_ctx.cmpr_in2_m1, speed_m1, s_ctx.period_ticks);
    apply_motor_channel(s_ctx.cmpr_in1_m2, s_ctx.cmpr_in2_m2, speed_m2, s_ctx.period_ticks);

    s_ctx.cur_speed_m1 = speed_m1;
    s_ctx.cur_speed_m2 = speed_m2;
    return ESP_OK;
}

esp_err_t dc_motors_stop(void)
{
    return dc_motors_set_speed(0.0f, 0.0f);
}

bool dc_motors_check_fault(void)
{
    int fault_m1 = gpio_get_level(M1_PIN_NFAULT);
    int fault_m2 = gpio_get_level(M2_PIN_NFAULT);

    // Active LOW: 0 indicates fault
    if (fault_m1 == 0 || fault_m2 == 0) {
        return true;
    }
    return false;
}

void dc_motors_get_speeds(float *speed_m1, float *speed_m2)
{
    if (speed_m1) *speed_m1 = s_ctx.cur_speed_m1;
    if (speed_m2) *speed_m2 = s_ctx.cur_speed_m2;
}
