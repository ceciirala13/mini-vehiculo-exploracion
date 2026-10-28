/**
 * @file dc_motors.c
 * @brief MCPWM control implementation for dual DC motors with DRV8873H driver logic.
 *
 * One MCPWM timer (20 kHz) shared by two operators, one per motor.
 * Each motor has two generators: IN1 carries the PWM when going forward,
 * IN2 when going in reverse; the idle input stays low.
 */

#include "dc_motors.h"
#include "app_config.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "DC_MOTORS";

#define MCPWM_RESOLUTION_HZ     1000000

typedef struct {
    gpio_num_t          pin_in1;
    gpio_num_t          pin_in2;
    gpio_num_t          pin_disable;
    gpio_num_t          pin_nfault;
    mcpwm_oper_handle_t oper;
    mcpwm_cmpr_handle_t cmpr_in1;
    mcpwm_cmpr_handle_t cmpr_in2;
    mcpwm_gen_handle_t  gen_in1;
    mcpwm_gen_handle_t  gen_in2;
    float               duty;
} motor_channel_t;

static mcpwm_timer_handle_t s_timer = NULL;
static uint32_t             s_period_ticks = 0;
static bool                 s_enabled = false;

static motor_channel_t s_motors[2] = {
    [DC_MOTOR_1] = { .pin_in1 = M1_PIN_IN1, .pin_in2 = M1_PIN_IN2,
                     .pin_disable = M1_PIN_DISABLE, .pin_nfault = M1_PIN_NFAULT },
    [DC_MOTOR_2] = { .pin_in1 = M2_PIN_IN1, .pin_in2 = M2_PIN_IN2,
                     .pin_disable = M2_PIN_DISABLE, .pin_nfault = M2_PIN_NFAULT },
};

static esp_err_t init_channel(motor_channel_t *m)
{
    mcpwm_operator_config_t oper_cfg = { .group_id = 0 };
    esp_err_t ret = mcpwm_new_operator(&oper_cfg, &m->oper);
    if (ret != ESP_OK) return ret;
    ret = mcpwm_operator_connect_timer(m->oper, s_timer);
    if (ret != ESP_OK) return ret;

    mcpwm_comparator_config_t cmpr_cfg = { .flags.update_cmp_on_tez = true };
    ret = mcpwm_new_comparator(m->oper, &cmpr_cfg, &m->cmpr_in1);
    if (ret != ESP_OK) return ret;
    ret = mcpwm_new_comparator(m->oper, &cmpr_cfg, &m->cmpr_in2);
    if (ret != ESP_OK) return ret;
    mcpwm_comparator_set_compare_value(m->cmpr_in1, 0);
    mcpwm_comparator_set_compare_value(m->cmpr_in2, 0);

    mcpwm_generator_config_t gen_cfg = { .gen_gpio_num = m->pin_in1 };
    ret = mcpwm_new_generator(m->oper, &gen_cfg, &m->gen_in1);
    if (ret != ESP_OK) return ret;
    gen_cfg.gen_gpio_num = m->pin_in2;
    ret = mcpwm_new_generator(m->oper, &gen_cfg, &m->gen_in2);
    if (ret != ESP_OK) return ret;

    /* High on timer empty, low on compare match -> duty = compare / period */
    mcpwm_gen_handle_t gens[2]  = { m->gen_in1, m->gen_in2 };
    mcpwm_cmpr_handle_t cmprs[2] = { m->cmpr_in1, m->cmpr_in2 };
    for (int i = 0; i < 2; i++) {
        ret = mcpwm_generator_set_action_on_timer_event(gens[i],
                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
        if (ret != ESP_OK) return ret;
        ret = mcpwm_generator_set_action_on_compare_event(gens[i],
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmprs[i], MCPWM_GEN_ACTION_LOW));
        if (ret != ESP_OK) return ret;
    }

    m->duty = 0.0f;
    return ESP_OK;
}

esp_err_t dc_motors_init(void)
{
    ESP_LOGI(TAG, "Initializing MCPWM dual motor control (%d Hz)...", MOTOR_PWM_FREQ_HZ);

    /* 1. DISABLE pins: start disabled (high = outputs Hi-Z) */
    gpio_config_t dis_io_conf = {
        .pin_bit_mask = (1ULL << M1_PIN_DISABLE) | (1ULL << M2_PIN_DISABLE),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&dis_io_conf);
    if (ret != ESP_OK) return ret;
    gpio_set_level(M1_PIN_DISABLE, 1);
    gpio_set_level(M2_PIN_DISABLE, 1);

    /* 2. nFAULT inputs with pull-up (open-drain, active low) */
    gpio_config_t fault_io_conf = {
        .pin_bit_mask = (1ULL << M1_PIN_NFAULT) | (1ULL << M2_PIN_NFAULT),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&fault_io_conf);
    if (ret != ESP_OK) return ret;

    /* 3. Shared timer */
    s_period_ticks = MCPWM_RESOLUTION_HZ / MOTOR_PWM_FREQ_HZ;
    mcpwm_timer_config_t timer_cfg = {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MCPWM_RESOLUTION_HZ,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks  = s_period_ticks,
    };
    ret = mcpwm_new_timer(&timer_cfg, &s_timer);
    if (ret != ESP_OK) return ret;

    /* 4. One operator + 2 generators per motor */
    for (int i = 0; i < 2; i++) {
        ret = init_channel(&s_motors[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Motor %d channel init failed: %s", i + 1, esp_err_to_name(ret));
            return ret;
        }
    }

    /* 5. Start timer */
    ret = mcpwm_timer_enable(s_timer);
    if (ret != ESP_OK) return ret;
    ret = mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK) return ret;

    s_enabled = false;
    ESP_LOGI(TAG, "MCPWM dual motor driver initialized (outputs disabled until armed).");
    return ESP_OK;
}

static void apply_channel(motor_channel_t *m, float duty)
{
    if (duty > 100.0f)  duty = 100.0f;
    if (duty < -100.0f) duty = -100.0f;

    uint32_t ticks = (uint32_t)((fabsf(duty) / 100.0f) * (float)s_period_ticks);

    if (duty > 0.0f) {
        mcpwm_comparator_set_compare_value(m->cmpr_in1, ticks);
        mcpwm_comparator_set_compare_value(m->cmpr_in2, 0);
    } else if (duty < 0.0f) {
        mcpwm_comparator_set_compare_value(m->cmpr_in1, 0);
        mcpwm_comparator_set_compare_value(m->cmpr_in2, ticks);
    } else {
        mcpwm_comparator_set_compare_value(m->cmpr_in1, 0);
        mcpwm_comparator_set_compare_value(m->cmpr_in2, 0);
    }
    m->duty = duty;
}

esp_err_t dc_motors_set_speed(float duty_m1, float duty_m2)
{
    if (s_timer == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    apply_channel(&s_motors[DC_MOTOR_1], duty_m1);
    apply_channel(&s_motors[DC_MOTOR_2], duty_m2);
    return ESP_OK;
}

esp_err_t dc_motors_stop(void)
{
    return dc_motors_set_speed(0.0f, 0.0f);
}

void dc_motors_enable(bool enable)
{
    if (!enable) {
        dc_motors_stop();
    }
    gpio_set_level(M1_PIN_DISABLE, enable ? 0 : 1);
    gpio_set_level(M2_PIN_DISABLE, enable ? 0 : 1);
    if (enable && !s_enabled) {
        vTaskDelay(pdMS_TO_TICKS(2)); /* DRV8873 t(WAKE)/enable settle */
    }
    s_enabled = enable;
}

bool dc_motors_is_enabled(void)
{
    return s_enabled;
}

bool dc_motors_fault(dc_motor_id_t motor)
{
    return gpio_get_level(s_motors[motor].pin_nfault) == 0;
}

bool dc_motors_check_fault(void)
{
    return dc_motors_fault(DC_MOTOR_1) || dc_motors_fault(DC_MOTOR_2);
}

void dc_motors_get_speeds(float *duty_m1, float *duty_m2)
{
    if (duty_m1) *duty_m1 = s_motors[DC_MOTOR_1].duty;
    if (duty_m2) *duty_m2 = s_motors[DC_MOTOR_2].duty;
}
