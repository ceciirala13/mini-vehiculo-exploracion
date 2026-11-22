/**
 * @file encoders.c
 * @brief Quadrature wheel encoders on PCNT (see encoders.h).
 */

#include "encoders.h"
#include "app_config.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "ENCODERS";

#define HIST_LEN    (ENC_SPEED_WINDOW_SAMPLES + 1)
#define PI_F        3.14159265f

typedef struct {
    const char        *name;
    gpio_num_t         pin_a;
    gpio_num_t         pin_b;
    bool               invert;
    pcnt_unit_handle_t unit;
    int32_t            hist[HIST_LEN];     /* counts, ring buffer */
} wheel_enc_t;

static wheel_enc_t s_left  = { .name = "left",  .pin_a = ENC_LEFT_PIN_A,  .pin_b = ENC_LEFT_PIN_B,  .invert = ENC_LEFT_INVERT };
static wheel_enc_t s_right = { .name = "right", .pin_a = ENC_RIGHT_PIN_A, .pin_b = ENC_RIGHT_PIN_B, .invert = ENC_RIGHT_INVERT };

static int64_t s_hist_us[HIST_LEN];        /* sample times, same ring index */
static int     s_hist_idx = 0;
static int     s_hist_fill = 0;

static portMUX_TYPE     s_lock = portMUX_INITIALIZER_UNLOCKED;
static encoders_state_t s_state;

/* =========================================================================
 * Init
 * ========================================================================= */
static esp_err_t init_wheel(wheel_enc_t *w)
{
    /* Take the pins over from whatever had them (GPIO43 is UART0 TX after the
     * ROM boot log): GPIO function, output off, pull-up on. */
    gpio_reset_pin(w->pin_a);
    gpio_reset_pin(w->pin_b);

    pcnt_unit_config_t unit_cfg = {
        .low_limit  = ENC_PCNT_LOW_LIMIT,
        .high_limit = ENC_PCNT_HIGH_LIMIT,
        .flags.accum_count = 1,                 /* keep counting past the 16-bit hardware limits */
    };
    esp_err_t ret = pcnt_new_unit(&unit_cfg, &w->unit);
    if (ret != ESP_OK) return ret;

    pcnt_glitch_filter_config_t filter = { .max_glitch_ns = ENC_GLITCH_NS };
    ret = pcnt_unit_set_glitch_filter(w->unit, &filter);
    if (ret != ESP_OK) return ret;

    /* x4 quadrature: channel A counts edges of A gated by B, channel B the opposite */
    pcnt_chan_config_t chan_a_cfg = { .edge_gpio_num = w->pin_a, .level_gpio_num = w->pin_b };
    pcnt_chan_config_t chan_b_cfg = { .edge_gpio_num = w->pin_b, .level_gpio_num = w->pin_a };
    pcnt_channel_handle_t chan_a = NULL, chan_b = NULL;
    ret = pcnt_new_channel(w->unit, &chan_a_cfg, &chan_a);
    if (ret != ESP_OK) return ret;
    ret = pcnt_new_channel(w->unit, &chan_b_cfg, &chan_b);
    if (ret != ESP_OK) return ret;

    pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    /* The encoders have their own pull-ups; the internal ones are a backup */
    gpio_pullup_en(w->pin_a);
    gpio_pullup_en(w->pin_b);

    /* accum_count needs the limits as watch points */
    ret = pcnt_unit_add_watch_point(w->unit, ENC_PCNT_HIGH_LIMIT);
    if (ret != ESP_OK) return ret;
    ret = pcnt_unit_add_watch_point(w->unit, ENC_PCNT_LOW_LIMIT);
    if (ret != ESP_OK) return ret;

    ret = pcnt_unit_enable(w->unit);
    if (ret != ESP_OK) return ret;
    ret = pcnt_unit_clear_count(w->unit);
    if (ret != ESP_OK) return ret;
    ret = pcnt_unit_start(w->unit);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "%s encoder: A=GPIO%d B=GPIO%d%s", w->name, w->pin_a, w->pin_b, w->invert ? " (inverted)" : "");
    return ESP_OK;
}

/* =========================================================================
 * Diagnostic: one spare PCNT unit per pin, counting every edge (ENC_DIAG)
 * ========================================================================= */
#if ENC_DIAG
static pcnt_unit_handle_t s_diag_unit[2];
static const gpio_num_t   s_diag_pin[2] = { ENC_DIAG_PIN_1, ENC_DIAG_PIN_2 };

static esp_err_t init_diag_unit(gpio_num_t pin, pcnt_unit_handle_t *unit)
{
    pcnt_unit_config_t cfg = {
        .low_limit  = ENC_PCNT_LOW_LIMIT,
        .high_limit = ENC_PCNT_HIGH_LIMIT,
        .flags.accum_count = 1,
    };
    esp_err_t ret = pcnt_new_unit(&cfg, unit);
    if (ret != ESP_OK) return ret;
    pcnt_glitch_filter_config_t filter = { .max_glitch_ns = ENC_GLITCH_NS };
    pcnt_unit_set_glitch_filter(*unit, &filter);

    /* the same pad also feeds the quadrature unit, through the GPIO matrix */
    pcnt_chan_config_t ch_cfg = { .edge_gpio_num = pin, .level_gpio_num = -1 };
    pcnt_channel_handle_t ch = NULL;
    ret = pcnt_new_channel(*unit, &ch_cfg, &ch);
    if (ret != ESP_OK) return ret;
    pcnt_channel_set_edge_action(ch, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(ch, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP);
    gpio_pullup_en(pin);

    pcnt_unit_add_watch_point(*unit, ENC_PCNT_HIGH_LIMIT);
    pcnt_unit_add_watch_point(*unit, ENC_PCNT_LOW_LIMIT);
    pcnt_unit_enable(*unit);
    pcnt_unit_clear_count(*unit);
    return pcnt_unit_start(*unit);
}
#endif

void encoders_diag_log(void)
{
#if ENC_DIAG
    int e1 = -1, e2 = -1;
    if (s_diag_unit[0]) pcnt_unit_get_count(s_diag_unit[0], &e1);
    if (s_diag_unit[1]) pcnt_unit_get_count(s_diag_unit[1], &e2);
    ESP_LOGI(TAG, "DIAG edges: GPIO%d=%d  GPIO%d=%d | levels: L A(GPIO%d)=%d B(GPIO%d)=%d  R A(GPIO%d)=%d B(GPIO%d)=%d",
             s_diag_pin[0], e1, s_diag_pin[1], e2,
             ENC_LEFT_PIN_A, gpio_get_level(ENC_LEFT_PIN_A), ENC_LEFT_PIN_B, gpio_get_level(ENC_LEFT_PIN_B),
             ENC_RIGHT_PIN_A, gpio_get_level(ENC_RIGHT_PIN_A), ENC_RIGHT_PIN_B, gpio_get_level(ENC_RIGHT_PIN_B));
#endif
}

esp_err_t encoders_init(void)
{
    memset(&s_state, 0, sizeof(s_state));

    esp_err_t ret = init_wheel(&s_left);
    if (ret == ESP_OK) {
        ret = init_wheel(&s_right);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCNT init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "%d counts per wheel revolution (%d PPR x4 x 1:%.0f), wheel %.0f mm, speed window %d ms",
             (int)ENC_COUNTS_PER_WHEEL_REV, ENC_MOTOR_PPR, (double)ENC_GEAR_RATIO,
             (double)(WHEEL_DIAMETER_M * 1000.0f), ENC_SPEED_WINDOW_SAMPLES * TASK_MOTION_PERIOD_MS);

#if ENC_DIAG
    for (int i = 0; i < 2; i++) {
        if (init_diag_unit(s_diag_pin[i], &s_diag_unit[i]) != ESP_OK) {
            ESP_LOGW(TAG, "DIAG: no spare PCNT unit for GPIO%d", s_diag_pin[i]);
            s_diag_unit[i] = NULL;
        }
    }
    ESP_LOGW(TAG, "DIAG on: counting every edge of GPIO%d and GPIO%d separately (ENC_DIAG in app_config.h)",
             s_diag_pin[0], s_diag_pin[1]);
#endif

    taskENTER_CRITICAL(&s_lock);
    s_state.ok = true;
    taskEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

/* =========================================================================
 * Sampling
 * ========================================================================= */
static int32_t read_count(const wheel_enc_t *w)
{
    int raw = 0;
    pcnt_unit_get_count(w->unit, &raw);
    return w->invert ? -(int32_t)raw : (int32_t)raw;
}

void encoders_update(void)
{
    if (!s_state.ok) return;

    const int64_t now = esp_timer_get_time();
    const int32_t cl = read_count(&s_left);
    const int32_t cr = read_count(&s_right);

    s_hist_idx = (s_hist_idx + 1) % HIST_LEN;
    s_left.hist[s_hist_idx]  = cl;
    s_right.hist[s_hist_idx] = cr;
    s_hist_us[s_hist_idx]    = now;
    if (s_hist_fill < HIST_LEN) s_hist_fill++;

    /* speed over the oldest sample still in the window */
    float cps_l = 0.0f, cps_r = 0.0f;
    if (s_hist_fill >= 2) {
        /* the first sample went to index 1; once full, the oldest is the next slot */
        const int oldest = (s_hist_fill < HIST_LEN) ? 1 : (s_hist_idx + 1) % HIST_LEN;
        const float dt = (float)(now - s_hist_us[oldest]) / 1e6f;
        if (dt > 0.0f) {
            cps_l = (float)(cl - s_left.hist[oldest])  / dt;
            cps_r = (float)(cr - s_right.hist[oldest]) / dt;
        }
    }

    const float rev_per_count = 1.0f / (float)ENC_COUNTS_PER_WHEEL_REV;
    const float m_per_rev     = PI_F * WHEEL_DIAMETER_M;

    taskENTER_CRITICAL(&s_lock);
    s_state.count_left  = cl;
    s_state.count_right = cr;
    s_state.rpm_left    = cps_l * rev_per_count * 60.0f;
    s_state.rpm_right   = cps_r * rev_per_count * 60.0f;
    s_state.mps_left    = cps_l * rev_per_count * m_per_rev;
    s_state.mps_right   = cps_r * rev_per_count * m_per_rev;
    s_state.dist_m      = 0.5f * (float)(cl + cr) * rev_per_count * m_per_rev;
    taskEXIT_CRITICAL(&s_lock);
}

void encoders_get(encoders_state_t *out)
{
    taskENTER_CRITICAL(&s_lock);
    *out = s_state;
    taskEXIT_CRITICAL(&s_lock);
}
