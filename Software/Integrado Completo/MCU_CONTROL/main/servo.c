/**
 * @file servo.c
 * @brief LEDC PWM driver for the steering servo (continuous angle).
 */

#include "servo.h"
#include "app_config.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "SERVO";

#define SERVO_PERIOD_US     (1000000 / SERVO_LEDC_FREQ_HZ)
#define SERVO_DUTY_MAX      ((1U << SERVO_LEDC_DUTY_RES) - 1)

static bool  s_initialized = false;
static float s_steer_deg   = 0.0f;

static float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

esp_err_t servo_init(void)
{
    ESP_LOGI(TAG, "Initializing LEDC PWM for servo on GPIO %d (%d Hz)...", SERVO_PIN_PWM, SERVO_LEDC_FREQ_HZ);

    ledc_timer_config_t timer_cfg = {
        .speed_mode       = SERVO_LEDC_MODE,
        .duty_resolution  = SERVO_LEDC_DUTY_RES,
        .timer_num        = SERVO_LEDC_TIMER,
        .freq_hz          = SERVO_LEDC_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ledc_channel_config_t channel_cfg = {
        .speed_mode     = SERVO_LEDC_MODE,
        .channel        = SERVO_LEDC_CHANNEL,
        .timer_sel      = SERVO_LEDC_TIMER,
        .gpio_num       = SERVO_PIN_PWM,
        .duty           = 0,
        .hpoint         = 0,
    };
    ret = ledc_channel_config(&channel_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_initialized = true;
    ret = steering_set_deg(0.0f);
    ESP_LOGI(TAG, "Servo initialized, steering centered (servo %.1f deg)", SERVO_CENTER_DEG);
    return ret;
}

esp_err_t servo_set_angle_deg(float angle_deg)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    angle_deg = clampf(angle_deg, 0.0f, 180.0f);

    float pulse_us = SERVO_MIN_PULSE_US +
                     (angle_deg / 180.0f) * (float)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
    uint32_t duty = (uint32_t)((pulse_us / (float)SERVO_PERIOD_US) * (float)SERVO_DUTY_MAX);

    esp_err_t ret = ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty);
    if (ret == ESP_OK) {
        ret = ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update servo duty: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t steering_set_deg(float steer_deg)
{
    steer_deg = clampf(steer_deg, -SERVO_MAX_STEER_DEG, SERVO_MAX_STEER_DEG);

#if SERVO_INVERT
    float servo_deg = SERVO_CENTER_DEG - steer_deg;
#else
    float servo_deg = SERVO_CENTER_DEG + steer_deg;
#endif

    esp_err_t ret = servo_set_angle_deg(servo_deg);
    if (ret == ESP_OK) {
        s_steer_deg = steer_deg;
    }
    return ret;
}

float steering_get_deg(void)
{
    return s_steer_deg;
}
