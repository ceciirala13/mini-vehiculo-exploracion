/**
 * @file servo.c
 * @brief LEDC PWM controller implementation for servo motor.
 */

#include "servo.h"
#include "project_config.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "SERVO";
static uint16_t s_current_angle = 90;

esp_err_t servo_init(void)
{
    ESP_LOGI(TAG, "Initializing LEDC PWM for Servo on GPIO %d (50 Hz)...", SERVO_PIN_PWM);

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
        .duty           = SERVO_DUTY_90_DEG,
        .hpoint         = 0,
    };
    ret = ledc_channel_config(&channel_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_current_angle = 90;
    ESP_LOGI(TAG, "Servo initialized to default position 90°");
    return ESP_OK;
}

esp_err_t servo_set_angle(servo_angle_t angle)
{
    uint32_t duty = SERVO_DUTY_90_DEG;

    switch (angle) {
        case SERVO_POS_0_DEG:
            duty = SERVO_DUTY_0_DEG;
            s_current_angle = 0;
            break;
        case SERVO_POS_90_DEG:
            duty = SERVO_DUTY_90_DEG;
            s_current_angle = 90;
            break;
        case SERVO_POS_180_DEG:
            duty = SERVO_DUTY_180_DEG;
            s_current_angle = 180;
            break;
        default:
            ESP_LOGW(TAG, "Invalid angle requested: %d. Defaulting to 90°", angle);
            duty = SERVO_DUTY_90_DEG;
            s_current_angle = 90;
            break;
    }

    esp_err_t ret = ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_set_duty failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_update_duty failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Servo position updated to %d° (Duty: %lu)", s_current_angle, duty);
    return ESP_OK;
}

uint16_t servo_get_current_angle(void)
{
    return s_current_angle;
}
