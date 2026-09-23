#include "servo.h"
#include "esp_log.h"
#include <algorithm>
#include <cmath>

static const char *TAG = "SERVO";

static servo_config_t g_servo_cfg;
static bool g_initialized = false;
static float g_current_angle = 0.0f;

esp_err_t servo_init(const servo_config_t *config) {
    if (config == NULL || config->gpio_num < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    g_servo_cfg = *config;
    if (g_servo_cfg.min_pulse_us == 0) g_servo_cfg.min_pulse_us = 500;   // 0.5ms por defecto
    if (g_servo_cfg.max_pulse_us == 0) g_servo_cfg.max_pulse_us = 2500;  // 2.5ms por defecto

    // Configurar Timer LEDC para 50 Hz y resolución de 13 bits (8191 ticks por período)
    ledc_timer_config_t timer_conf = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .timer_num        = g_servo_cfg.timer,
        .freq_hz          = 50, // 50 Hz servomotor Estándar (período de 20 ms)
        .clk_cfg          = LEDC_AUTO_CLK
    };
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando timer LEDC: %s", esp_err_to_name(err));
        return err;
    }

    // Configurar Canal LEDC
    ledc_channel_config_t channel_conf = {
        .gpio_num       = g_servo_cfg.gpio_num,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = g_servo_cfg.channel,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = g_servo_cfg.timer,
        .duty           = 0,
        .hpoint         = 0,
        .sleep_mode     = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD
    };
    err = ledc_channel_config(&channel_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando canal LEDC: %s", esp_err_to_name(err));
        return err;
    }

    g_initialized = true;
    ESP_LOGI(TAG, "Servomotor inicializado en GPIO %d (Rango: %.1f° a %.1f°)", 
             g_servo_cfg.gpio_num, g_servo_cfg.min_angle, g_servo_cfg.max_angle);

    // Posición inicial neutra (0° / centro)
    return servo_set_angle(0.0f);
}

esp_err_t servo_set_angle(float angle) {
    if (!g_initialized) {
        ESP_LOGE(TAG, "Servomotor no inicializado.");
        return ESP_ERR_INVALID_STATE;
    }

    // Clampar el ángulo al rango configurado
    float clamped_angle = std::max(g_servo_cfg.min_angle, std::min(angle, g_servo_cfg.max_angle));

    // Mapear ángulo a microsegundos del pulso PWM
    float angle_ratio = (clamped_angle - g_servo_cfg.min_angle) / (g_servo_cfg.max_angle - g_servo_cfg.min_angle);
    uint32_t pulse_us = g_servo_cfg.min_pulse_us + static_cast<uint32_t>(angle_ratio * (g_servo_cfg.max_pulse_us - g_servo_cfg.min_pulse_us));

    // Calcular el duty cycle para 13 bits (8191 max count en 20000 us period)
    uint32_t duty = (pulse_us * 8191) / 20000;

    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, g_servo_cfg.channel, duty);
    if (err == ESP_OK) {
        err = ledc_update_duty(LEDC_LOW_SPEED_MODE, g_servo_cfg.channel);
    }

    if (err == ESP_OK) {
        g_current_angle = clamped_angle;
    } else {
        ESP_LOGE(TAG, "Error al actualizar duty del servo: %s", esp_err_to_name(err));
    }

    return err;
}

float servo_get_angle(void) {
    return g_current_angle;
}
