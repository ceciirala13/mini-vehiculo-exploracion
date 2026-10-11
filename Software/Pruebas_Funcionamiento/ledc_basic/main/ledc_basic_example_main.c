/* LEDC Servo Test Example */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "sdkconfig.h"
#include "esp_pm.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (39)                 // GPIO 2 (built-in LED pin or connected to Servo PWM signal)
#define LEDC_CHANNEL           LEDC_CHANNEL_0
#define LEDC_DUTY_RES          LEDC_TIMER_13_BIT   // 13-bit resolution (0 to 8191)

// SERVOS REQUIRE 50 Hz (20ms total period)
#define LEDC_FREQUENCY          (50)                

// Duty cycle values for 13-bit resolution at 50 Hz:
// 20ms period = 8191 counts
// 0.5ms pulse (~0 deg)   -> (0.5 / 20.0) * 8191 ≈ 205
// 1.5ms pulse (~90 deg)  -> (1.5 / 20.0) * 8191 ≈ 614
// 2.5ms pulse (~180 deg) -> (2.5 / 20.0) * 8191 ≈ 1024
#define SERVO_DUTY_0_DEG        (205)
#define SERVO_DUTY_90_DEG       (614)
#define SERVO_DUTY_180_DEG      (1024)

#if CONFIG_PM_ENABLE
#define LEDC_CLK_SRC            LEDC_USE_RC_FAST_CLK
#else
#define LEDC_CLK_SRC            LEDC_AUTO_CLK
#endif

static void example_ledc_init(void)
{
    // Prepare and apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set to 50 Hz for Servo
        .clk_cfg          = LEDC_CLK_SRC,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0,                // Start at 0
        .hpoint         = 0,
#if CONFIG_PM_ENABLE
        .sleep_mode     = LEDC_SLEEP_MODE_KEEP_ALIVE,
#endif
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void app_main(void)
{
#if CONFIG_PM_ENABLE
    esp_pm_config_t pm_config = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = true
#endif
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
#endif

    // Initialize LEDC configuration
    example_ledc_init();

    // Loop to sweep the servo arm back and forth to test motion
    while (1) {
        printf("Moving to 0 degrees\n");
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, SERVO_DUTY_0_DEG));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        vTaskDelay(pdMS_TO_TICKS(1500));

        printf("Moving to 90 degrees\n");
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, SERVO_DUTY_90_DEG));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        vTaskDelay(pdMS_TO_TICKS(1500));

        printf("Moving to 180 degrees\n");
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, SERVO_DUTY_180_DEG));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}