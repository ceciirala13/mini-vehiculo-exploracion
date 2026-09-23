#ifndef SERVO_H
#define SERVO_H

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int gpio_num;           // Pin asignado al servomotor
    float min_angle;        // Ángulo mínimo en grados (ej: -45.0f o 0.0f)
    float max_angle;        // Ángulo máximo en grados (ej: 45.0f o 180.0f)
    uint32_t min_pulse_us;  // Ancho de pulso mínimo en microsegundos (ej: 500 us = 0.5ms)
    uint32_t max_pulse_us;  // Ancho de pulso máximo en microsegundos (ej: 2500 us = 2.5ms)
    ledc_channel_t channel; // Canal LEDC (LEDC_CHANNEL_0 por defecto)
    ledc_timer_t timer;     // Timer LEDC (LEDC_TIMER_0 por defecto)
} servo_config_t;

/**
 * @brief Inicializa el servomotor usando el periférico LEDC de ESP-IDF
 */
esp_err_t servo_init(const servo_config_t *config);

/**
 * @brief Establece el ángulo del servomotor
 * @param angle Ángulo deseado (se limita automáticamente entre min_angle y max_angle)
 */
esp_err_t servo_set_angle(float angle);

/**
 * @brief Obtiene el ángulo actual configurado en el servomotor
 */
float servo_get_angle(void);

#ifdef __cplusplus
}
#endif

#endif // SERVO_H
