#ifndef manejo
#define manejo

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"


/**
 * @brief Configuración de pines para el driver DRV8873H
 */
typedef struct {
  int IN1_M1;        // Pin PWM 1 del MOTOR 1 / Dirección 1
  int IN2_M1;        // Pin PWM 2 del MOTOR 1 / Dirección 2
  int nFAULT_M1;     // Pin de fallo de MOTOR 1 (Entrada, activo en BAJO)
  int IN1_M2;        // Pin PWM 1 del MOTOR 2 / Dirección 1
  int IN2_M2;        // Pin PWM 2 del MOTOR 2 / Dirección 2
  int nFAULT_M2;     // Pin de fallo de MOTOR 2 (Entrada, activo en BAJO)
  uint32_t pwm_freq; // Frecuencia del PWM (Ej: 20000 para 20kHz)
} manejo_config_t;

/**
 * @brief Handle de control del motor
 */
typedef struct {
  mcpwm_timer_handle_t timer;
  mcpwm_oper_handle_t oper_m1;
  mcpwm_oper_handle_t oper_m2;
  // Comparadores independientes para cada operador/motor
  mcpwm_cmpr_handle_t cmpr_in1_m1;
  mcpwm_cmpr_handle_t cmpr_in2_m1;
  mcpwm_cmpr_handle_t cmpr_in1_m2;
  mcpwm_cmpr_handle_t cmpr_in2_m2;
  // Generadores independientes
  mcpwm_gen_handle_t gen_in1_m1;
  mcpwm_gen_handle_t gen_in2_m1;
  mcpwm_gen_handle_t gen_in1_m2;
  mcpwm_gen_handle_t gen_in2_m2;
  // Deteccion de fallas independiente
  gpio_num_t pin_nfault_m1;
  gpio_num_t pin_nfault_m2;
  uint32_t period_ticks;
} manejo_handle_t;

/**
 * @brief Inicializa los drivers DRV8873H con MCPWM y GPIO
 */
esp_err_t manejo_init(const manejo_config_t *config, manejo_handle_t *handle);

/**
 * @brief Controla los motores en base a la velocidad (-100.0 a 100.0)
 * Valores positivos: Giro hacia adelante
 * Valores negativos: Giro en reversa
 * 0: Freno (o rueda libre, según la lógica IN1/IN2)
 */
esp_err_t manejo_set_speed(manejo_handle_t *handle, float speed);

/**
 * @brief Lee el estado de los pines nFAULT
 * @return true si hay una falla (Pin en BAJO), false si opera normalmente
 */
bool manejo_check_fault(manejo_handle_t *handle);

#endif
