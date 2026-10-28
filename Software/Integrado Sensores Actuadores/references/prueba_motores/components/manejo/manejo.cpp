#include "manejo.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "MANEJO";

esp_err_t manejo_init(const manejo_config_t *config, manejo_handle_t *handle) {
  if (config == NULL || handle == NULL)
    return ESP_ERR_INVALID_ARG;

  handle->pin_nfault_m1 = (gpio_num_t)config->nFAULT_M1;
  handle->pin_nfault_m2 = (gpio_num_t)config->nFAULT_M2;

  // 1. Configurar Pines nFAULT (Entradas con pull-up externo ya existente)
  gpio_config_t io_conf;
  memset(&io_conf, 0, sizeof(io_conf));
  io_conf.pin_bit_mask = (1ULL << handle->pin_nfault_m1) | (1ULL << handle->pin_nfault_m2);
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&io_conf));

  // 2. Configurar un único Timer MCPWM para ambos motores
  handle->period_ticks = 1000000 / config->pwm_freq;
  mcpwm_timer_config_t timer_config;
  memset(&timer_config, 0, sizeof(timer_config));
  timer_config.group_id = 0;
  timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  timer_config.resolution_hz = 1000000; // 1MHz, es la resolucion de reloj interno
  timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
  timer_config.period_ticks = handle->period_ticks; // pwm_freq es la frecuencia del pwm al motor
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &handle->timer));

  // 3. Configurar dos Operadores MCPWM (Uno para cada motor)
  mcpwm_operator_config_t operator_config;
  memset(&operator_config, 0, sizeof(operator_config));
  operator_config.group_id = 0;
  ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &handle->oper_m1));
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(handle->oper_m1, handle->timer));

  ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &handle->oper_m2));
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(handle->oper_m2, handle->timer));

  // 4. Configurar comparadores (Avance/Reversa para cada operador)
  mcpwm_comparator_config_t compare_config;
  memset(&compare_config, 0, sizeof(compare_config));
  compare_config.flags.update_cmp_on_tez = true;

  // Comparadores para Motor 1
  ESP_ERROR_CHECK(
      mcpwm_new_comparator(handle->oper_m1, &compare_config, &handle->cmpr_in1_m1));
  ESP_ERROR_CHECK(
      mcpwm_new_comparator(handle->oper_m1, &compare_config, &handle->cmpr_in2_m1));

  // Comparadores para Motor 2
  ESP_ERROR_CHECK(
      mcpwm_new_comparator(handle->oper_m2, &compare_config, &handle->cmpr_in1_m2));
  ESP_ERROR_CHECK(
      mcpwm_new_comparator(handle->oper_m2, &compare_config, &handle->cmpr_in2_m2));

  // 5. Crear los 4 Generadores asignados a sus respectivos pines
  mcpwm_generator_config_t gen_config;
  memset(&gen_config, 0, sizeof(gen_config));

  // Motor 1 (asociado a oper_m1)
  gen_config.gen_gpio_num = config->IN1_M1; // avance motor 1
  ESP_ERROR_CHECK(
      mcpwm_new_generator(handle->oper_m1, &gen_config, &handle->gen_in1_m1));
  gen_config.gen_gpio_num = config->IN2_M1; // retroceso motor 1
  ESP_ERROR_CHECK(
      mcpwm_new_generator(handle->oper_m1, &gen_config, &handle->gen_in2_m1));

  // Motor 2 (asociado a oper_m2)
  gen_config.gen_gpio_num = config->IN1_M2; // avance motor 2
  ESP_ERROR_CHECK(
      mcpwm_new_generator(handle->oper_m2, &gen_config, &handle->gen_in1_m2));
  gen_config.gen_gpio_num = config->IN2_M2; // retroceso motor 2
  ESP_ERROR_CHECK(
      mcpwm_new_generator(handle->oper_m2, &gen_config, &handle->gen_in2_m2));

  // Inicializar comparadores en 0
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(handle->cmpr_in1_m1, 0));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(handle->cmpr_in2_m1, 0));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(handle->cmpr_in1_m2, 0));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(handle->cmpr_in2_m2, 0));

  // 6. Vincular las acciones de los generadores

  // Configurar flancos altos al vaciarse el timer
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
      handle->gen_in1_m1, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                       MCPWM_TIMER_EVENT_EMPTY,
                                                       MCPWM_GEN_ACTION_HIGH)));
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
      handle->gen_in1_m2, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                       MCPWM_TIMER_EVENT_EMPTY,
                                                       MCPWM_GEN_ACTION_HIGH)));
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
      handle->gen_in2_m1, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                       MCPWM_TIMER_EVENT_EMPTY,
                                                       MCPWM_GEN_ACTION_HIGH)));
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
      handle->gen_in2_m2, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                       MCPWM_TIMER_EVENT_EMPTY,
                                                       MCPWM_GEN_ACTION_HIGH)));

  // Configurar flancos bajos al alcanzar el valor del comparador asignado
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
      handle->gen_in1_m1,
      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, handle->cmpr_in1_m1,
                                     MCPWM_GEN_ACTION_LOW)));
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
      handle->gen_in1_m2,
      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, handle->cmpr_in1_m2,
                                     MCPWM_GEN_ACTION_LOW)));

  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
      handle->gen_in2_m1,
      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, handle->cmpr_in2_m1,
                                     MCPWM_GEN_ACTION_LOW)));
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
      handle->gen_in2_m2,
      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, handle->cmpr_in2_m2,
                                     MCPWM_GEN_ACTION_LOW)));

  // 7. Arrancar el Timer
  ESP_ERROR_CHECK(mcpwm_timer_enable(handle->timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(handle->timer, MCPWM_TIMER_START_NO_STOP));

  ESP_LOGI(TAG, "Sistema de tracción dual inicializado con operadores independientes.");
  return ESP_OK;
}

esp_err_t manejo_set_speed(manejo_handle_t *handle, float speed) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;

  if (speed > 100.0f)
    speed = 100.0f;
  if (speed < -100.0f)
    speed = -100.0f;

  uint32_t compare_value = (uint32_t)((fabsf(speed) / 100.0f) * handle->period_ticks);

  if (speed > 0.0f) {
    // AMBOS MOTORES ADELANTE
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(handle->cmpr_in1_m1, compare_value));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(handle->cmpr_in2_m1, 0));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(handle->cmpr_in1_m2, compare_value));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(handle->cmpr_in2_m2, 0));
  } else if (speed < 0.0f) {
    // AMBOS MOTORES REVERSA
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(handle->cmpr_in1_m1, 0));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(handle->cmpr_in2_m1, compare_value));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(handle->cmpr_in1_m2, 0));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(handle->cmpr_in2_m2, compare_value));
  } else {
    // FRENADO TOTAL
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(handle->cmpr_in1_m1, 0));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(handle->cmpr_in2_m1, 0));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(handle->cmpr_in1_m2, 0));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(handle->cmpr_in2_m2, 0));
  }

  return ESP_OK;
}

bool manejo_check_fault(manejo_handle_t *handle) {
  if (handle == NULL)
    return false;

  // nFAULT es activo en bajo (0 significa error).
  // Si el motor 1 tiene un 0 O el motor 2 tiene un 0, disparamos la alerta de
  // fallo.
  return (gpio_get_level(handle->pin_nfault_m1) == 0 ||
          gpio_get_level(handle->pin_nfault_m2) == 0);
}