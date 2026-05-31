/**
 * @file bme690.c
 * @brief Implementación del HAL del BME690 para ESP-IDF (I2C)
 *
 * Basado en el ejemplo "common" de:
 *   https://github.com/boschsensortec/BME690_SensorAPI/tree/master/examples/common
 *
 * Flujo de inicialización (igual que common.c de Bosch):
 *   1. Configurar punteros de lectura/escritura/delay en bme69x_dev.
 *   2. bme69x_init()  → lee chip-id y carga coeficientes de calibración.
 *   3. Configurar oversampling y filtro IIR.
 *   4. Configurar perfil del calentador de gas.
 *
 * Flujo de lectura (modo forzado):
 *   1. bme69x_set_op_mode(FORCED)  → dispara una única medición.
 *   2. Calcular tiempo de medición con bme69x_get_meas_dur().
 *   3. Esperar el tiempo calculado.
 *   4. bme69x_get_data() → obtiene y compensa los datos crudos.
 */

#include "bme690.hpp"

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>


static const char *TAG = "BME690";

/* ─── Instancia global del driver Bosch ─────────────────────────────────── */
static struct bme69x_dev s_dev;
static struct bme69x_conf s_conf;
static struct bme69x_heatr_conf s_heatr_conf;
static bool s_initialized = false;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Callbacks requeridos por la API de Bosch
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * @brief Callback de lectura I2C para la API de Bosch.
 *
 * La firma debe coincidir con bme69x_read_fptr_t:
 *   int8_t read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void
 * *intf_ptr)
 */
static BME69X_INTF_RET_TYPE bme690_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                                            uint32_t len, void *intf_ptr) {
  uint8_t dev_addr = *(uint8_t *)intf_ptr;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  /* Escritura del registro a leer */
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg_addr, true);
  /* Lectura de los datos */
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
  if (len > 1) {
    i2c_master_read(cmd, reg_data, len - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, reg_data + len - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);

  esp_err_t ret =
      i2c_master_cmd_begin(BME690_I2C_PORT, cmd, pdMS_TO_TICKS(100));
  i2c_cmd_link_delete(cmd);

  return (ret == ESP_OK) ? BME69X_OK : BME69X_E_COM_FAIL;
}

/**
 * @brief Callback de escritura I2C para la API de Bosch.
 *
 * La firma debe coincidir con bme69x_write_fptr_t:
 *   int8_t write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void
 * *intf_ptr)
 */
static BME69X_INTF_RET_TYPE bme690_i2c_write(uint8_t reg_addr,
                                             const uint8_t *reg_data,
                                             uint32_t len, void *intf_ptr) {
  uint8_t dev_addr = *(uint8_t *)intf_ptr;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg_addr, true);
  i2c_master_write(cmd, reg_data, len, true);
  i2c_master_stop(cmd);

  esp_err_t ret =
      i2c_master_cmd_begin(BME690_I2C_PORT, cmd, pdMS_TO_TICKS(100));
  i2c_cmd_link_delete(cmd);

  return (ret == ESP_OK) ? BME69X_OK : BME69X_E_COM_FAIL;
}

/**
 * @brief Callback de delay en microsegundos para la API de Bosch.
 *
 * La firma debe coincidir con bme69x_delay_us_fptr_t:
 *   void delay_us(uint32_t period, void *intf_ptr)
 */
static void bme690_delay_us(uint32_t period, void *intf_ptr) {
  (void)intf_ptr;
  /* FreeRTOS no resuelve microsegundos directamente; usamos vTaskDelay
   * redondeando hacia arriba al siguiente tick (1 ms como mínimo).
   * Para delays muy cortos (< 1 ms) esto introduce latencia extra,
   * pero es aceptable en modo forzado donde los waits son de varios ms. */
  uint32_t ms = (period + 999) / 1000;
  if (ms == 0)
    ms = 1;
  vTaskDelay(pdMS_TO_TICKS(ms));
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Inicialización del bus I2C
 * ═══════════════════════════════════════════════════════════════════════════
 */

static esp_err_t i2c_master_init(void) {
  i2c_config_t conf;
  memset(&conf, 0, sizeof(conf));
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = BME690_PIN_SDA;
  conf.scl_io_num = BME690_PIN_SCL;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = BME690_I2C_FREQ_HZ;

  esp_err_t ret = i2c_param_config(BME690_I2C_PORT, &conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "i2c_param_config falló: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = i2c_driver_install(BME690_I2C_PORT, I2C_MODE_MASTER,
                           0,  /* rx buffer (solo modo esclavo) */
                           0,  /* tx buffer (solo modo esclavo) */
                           0); /* flags                          */
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "i2c_driver_install falló: %s", esp_err_to_name(ret));
  }
  return ret;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  API pública
 * ═══════════════════════════════════════════════════════════════════════════
 */

esp_err_t bme690_init(void) {
  int8_t rslt;

  /* 1. Inicializar bus I2C */
  esp_err_t err = i2c_master_init();
  if (err != ESP_OK)
    return err;

  /* 2. Rellenar estructura de dispositivo Bosch */
  static uint8_t dev_addr = BME690_I2C_ADDR;

  s_dev.read = bme690_i2c_read;
  s_dev.write = bme690_i2c_write;
  s_dev.delay_us = bme690_delay_us;
  s_dev.intf = BME69X_I2C_INTF;
  s_dev.intf_ptr = &dev_addr;
  s_dev.amb_temp = BME690_AMB_TEMP_INIT;

  /* 3. Inicializar sensor (lee chip-id y calibración) */
  rslt = bme69x_init(&s_dev);
  if (rslt != BME69X_OK) {
    ESP_LOGE(TAG, "bme69x_init falló: %d (¿dirección I2C correcta?)", rslt);
    i2c_driver_delete(BME690_I2C_PORT);
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "BME690 detectado correctamente");

  /* 4. Configurar oversampling y filtro IIR
   *    Oversampling x2 para temperatura y presión, x1 para humedad.
   *    Filtro IIR coeficiente 3 para estabilizar lecturas de presión. */
  s_conf.filter = BME69X_FILTER_SIZE_3;
  s_conf.odr = BME69X_ODR_NONE; /* Sin ODR; usamos modo forzado */
  s_conf.os_hum = BME69X_OS_1X;
  s_conf.os_pres = BME69X_OS_2X;
  s_conf.os_temp = BME69X_OS_2X;

  rslt = bme69x_set_conf(&s_conf, &s_dev);
  if (rslt != BME69X_OK) {
    ESP_LOGE(TAG, "bme69x_set_conf falló: %d", rslt);
    i2c_driver_delete(BME690_I2C_PORT);
    return ESP_FAIL;
  }

  /* 5. Configurar calentador de gas (perfil único para modo forzado) */
  s_heatr_conf.enable = BME69X_ENABLE;
  s_heatr_conf.heatr_temp = BME690_GAS_HEATER_TEMP; /* °C  */
  s_heatr_conf.heatr_dur = BME690_GAS_HEATER_DUR;   /* ms  */

  rslt = bme69x_set_heatr_conf(BME69X_FORCED_MODE, &s_heatr_conf, &s_dev);
  if (rslt != BME69X_OK) {
    ESP_LOGE(TAG, "bme69x_set_heatr_conf falló: %d", rslt);
    i2c_driver_delete(BME690_I2C_PORT);
    return ESP_FAIL;
  }

  s_initialized = true;
  ESP_LOGI(TAG, "BME690 inicializado. SDA=GPIO%d, SCL=GPIO%d, addr=0x%02X",
           BME690_PIN_SDA, BME690_PIN_SCL, BME690_I2C_ADDR);
  return ESP_OK;
}

esp_err_t bme690_read_data(bme690_data_t *out) {
  if (!s_initialized) {
    ESP_LOGE(TAG, "Sensor no inicializado. Llame primero a bme690_init().");
    return ESP_ERR_INVALID_STATE;
  }
  if (!out) {
    return ESP_ERR_INVALID_ARG;
  }

  int8_t rslt;
  struct bme69x_data data;
  uint8_t n_fields = 0;
  uint32_t delay_us_val;

  /* Modo forzado: dispara una sola medición */
  rslt = bme69x_set_op_mode(BME69X_FORCED_MODE, &s_dev);
  if (rslt != BME69X_OK) {
    ESP_LOGE(TAG, "set_op_mode falló: %d", rslt);
    return ESP_FAIL;
  }

  /* Calcular tiempo de medición y esperar */
  delay_us_val = bme69x_get_meas_dur(BME69X_FORCED_MODE, &s_conf, &s_dev) +
                 (s_heatr_conf.heatr_dur * 1000); /* heatr_dur en ms → us */
  s_dev.delay_us(delay_us_val, s_dev.intf_ptr);

  /* Leer datos compensados */
  rslt = bme69x_get_data(BME69X_FORCED_MODE, &data, &n_fields, &s_dev);
  if (rslt != BME69X_OK || n_fields == 0) {
    ESP_LOGW(TAG, "bme69x_get_data: rslt=%d, n_fields=%d", rslt, n_fields);
    return ESP_FAIL;
  }

  /* Actualizar temperatura ambiente para compensación interna */
  s_dev.amb_temp = (int8_t)data.temperature;

  /* Copiar resultados */
  out->temperature = data.temperature;
  out->humidity = data.humidity;
  out->pressure = data.pressure / 100.0f; /* Pa → hPa */
  out->gas_resistance = data.gas_resistance;
  out->gas_valid = (data.status & BME69X_GASM_VALID_MSK) ? 1u : 0u;
  out->heater_stable = (data.status & BME69X_HEAT_STAB_MSK) ? 1u : 0u;

  return ESP_OK;
}

esp_err_t bme690_deinit(void) {
  if (s_initialized) {
    i2c_driver_delete(BME690_I2C_PORT);
    s_initialized = false;
    ESP_LOGI(TAG, "BME690 desinicializado.");
  }
  return ESP_OK;
}
