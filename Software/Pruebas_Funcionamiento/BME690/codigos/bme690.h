/**
 * @file bme690.h
 * @brief HAL wrapper del BME690 (bme69x API) para ESP-IDF
 *
 * Sensor: BME690 (Bosch Sensortec)
 * API base: https://github.com/boschsensortec/BME690_SensorAPI
 * Interfaz: I2C
 * Target: ESP32-S3
 *   SDA -> GPIO05
 *   SCL -> GPIO06
 *
 * Uso:
 *   1. Llamar bme690_init() una vez al arrancar.
 *   2. Llamar bme690_read_data() en el loop para obtener mediciones.
 *   3. Llamar bme690_deinit() al terminar (opcional).
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "esp_err.h"
#include "bme69x.h"          /* Bosch SensorAPI principal */
#include "bme69x_defs.h"     /* Definiciones y constantes  */

/* ─── Configuración I2C ─────────────────────────────────────────────────── */

/** Puerto I2C a utilizar */
#define BME690_I2C_PORT         I2C_NUM_0

/** GPIO SDA del ESP32-S3 */
#define BME690_PIN_SDA          GPIO_NUM_5

/** GPIO SCL del ESP32-S3 */
#define BME690_PIN_SCL          GPIO_NUM_6

/** Frecuencia del bus I2C (400 kHz) */
#define BME690_I2C_FREQ_HZ      400000

/**
 * Dirección I2C del BME690.
 * SDO conectado a GND  → 0x76 (BME69X_I2C_ADDR_LOW)
 * SDO conectado a VDD  → 0x77 (BME69X_I2C_ADDR_HIGH)
 */
#define BME690_I2C_ADDR         BME69X_I2C_ADDR_LOW

/* ─── Configuración del sensor ──────────────────────────────────────────── */

/** Temperatura del calentador de gas (°C) */
#define BME690_GAS_HEATER_TEMP  320

/** Duración del calentador de gas (ms) */
#define BME690_GAS_HEATER_DUR   150

/** Temperatura ambiente inicial (°C), se actualiza con cada lectura */
#define BME690_AMB_TEMP_INIT    25

/* ─── Estructura de datos de medición ───────────────────────────────────── */

/**
 * @brief Resultado de una medición del BME690.
 */
typedef struct {
    float    temperature;   /**< Temperatura en °C                   */
    float    humidity;      /**< Humedad relativa en %               */
    float    pressure;      /**< Presión en hPa                      */
    uint32_t gas_resistance;/**< Resistencia de gas en Ohm           */
    uint8_t  gas_valid;     /**< 1 = medición de gas válida          */
    uint8_t  heater_stable; /**< 1 = calentador en temperatura meta  */
} bme690_data_t;

/* ─── API pública ───────────────────────────────────────────────────────── */

/**
 * @brief Inicializa el bus I2C y el sensor BME690.
 *
 * Configura el driver de Bosch (bme69x), el modo de operación forzado
 * y el perfil del calentador de gas.
 *
 * @return ESP_OK si todo fue correcto, error de ESP-IDF en caso contrario.
 */
esp_err_t bme690_init(void);

/**
 * @brief Lee temperatura, humedad, presión y resistencia de gas.
 *
 * Dispara una medición en modo forzado, espera el tiempo necesario y
 * rellena la estructura @p out con los datos compensados.
 *
 * @param[out] out  Puntero a la estructura donde se guardan los datos.
 * @return ESP_OK si la lectura fue exitosa.
 */
esp_err_t bme690_read_data(bme690_data_t *out);

/**
 * @brief Libera el bus I2C.
 *
 * @return ESP_OK siempre.
 */
esp_err_t bme690_deinit(void);

#ifdef __cplusplus
}
#endif
