/**
 * @file main.cpp
 * @brief Ejemplo principal para ESP32-S3 con el sensor BME690
 *
 * Estructura del proyecto ESP-IDF sugerida:
 *
 *   my_project/
 *   ├── CMakeLists.txt                  ← raíz del proyecto
 *   ├── main/
 *   │   ├── CMakeLists.txt
 *   │   ├── main.cpp                    ← este archivo
 *   │   ├── bme690.c
 *   │   └── bme690.h
 *   └── components/
 *       └── BME690_SensorAPI/           ← clonar el repo de Bosch aquí
 *           ├── bme69x.c
 *           ├── bme69x.h
 *           ├── bme69x_defs.h
 *           └── CMakeLists.txt          ← ver nota al final
 *
 * CMakeLists.txt del proyecto (raíz):
 *   cmake_minimum_required(VERSION 3.16)
 *   include($ENV{IDF_PATH}/tools/cmake/project.cmake)
 *   project(bme690_demo)
 *
 * main/CMakeLists.txt:
 *   idf_component_register(
 *       SRCS "main.cpp" "bme690.c"
 *       INCLUDE_DIRS "."
 *       REQUIRES driver freertos esp_log BME690_SensorAPI
 *   )
 *
 * components/BME690_SensorAPI/CMakeLists.txt:
 *   idf_component_register(
 *       SRCS "bme69x.c"
 *       INCLUDE_DIRS "."
 *   )
 *
 * Conexiones físicas ESP32-S3:
 *   BME690 SDA  →  GPIO05
 *   BME690 SCL  →  GPIO06
 *   BME690 VDD  →  3.3 V
 *   BME690 GND  →  GND
 *   BME690 SDO  →  GND  (dirección I2C = 0x76)
 *   BME690 CSB  →  VDD  (selecciona modo I2C)
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bme690.h"

static const char *TAG = "MAIN";

/* Intervalo entre lecturas (ms) */
#define READ_INTERVAL_MS    2000

/* ─── Tarea principal ───────────────────────────────────────────────────── */

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== Ejemplo BME690 en ESP32-S3 ===");
    ESP_LOGI(TAG, "SDA: GPIO%d | SCL: GPIO%d", BME690_PIN_SDA, BME690_PIN_SCL);

    /* Inicializar el sensor */
    esp_err_t err = bme690_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo inicializar el BME690 (err=%s). Verificar cableado.",
                 esp_err_to_name(err));
        /* En una aplicación real, aquí se podría reintentar o dormir el sistema */
        return;
    }

    bme690_data_t sensor_data;

    while (true) {
        err = bme690_read_data(&sensor_data);

        if (err == ESP_OK) {
            ESP_LOGI(TAG, "─────────────────────────────────");
            ESP_LOGI(TAG, "Temperatura  : %.2f °C",   sensor_data.temperature);
            ESP_LOGI(TAG, "Humedad      : %.2f %%",   sensor_data.humidity);
            ESP_LOGI(TAG, "Presión      : %.2f hPa",  sensor_data.pressure);

            if (sensor_data.gas_valid && sensor_data.heater_stable) {
                ESP_LOGI(TAG, "Gas (VOC)    : %lu Ohm", (unsigned long)sensor_data.gas_resistance);
            } else {
                ESP_LOGW(TAG, "Gas (VOC)    : medición no válida "
                              "(gas_valid=%d, heater_stable=%d)",
                              sensor_data.gas_valid, sensor_data.heater_stable);
            }
        } else {
            ESP_LOGE(TAG, "Error al leer datos: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(READ_INTERVAL_MS));
    }

    /* Nunca se llega aquí en condiciones normales */
    bme690_deinit();
}
