#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "manejo.h"
#include <stdio.h>

// Pines Motor 1
#define M1_IN1 7
#define M1_IN2 12
#define M1_NFAULT 21 // Changed from 23 (GPIO 23 does not exist on ESP32-S3)
// falta añadir disable 1 y 2 y sus funcionalidades

// Pines Motor 2
#define M2_IN1 14 // Changed from 22 (GPIO 22 does not exist on ESP32-S3)
#define M2_IN2 8
#define M2_NFAULT                                                              \
  38 // Changed from 31 (GPIO 26-32 are reserved for SPI Flash/PSRAM)

// Definición de velocidades asignadas a las acciones
#define VELOCIDAD_AVANCE 40.0f   // % hacia adelante
#define VELOCIDAD_REVERSA -40.0f // % en reversa
#define VELOCIDAD_PARADO 0.0f

extern "C" void app_main(void) {
  manejo_config_t traccion_cfg = {
      .IN1_M1 = M1_IN1,
      .IN2_M1 = M1_IN2,
      .nFAULT_M1 = M1_NFAULT,

      .IN1_M2 = M2_IN1,
      .IN2_M2 = M2_IN2,
      .nFAULT_M2 = M2_NFAULT,

      .pwm_freq = 20000 // 20kHz
  };

  manejo_handle_t traccion;
  float velocidad_actual = VELOCIDAD_PARADO;

  if (manejo_init(&traccion_cfg, &traccion) == ESP_OK) {
    ESP_LOGI("MAIN", "Sistema de tracción listo.");
  }

  while (1) {
    // Verificar seguridad: ¿alguno de los dos drivers falló?
    if (manejo_check_fault(&traccion)) {
      // Freno de emergencia inmediato
      manejo_set_speed(&traccion, VELOCIDAD_PARADO);

      ESP_LOGE("SEGURIDAD",
               "¡Fallo detectado en uno de los drivers! Tracción detenida.");

      // Bloquear ejecución temporalmente o esperar solución por hardware
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    // --- Aquí va tu lógica normal de control ---
    // 2. LEER EL TECLADO (A través de la entrada estándar UART sin bloquear el
    // bucle)
    int c = fgetc(stdin); // fgetc devolverá EOF (-1) si no hay ninguna tecla
                          // presionada en ese instante

    if (c != EOF) {
      // Se detectó una pulsación de tecla
      switch (c) {
      case 'w': // avance
      case 'W':
        if (velocidad_actual != VELOCIDAD_AVANCE) {
          velocidad_actual = VELOCIDAD_AVANCE;
          // ESP_LOGI("CONTROL", "Comando recibido: AVANZAR (40%)");
        }
        break;

      case 's':
      case 'S':
        if (velocidad_actual != VELOCIDAD_REVERSA) {
          velocidad_actual = VELOCIDAD_REVERSA;
          // ESP_LOGI("CONTROL", "Comando recibido: RETROCEDER (-40%)");
        }
        break;

      default:
        // Cualquier otra tecla actúa como freno general
        if (velocidad_actual != VELOCIDAD_PARADO) {
          velocidad_actual = VELOCIDAD_PARADO;
          // ESP_LOGW("CONTROL", "Comando recibido: DETENER");
        }
        break;
      }
    }

    // 3. ACTUALIZAR LA VELOCIDAD REAL DE LOS MOTORES
    manejo_set_speed(&traccion, velocidad_actual);

    vTaskDelay(pdMS_TO_TICKS(50)); // Muestreo del bucle de control
  }
}