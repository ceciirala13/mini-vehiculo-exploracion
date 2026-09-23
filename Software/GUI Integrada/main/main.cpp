// ============================================================
//  ROVER EXPLORER - CONTROL DE VEHÍCULO VÍA PUENTE UART SERIAL
//  ESP32-S3 + ARDUCAM OV2640 + DRV8873-Q1 DUAL + SERVO + BME690
// ============================================================
#include <cstdio>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "manejo.h"
#include "servo.h"
#include "bme690.h"
#include "arducam_ov2640.h"
#include "uart_bridge.h"

static const char *TAG = "ROVER_MAIN";

// Configuración de Pines del ESP32-S3 usando constexpr
namespace Config {
    // UART Serial hacia ESP32-C6 (TX=IO43, RX=IO44)
    constexpr gpio_num_t UART_TX_PIN  = GPIO_NUM_43;
    constexpr gpio_num_t UART_RX_PIN  = GPIO_NUM_44;
    constexpr uint32_t   UART_BAUD    = 921600;

    // Motor 1 (DRV8873-Q1 #1)
    constexpr gpio_num_t M1_IN1       = GPIO_NUM_7;
    constexpr gpio_num_t M1_IN2       = GPIO_NUM_4;
    constexpr gpio_num_t M1_NFAULT    = GPIO_NUM_21;
    constexpr gpio_num_t M1_DISABLE   = GPIO_NUM_9;
    constexpr gpio_num_t M1_NSLEEP    = GPIO_NUM_NC; // Configurado por Hardware (-1 / NC)

    // Motor 2 (DRV8873-Q1 #2)
    constexpr gpio_num_t M2_IN1       = GPIO_NUM_14;
    constexpr gpio_num_t M2_IN2       = GPIO_NUM_8;
    constexpr gpio_num_t M2_NFAULT    = GPIO_NUM_38;
    constexpr gpio_num_t M2_DISABLE   = GPIO_NUM_16;
    constexpr gpio_num_t M2_NSLEEP    = GPIO_NUM_NC; // Configurado por Hardware (-1 / NC)

    // Servomotor para Dirección
    constexpr gpio_num_t SERVO_PIN    = GPIO_NUM_18;

    // Sensor Ambiental BME690 (I2C_NUM_0)
    constexpr gpio_num_t BME_SDA      = GPIO_NUM_5;  // IO5
    constexpr gpio_num_t BME_SCL      = GPIO_NUM_6;  // IO6

    // Cámara ArduCAM OV2640 (SPI2_HOST + I2C_NUM_1 dedicado)
    constexpr gpio_num_t CAM_CS       = GPIO_NUM_10; // IO10
    constexpr gpio_num_t CAM_MOSI     = GPIO_NUM_11; // IO11
    constexpr gpio_num_t CAM_SCK      = GPIO_NUM_12; // IO12
    constexpr gpio_num_t CAM_MISO     = GPIO_NUM_13; // IO13
    constexpr gpio_num_t CAM_SDA      = GPIO_NUM_47; // IO47 (I2C_NUM_1)
    constexpr gpio_num_t CAM_SCL      = GPIO_NUM_48; // IO48 (I2C_NUM_1)
}

// Variables Globales de Estado del Vehículo
static manejo_handle_t g_traccion;
static bool g_traccion_ok = false;
static bool g_servo_ok = false;
static bool g_bme_ok = false;
static bool g_camera_ok = false;
static bool g_fault_state = false;

// Estado Cámara Streaming
static volatile bool g_cam_streaming = false;
static volatile bool g_snapshot_requested = false;

// Task para la lectura continua del BME690 y transmisión de telemetría (1 Hz)
static void bme690_telemetry_task(void *pvParameters) {
    bme690_data_t bme_data = { .temperature = 25.0f, .humidity = 50.0f, .pressure = 1013.2f, .gas_resistance = 75000 };
    while (true) {
        if (g_bme_ok) {
            if (bme690_read_data(&bme_data) != ESP_OK) {
                ESP_LOGW("BME690_TASK", "Advertencia: Lectura BME690 no disponible en este ciclo.");
            }
        }

        // Simulación/Lectura de voltajes de batería (Potencia 3S y Lógica ESP)
        float v_motors = 11.8f;
        float pct_motors = 85.0f;
        float v_esp = 4.12f;
        float pct_esp = 92.0f;

        uart_bridge_send_telemetry(bme_data.temperature,
                                   bme_data.humidity,
                                   bme_data.pressure,
                                   bme_data.gas_resistance,
                                   v_motors, pct_motors,
                                   v_esp, pct_esp,
                                   g_fault_state);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Task para la captura y transmisión de imágenes de la cámara ArduCAM OV2640
static void camera_stream_task(void *pvParameters) {
    while (true) {
        if (g_camera_ok && (g_cam_streaming || g_snapshot_requested)) {
            g_snapshot_requested = false;

            uint8_t *jpeg_buf = NULL;
            size_t jpeg_len = 0;

            if (arducam_capture_frame(&jpeg_buf, &jpeg_len) == ESP_OK && jpeg_buf != NULL && jpeg_len > 0) {
                uart_bridge_send_camera_frame(jpeg_buf, jpeg_len);
                free(jpeg_buf);
            } else {
                ESP_LOGW("CAM_TASK", "Falla al capturar frame de la cámara.");
            }

            vTaskDelay(pdMS_TO_TICKS(60)); // ~15 FPS máximo en modo streaming continuo
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

// Callback invocado cuando llega un comando de movimiento desde el ESP32-C6 (vía UART)
static void on_control_received(float speed, float angle) {
    if (g_fault_state) {
        ESP_LOGW(TAG, "Comando ignorado: ¡Sistema en estado de falla hardware!");
        if (g_traccion_ok) manejo_set_speed(&g_traccion, 0.0f);
        return;
    }

    // 1. Aplicar velocidad a los dos motores simultáneamente (-100.0 a 100.0%)
    if (g_traccion_ok) {
        manejo_set_speed(&g_traccion, speed);
    }

    // 2. Aplicar ángulo al servomotor de dirección (-45.0 a 45.0°)
    if (g_servo_ok) {
        servo_set_angle(angle);
    }
}

// Callback invocado cuando llega un comando de cámara desde el ESP32-C6 (vía UART)
static void on_cam_command_received(uint8_t cmd, uint8_t param) {
    switch (cmd) {
        case 1: // Tomar foto / snapshot
            g_snapshot_requested = true;
            ESP_LOGI(TAG, "Comando recibido: Snapshot");
            break;
        case 2: // Iniciar stream
            g_cam_streaming = true;
            ESP_LOGI(TAG, "Comando recibido: Iniciar Stream");
            break;
        case 3: // Detener stream
            g_cam_streaming = false;
            ESP_LOGI(TAG, "Comando recibido: Detener Stream");
            break;
        case 4: // Cambiar resolución
            if (param <= 8) {
                arducam_set_resolution((arducam_resolution_t)param);
                ESP_LOGI(TAG, "Comando recibido: Cambiar Resolución -> %d", param);
            }
            break;
        default:
            break;
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "  Iniciando Rover Explorer - ESP32-S3 (UART)     ");
    ESP_LOGI(TAG, "==================================================");

    // 1. Inicializar Puente UART1 con ESP32-C6 (TX=IO43, RX=IO44, 921600 baud)
    ESP_LOGI(TAG, "Iniciando enlace UART Serial con ESP32-C6...");
    esp_err_t err = uart_bridge_init(Config::UART_TX_PIN, 
                                     Config::UART_RX_PIN, 
                                     Config::UART_BAUD,
                                     on_control_received, 
                                     on_cam_command_received);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falla al inicializar UART Bridge.");
    }

    // 2. Inicializar Cámara ArduCAM OV2640 (SPI + I2C en IO47/IO48)
    ESP_LOGI(TAG, "Inicializando ArduCAM OV2640...");
    arducam_config_t cam_cfg = {
        .pin_cs = Config::CAM_CS,
        .pin_mosi = Config::CAM_MOSI,
        .pin_miso = Config::CAM_MISO,
        .pin_sck = Config::CAM_SCK,
        .pin_sda = Config::CAM_SDA,
        .pin_scl = Config::CAM_SCL,
        .spi_freq_hz = 8000000, // 8 MHz SPI
        .i2c_freq_hz = 100000,  // 100 kHz I2C
    };

    if (arducam_init(&cam_cfg) == ESP_OK) {
        g_camera_ok = true;
        ESP_LOGI(TAG, "Hardware ArduCAM OV2640 inicializado con éxito.");
        xTaskCreate(camera_stream_task, "cam_task", 4096, NULL, 5, NULL);
    } else {
        ESP_LOGE(TAG, "Error al inicializar ArduCAM OV2640. Verifique conexiones físicas.");
    }

    // 3. Inicializar Drivers de Tracción Dual DRV8873-Q1 (MCPWM)
    manejo_config_t traccion_cfg = {
        .IN1_M1 = Config::M1_IN1,
        .IN2_M1 = Config::M1_IN2,
        .nFAULT_M1 = Config::M1_NFAULT,
        .nSLEEP_M1 = Config::M1_NSLEEP,
        .DISABLE_M1 = Config::M1_DISABLE,
        .IN1_M2 = Config::M2_IN1,
        .IN2_M2 = Config::M2_IN2,
        .nFAULT_M2 = Config::M2_NFAULT,
        .nSLEEP_M2 = Config::M2_NSLEEP,
        .DISABLE_M2 = Config::M2_DISABLE,
        .pwm_freq = 20000
    };

    if (manejo_init(&traccion_cfg, &g_traccion) == ESP_OK) {
        g_traccion_ok = true;
        ESP_LOGI(TAG, "Drivers de tracción DRV8873 inicializados correctamente.");
    } else {
        ESP_LOGE(TAG, "Error al inicializar la tracción dual.");
    }

    // 4. Inicializar Servomotor de Dirección
    servo_config_t servo_cfg = {
        .gpio_num = Config::SERVO_PIN,
        .min_angle = -45.0f,
        .max_angle = 45.0f,
        .min_pulse_us = 500,   // 0.5ms
        .max_pulse_us = 2500,  // 2.5ms
        .channel = LEDC_CHANNEL_0,
        .timer = LEDC_TIMER_0
    };

    if (servo_init(&servo_cfg) == ESP_OK) {
        g_servo_ok = true;
        ESP_LOGI(TAG, "Servomotor de dirección inicializado en GPIO %d.", Config::SERVO_PIN);
    } else {
        ESP_LOGE(TAG, "Error al inicializar servomotor.");
    }

    // 5. Inicializar Sensor Ambiental BME690 (I2C_NUM_0 SDA=IO5, SCL=IO6)
    if (bme690_init() == ESP_OK) {
        g_bme_ok = true;
        ESP_LOGI(TAG, "Sensor ambiental BME690 inicializado en I2C (SDA=IO5, SCL=IO6).");
    } else {
        ESP_LOGE(TAG, "No se pudo detectar el BME690 en el bus I2C (SDA=IO5, SCL=IO6).");
    }

    // Tarea de telemetría periódica
    xTaskCreate(bme690_telemetry_task, "telemetry_task", 3072, NULL, 5, NULL);

    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "  SISTEMA OPERATIVO Y LISTO PARA CONTROL SERIAL   ");
    ESP_LOGI(TAG, "==================================================");

    // Loop principal: Monitoreo de seguridad y fallas hardware (nFAULT)
    while (true) {
        if (g_traccion_ok) {
            bool has_fault = manejo_check_fault(&g_traccion);

            if (has_fault != g_fault_state) {
                g_fault_state = has_fault;
                if (g_fault_state) {
                    manejo_set_speed(&g_traccion, 0.0f);
                    ESP_LOGE(TAG, "¡ALERTA H-BRIDGE! Se ha detectado una falla en el driver DRV8873.");
                } else {
                    ESP_LOGI(TAG, "Falla del driver recuperada/normalizada.");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}