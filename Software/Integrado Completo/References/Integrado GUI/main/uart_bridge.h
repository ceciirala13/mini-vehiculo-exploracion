#ifndef UART_BRIDGE_H
#define UART_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// Delimitador de inicio de trama
#define UART_PKT_MAGIC_0 0xAA
#define UART_PKT_MAGIC_1 0x55

// Tipos de Paquetes
typedef enum {
    PKT_TYPE_HEARTBEAT    = 0x00,
    PKT_TYPE_CONTROL      = 0x01, // C6 -> S3: velocidad y ángulo
    PKT_TYPE_TELEMETRY    = 0x02, // S3 -> C6: BME690 + Baterías + Fault
    PKT_TYPE_CAMERA_CHUNK = 0x03, // S3 -> C6: Fragmentos de imagen JPEG
    PKT_TYPE_CAM_COMMAND  = 0x04  // C6 -> S3: comandos de captura/stream
} uart_pkt_type_t;

// Estructura de Telemetría S3
typedef struct __attribute__((packed)) {
    float temp;
    float hum;
    float press;
    uint32_t gas;
    float v_motors;
    float pct_motors;
    float v_esp;
    float pct_esp;
    uint8_t fault;
} s3_telemetry_pkt_t;

// Estructura de Comando de Control
typedef struct __attribute__((packed)) {
    float speed;
    float angle;
} s3_control_pkt_t;

// Estructura de Comando de Cámara
typedef struct __attribute__((packed)) {
    uint8_t cmd;   // 1=Capture, 2=StartStream, 3=StopStream, 4=SetResolution
    uint8_t param; // e.g. resolution index
} s3_cam_cmd_pkt_t;

// Estructura de Cabecera de Chunk de Cámara
typedef struct __attribute__((packed)) {
    uint32_t frame_id;
    uint32_t total_len;
    uint32_t offset;
    uint16_t chunk_len;
} s3_cam_chunk_hdr_t;

// Callback para recepción de comandos de control
typedef void (*uart_control_cb_t)(float speed, float angle);

// Callback para recepción de comandos de cámara
typedef void (*uart_cam_cmd_cb_t)(uint8_t cmd, uint8_t param);

/**
 * @brief Inicializar el puente UART1 en el ESP32-S3
 * @param tx_pin Pin TX (GPIO 43)
 * @param rx_pin Pin RX (GPIO 44)
 * @param baud_rate Tasa de baudios (e.g. 921600)
 * @param control_cb Callback para procesar comandos de movimiento
 * @param cam_cmd_cb Callback para procesar comandos de cámara
 */
esp_err_t uart_bridge_init(gpio_num_t tx_pin, gpio_num_t rx_pin, uint32_t baud_rate,
                          uart_control_cb_t control_cb, uart_cam_cmd_cb_t cam_cmd_cb);

/**
 * @brief Enviar paquete de telemetría de sensores y estado hacia el ESP32-C6
 */
esp_err_t uart_bridge_send_telemetry(float temp, float hum, float press, uint32_t gas,
                                     float v_motors, float pct_motors,
                                     float v_esp, float pct_esp,
                                     bool fault);

/**
 * @brief Transmitir un frame JPEG completo fragmentado en chunks por UART
 * @param jpeg_buf Buffer en memoria del frame JPEG
 * @param jpeg_len Tamaño total del frame JPEG en bytes
 */
esp_err_t uart_bridge_send_camera_frame(const uint8_t *jpeg_buf, size_t jpeg_len);

#ifdef __cplusplus
}
#endif

#endif // UART_BRIDGE_H
