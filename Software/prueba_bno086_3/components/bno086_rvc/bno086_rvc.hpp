/**
 * @file bno086_rvc.hpp
 * @brief Librería C++ para el BNO086 en modo UART-RVC, para ESP32-S3 con
 * ESP-IDF.
 *
 * Conexiones utilizadas:
 *   - GPIO17 → TX del ESP32-S3 → RX del BNO086  (H_SCL/SCK/RX, pin 19 del
 * BNO086)
 *   - GPIO18 → RX del ESP32-S3 → TX del BNO086  (H_SDA/H_MISO/TX, pin 20 del
 * BNO086)
 *   - GPIO4  → NRST del BNO086  (pin 11, activo en bajo)
 *
 * Configuración de pines de protocolo del BNO086 para UART-RVC:
 *   - PS1 (pin 5) → GND  → 0
 *   - PS0 (pin 6) → VDDIO → 1
 *   BOOTN (pin 4) → 3V3 vía resistencia 10kΩ (modo aplicación normal)
 *
 * Protocolo UART-RVC:
 *   - Baudrate : 115200 bps, 8N1
 *   - Tasa de reporte: 100 Hz
 *   - Paquete de 19 bytes, encabezado 0xAA 0xAA
 *
 * Formato del paquete (19 bytes):
 *   [0-1]  Header   : 0xAA 0xAA
 *   [2]    Index    : contador monotónico 0-255
 *   [3-4]  Yaw      : int16_t LSB primero, unidades 0.01°, rango ±180°
 *   [5-6]  Pitch    : int16_t LSB primero, unidades 0.01°, rango ±90°
 *   [7-8]  Roll     : int16_t LSB primero, unidades 0.01°, rango ±180°
 *   [9-10] Accel X  : int16_t LSB primero, unidades mg
 *   [11-12]Accel Y  : int16_t LSB primero, unidades mg
 *   [13-14]Accel Z  : int16_t LSB primero, unidades mg
 *   [15]   MI       : Motion Intent (BNO086 únicamente)
 *   [16]   MR       : Motion Request (BNO086 únicamente)
 *   [17]   Reserved : 0x00
 *   [18]   Checksum : suma de bytes [2..17]
 */

#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
// #include <cstdbool>
#include <cstdint>

// ─── Constantes del protocolo ────────────────────────────────────────────────

static constexpr uint32_t BNO086_RVC_BAUDRATE = 115200;
static constexpr size_t BNO086_RVC_PACKET_SIZE = 19;
static constexpr uint8_t BNO086_RVC_HEADER_BYTE = 0xAA;
static constexpr uint32_t BNO086_RVC_REPORT_RATE_HZ = 100;

/** Tiempo máximo de espera por un paquete completo (2 períodos a 100 Hz = 20
 * ms) */
static constexpr uint32_t BNO086_RVC_PACKET_TIMEOUT_MS = 20;

/** Tiempo de reset (NRST bajo) en milisegundos */
static constexpr uint32_t BNO086_RESET_PULSE_MS = 10;

/** Tiempo de inicialización interna tras reset (~90 ms según datasheet) */
static constexpr uint32_t BNO086_BOOT_TIME_MS = 150;

// ─── Estructura de datos de orientación ──────────────────────────────────────

/**
 * @brief Datos de orientación e inercia parseados de un paquete UART-RVC.
 *
 * Los ángulos son en grados decimales.
 * Las aceleraciones son en m/s².
 *
 * Orden de aplicación de rotaciones para obtener la orientación real:
 *   Yaw → Pitch → Roll
 */
struct BNO086_RVC_Data {
  uint8_t index; ///< Contador de paquete (0-255, monotónico)

  float yaw_deg;   ///< Rotación alrededor del eje Z, ±180°
  float pitch_deg; ///< Rotación alrededor del eje Y, ±90°
  float roll_deg;  ///< Rotación alrededor del eje X, ±180°

  float accel_x_ms2; ///< Aceleración eje X en m/s²
  float accel_y_ms2; ///< Aceleración eje Y en m/s²
  float accel_z_ms2; ///< Aceleración eje Z en m/s²

  uint8_t motion_intent;  ///< Motion Intent (BNO086)
  uint8_t motion_request; ///< Motion Request (BNO086)

  bool valid; ///< true si el checksum es correcto
};

// ─── Clase principal
// ──────────────────────────────────────────────────────────

/**
 * @brief Driver C++ para el BNO086 en modo UART-RVC sobre ESP-IDF.
 *
 * Uso básico (polling):
 * @code
 *   BNO086_RVC imu(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, GPIO_NUM_4);
 *   imu.begin();
 *   BNO086_RVC_Data data;
 *   while (true) {
 *       if (imu.read(data)) {
 *           // usar data.yaw_deg, data.pitch_deg, data.roll_deg ...
 *       }
 *   }
 * @endcode
 *
 * Uso con tarea FreeRTOS (callback):
 * @code
 *   imu.setDataCallback(myCallback);
 *   imu.startTask();
 * @endcode
 */
class BNO086_RVC {
public:
  /** Tipo del callback invocado desde la tarea interna */
  using DataCallback = void (*)(const BNO086_RVC_Data &data, void *user_ctx);

  /**
   * @brief Constructor.
   * @param uart_num   Puerto UART (p.ej. UART_NUM_1).
   * @param tx_gpio    GPIO conectado al RX del BNO086 (transmisión del ESP32).
   * @param rx_gpio    GPIO conectado al TX del BNO086 (recepción del ESP32).
   * @param rst_gpio   GPIO conectado al NRST del BNO086 (activo en bajo).
   * @param uart_buf   Tamaño del buffer interno de la UART (mínimo 2×paquete).
   */
  explicit BNO086_RVC(uart_port_t uart_num = UART_NUM_1,
                      gpio_num_t tx_gpio = GPIO_NUM_17,
                      gpio_num_t rx_gpio = GPIO_NUM_18,
                      gpio_num_t rst_gpio = GPIO_NUM_4, size_t uart_buf = 512);

  ~BNO086_RVC();

  // ── Ciclo de vida ────────────────────────────────────────────────────────

  /**
   * @brief Inicializa la UART y el GPIO de reset. Ejecuta un reset del BNO086.
   * @return ESP_OK si todo fue correcto.
   */
  esp_err_t begin();

  /**
   * @brief Libera los recursos UART y detiene la tarea interna si está activa.
   */
  void end();

  /**
   * @brief Realiza un reset hardware del BNO086 (pulso en NRST).
   *        Bloquea ~150 ms hasta que el dispositivo está listo.
   */
  void hardReset();

  // ── Lectura de datos (modo polling) ─────────────────────────────────────

  /**
   * @brief Lee y parsea el siguiente paquete UART-RVC disponible.
   * @param[out] out   Estructura rellena con los datos del paquete.
   * @param timeout_ms Tiempo máximo de espera en ms (0 = no esperar).
   * @return true si se obtuvo un paquete válido con checksum correcto.
   */
  bool read(BNO086_RVC_Data &out,
            uint32_t timeout_ms = BNO086_RVC_PACKET_TIMEOUT_MS);

  /**
   * @brief Devuelve el último dato leído con éxito.
   *        Útil cuando se usa la tarea interna.
   */
  BNO086_RVC_Data getLastData() const;

  // ── Tarea FreeRTOS (modo asíncrono) ─────────────────────────────────────

  /**
   * @brief Registra un callback que se invocará por cada paquete recibido.
   * @param cb       Función callback.
   * @param user_ctx Puntero de contexto del usuario (puede ser nullptr).
   */
  void setDataCallback(DataCallback cb, void *user_ctx = nullptr);

  /**
   * @brief Crea y arranca una tarea FreeRTOS que lee la UART continuamente.
   * @param priority   Prioridad de la tarea (por defecto 5).
   * @param stack_size Tamaño del stack en bytes (por defecto 4096).
   * @param core_id    Core de ejecución: 0, 1 o tskNO_AFFINITY.
   * @return ESP_OK si la tarea fue creada.
   */
  esp_err_t startTask(UBaseType_t priority = 5, uint32_t stack_size = 4096,
                      BaseType_t core_id = tskNO_AFFINITY);

  /**
   * @brief Detiene y elimina la tarea FreeRTOS interna.
   */
  void stopTask();

  /**
   * @brief Indica si la tarea interna está activa.
   */
  bool isTaskRunning() const { return task_handle_ != nullptr; }

  // ── Estadísticas ─────────────────────────────────────────────────────────

  /** Número de paquetes válidos recibidos desde begin(). */
  uint32_t getPacketCount() const { return packet_count_; }

  /** Número de errores de checksum acumulados. */
  uint32_t getChecksumErrors() const { return checksum_errors_; }

  /** Número de errores de sincronización (header no encontrado). */
  uint32_t getSyncErrors() const { return sync_errors_; }

  /** Reinicia los contadores de estadísticas. */
  void resetStats();

private:
  // ── Parámetros de configuración ──────────────────────────────────────────
  uart_port_t uart_num_;
  gpio_num_t tx_gpio_;
  gpio_num_t rx_gpio_;
  gpio_num_t rst_gpio_;
  size_t uart_buf_size_;

  // ── Estado interno ───────────────────────────────────────────────────────
  bool initialized_ = false;

  // ── Último dato parseado ─────────────────────────────────────────────────
  BNO086_RVC_Data last_data_ = {};
  mutable SemaphoreHandle_t data_mutex_ = nullptr;

  // ── Tarea FreeRTOS ───────────────────────────────────────────────────────
  TaskHandle_t task_handle_ = nullptr;
  volatile bool task_stop_ = false;
  DataCallback callback_ = nullptr;
  void *callback_ctx_ = nullptr;

  // ── Estadísticas ─────────────────────────────────────────────────────────
  volatile uint32_t packet_count_ = 0;
  volatile uint32_t checksum_errors_ = 0;
  volatile uint32_t sync_errors_ = 0;

  // ── Métodos privados ─────────────────────────────────────────────────────

  /**
   * @brief Busca el header 0xAA 0xAA en la UART y descarta bytes anteriores.
   * @return true si el header fue encontrado dentro del timeout.
   */
  bool syncHeader(uint32_t timeout_ms);

  /**
   * @brief Lee exactamente `len` bytes de la UART.
   * @return true si todos los bytes fueron leídos en tiempo.
   */
  bool readBytes(uint8_t *buf, size_t len, uint32_t timeout_ms);

  /**
   * @brief Verifica el checksum del paquete (suma de bytes [2..17]).
   */
  static bool verifyChecksum(const uint8_t *packet);

  /**
   * @brief Parsea un buffer de 19 bytes ya validado en una estructura de datos.
   */
  static BNO086_RVC_Data parsePacket(const uint8_t *packet);

  /**
   * @brief Función de entrada de la tarea FreeRTOS.
   */
  static void taskEntry(void *arg);

  /**
   * @brief Bucle principal de la tarea.
   */
  void taskLoop();

  // ── No copiable ──────────────────────────────────────────────────────────
  BNO086_RVC(const BNO086_RVC &) = delete;
  BNO086_RVC &operator=(const BNO086_RVC &) = delete;
};
