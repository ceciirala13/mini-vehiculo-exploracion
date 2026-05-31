/**
 * @file bno086_rvc.cpp
 * @brief Implementación del driver BNO086 UART-RVC para ESP32-S3 / ESP-IDF.
 */

#include "bno086_rvc.hpp"
#include "esp_log.h"
#include <cstring>

static const char *TAG = "BNO086_RVC";

// ─── Constantes de conversión ────────────────────────────────────────────────

/** 1 mg en m/s² (g ≈ 9.80665 m/s²) */
static constexpr float MG_TO_MS2 = 9.80665f / 1000.0f;

/** Resolución angular del protocolo UART-RVC: 0.01°/LSB */
static constexpr float ANGLE_SCALE = 0.01f;

// ═══════════════════════════════════════════════════════════════════════════════
// Constructor / Destructor
// ═══════════════════════════════════════════════════════════════════════════════

BNO086_RVC::BNO086_RVC(uart_port_t uart_num, gpio_num_t tx_gpio,
                       gpio_num_t rx_gpio, gpio_num_t rst_gpio, size_t uart_buf)
    : uart_num_(uart_num), tx_gpio_(tx_gpio), rx_gpio_(rx_gpio),
      rst_gpio_(rst_gpio), uart_buf_size_(uart_buf) {
  data_mutex_ = xSemaphoreCreateMutex();
  if (data_mutex_ == nullptr) {
    ESP_LOGE(TAG, "No se pudo crear el mutex de datos");
  }
  std::memset(&last_data_, 0, sizeof(last_data_));
}

BNO086_RVC::~BNO086_RVC() {
  end();
  if (data_mutex_ != nullptr) {
    vSemaphoreDelete(data_mutex_);
    data_mutex_ = nullptr;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Ciclo de vida
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t BNO086_RVC::begin() {
  if (initialized_) {
    ESP_LOGW(TAG, "begin() llamado más de una vez; ignorado");
    return ESP_OK;
  }

  // ── Configurar GPIO de reset (salida, activo en bajo) ────────────────────
  gpio_config_t rst_conf = {};
  rst_conf.pin_bit_mask = (1ULL << rst_gpio_);
  rst_conf.mode = GPIO_MODE_OUTPUT;
  rst_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  rst_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  rst_conf.intr_type = GPIO_INTR_DISABLE;

  esp_err_t err = gpio_config(&rst_conf);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "gpio_config(rst) falló: %s", esp_err_to_name(err));
    return err;
  }

  // Mantener reset en alto (inactivo) inicialmente
  gpio_set_level(rst_gpio_, 1);

  // ── Configurar UART ──────────────────────────────────────────────────────
  const uart_config_t uart_cfg = {
      .baud_rate = static_cast<int>(BNO086_RVC_BAUDRATE),
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 0,
      .source_clk = UART_SCLK_DEFAULT,
      .flags = {},
  };

  err = uart_param_config(uart_num_, &uart_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "uart_param_config falló: %s", esp_err_to_name(err));
    return err;
  }

  // Asignar pines: TX→tx_gpio_, RX→rx_gpio_, sin RTS/CTS
  err = uart_set_pin(uart_num_,
                     tx_gpio_, // UART TX (→ RX del BNO086)
                     rx_gpio_, // UART RX (← TX del BNO086)
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "uart_set_pin falló: %s", esp_err_to_name(err));
    return err;
  }

  // Instalar driver con buffer de recepción; sin buffer TX (solo RX en RVC)
  err = uart_driver_install(uart_num_,
                            uart_buf_size_, // RX buffer
                            0,              // TX buffer (no necesario)
                            0,              // event queue size
                            nullptr,        // event queue handle
                            0);             // intr alloc flags
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "uart_driver_install falló: %s", esp_err_to_name(err));
    return err;
  }

  initialized_ = true;
  ESP_LOGI(TAG, "UART%d iniciada a %lu bps (TX=GPIO%d, RX=GPIO%d, RST=GPIO%d)",
           uart_num_, BNO086_RVC_BAUDRATE, tx_gpio_, rx_gpio_, rst_gpio_);

  // ── Reset hardware del BNO086 ────────────────────────────────────────────
  hardReset();

  return ESP_OK;
}

void BNO086_RVC::end() {
  stopTask();

  if (initialized_) {
    uart_driver_delete(uart_num_);
    initialized_ = false;
    ESP_LOGI(TAG, "Driver UART liberado");
  }
}

void BNO086_RVC::hardReset() {
  ESP_LOGI(TAG, "Realizando reset hardware del BNO086...");

  // Pulso de reset: llevar NRST a bajo durante BNO086_RESET_PULSE_MS
  gpio_set_level(rst_gpio_, 0);
  vTaskDelay(pdMS_TO_TICKS(BNO086_RESET_PULSE_MS));
  gpio_set_level(rst_gpio_, 1);

  // Esperar tiempo de inicialización interna (~90 ms, usamos 150 ms de margen)
  vTaskDelay(pdMS_TO_TICKS(BNO086_BOOT_TIME_MS));

  // Vaciar el buffer UART para descartar cualquier dato anterior
  if (initialized_) {
    uart_flush(uart_num_);
  }

  ESP_LOGI(TAG, "Reset completado. El BNO086 enviará el banner de inicio.");
}

// ═══════════════════════════════════════════════════════════════════════════════
// Lectura en modo polling
// ═══════════════════════════════════════════════════════════════════════════════

bool BNO086_RVC::read(BNO086_RVC_Data &out, uint32_t timeout_ms) {
  if (!initialized_) {
    ESP_LOGE(TAG, "read() llamado antes de begin()");
    return false;
  }

  // 1. Sincronizar con el header 0xAA 0xAA
  if (!syncHeader(timeout_ms)) {
    sync_errors_ += 1;
    return false;
  }

  // 2. Leer los 17 bytes restantes del paquete (19 - 2 bytes de header)
  uint8_t packet[BNO086_RVC_PACKET_SIZE] = {};
  packet[0] = BNO086_RVC_HEADER_BYTE;
  packet[1] = BNO086_RVC_HEADER_BYTE;

  if (!readBytes(&packet[2], BNO086_RVC_PACKET_SIZE - 2, timeout_ms)) {
    ESP_LOGD(TAG, "Timeout al leer cuerpo del paquete");
    return false;
  }

  // 3. Verificar checksum
  if (!verifyChecksum(packet)) {
    checksum_errors_ += 1;
    ESP_LOGD(TAG, "Error de checksum (total: %lu)", checksum_errors_);
    return false;
  }

  // 4. Parsear y almacenar
  out = parsePacket(packet);
  out.valid = true;

  if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
    last_data_ = out;
    xSemaphoreGive(data_mutex_);
  }

  packet_count_ += 1;
  return true;
}

BNO086_RVC_Data BNO086_RVC::getLastData() const {
  BNO086_RVC_Data copy = {};
  if (data_mutex_ != nullptr &&
      xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
    copy = last_data_;
    xSemaphoreGive(data_mutex_);
  }
  return copy;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Tarea FreeRTOS
// ═══════════════════════════════════════════════════════════════════════════════

void BNO086_RVC::setDataCallback(DataCallback cb, void *user_ctx) {
  callback_ = cb;
  callback_ctx_ = user_ctx;
}

esp_err_t BNO086_RVC::startTask(UBaseType_t priority, uint32_t stack_size,
                                BaseType_t core_id) {
  if (task_handle_ != nullptr) {
    ESP_LOGW(TAG, "La tarea ya está en ejecución");
    return ESP_OK;
  }
  if (!initialized_) {
    ESP_LOGE(TAG, "startTask() llamado antes de begin()");
    return ESP_ERR_INVALID_STATE;
  }

  task_stop_ = false;

  BaseType_t ret =
      xTaskCreatePinnedToCore(taskEntry, "bno086_rvc", stack_size, this,
                              priority, &task_handle_, core_id);

  if (ret != pdPASS) {
    task_handle_ = nullptr;
    ESP_LOGE(TAG, "xTaskCreatePinnedToCore falló");
    return ESP_ERR_NO_MEM;
  }

  ESP_LOGI(TAG, "Tarea BNO086 iniciada (prio=%d, stack=%lu)", priority,
           stack_size);
  return ESP_OK;
}

void BNO086_RVC::stopTask() {
  if (task_handle_ == nullptr)
    return;

  task_stop_ = true;
  // Dar tiempo para que la tarea detecte la señal de parada
  vTaskDelay(pdMS_TO_TICKS(50));

  if (task_handle_ != nullptr) {
    vTaskDelete(task_handle_);
    task_handle_ = nullptr;
  }
  ESP_LOGI(TAG, "Tarea BNO086 detenida");
}

void BNO086_RVC::taskEntry(void *arg) {
  static_cast<BNO086_RVC *>(arg)->taskLoop();
  vTaskDelete(nullptr);
}

void BNO086_RVC::taskLoop() {
  ESP_LOGI(TAG, "Tarea BNO086 ejecutándose en core %d", xPortGetCoreID());
  BNO086_RVC_Data data;

  while (!task_stop_) {
    if (read(data, BNO086_RVC_PACKET_TIMEOUT_MS)) {
      if (callback_ != nullptr) {
        callback_(data, callback_ctx_);
      }
    } else {
      // Evitar bucle de alta CPU si falla la lectura (e.g. no hay datos o error)
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }

  task_handle_ = nullptr;
  ESP_LOGI(TAG, "Tarea BNO086 finalizada");
}

// ═══════════════════════════════════════════════════════════════════════════════
// Estadísticas
// ═══════════════════════════════════════════════════════════════════════════════

void BNO086_RVC::resetStats() {
  packet_count_ = 0;
  checksum_errors_ = 0;
  sync_errors_ = 0;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Métodos privados
// ═══════════════════════════════════════════════════════════════════════════════

bool BNO086_RVC::syncHeader(uint32_t timeout_ms) {
  /*
   * El header del protocolo UART-RVC es 0xAA 0xAA.
   * Leemos byte a byte hasta encontrar dos 0xAA consecutivos.
   * Si no encontramos el header dentro del timeout, devolvemos false.
   */
  TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
  uint8_t prev = 0x00;
  uint8_t curr = 0x00;
  uint32_t bytes_read = 0;

  while (xTaskGetTickCount() < deadline) {
    TickType_t wait_ticks = pdMS_TO_TICKS(2);
    if (wait_ticks == 0) {
      wait_ticks = 1; // Asegurar al menos 1 tick de bloqueo para no saturar la CPU
    }
    int len = uart_read_bytes(uart_num_, &curr, 1, wait_ticks);
    if (len <= 0) {
      vTaskDelay(1); // Ceder CPU ante fallos de lectura o timeouts inmediatos
      continue;
    }
    bytes_read++;

    if (prev == BNO086_RVC_HEADER_BYTE && curr == BNO086_RVC_HEADER_BYTE) {
      return true; // Header encontrado
    }
    prev = curr;
  }

  static TickType_t last_log_time = 0;
  if (xTaskGetTickCount() - last_log_time >= pdMS_TO_TICKS(2000)) {
    size_t buffered_len = 0;
    uart_get_buffered_data_len(uart_num_, &buffered_len);
    ESP_LOGW(TAG, "syncHeader timeout! Bytes leídos en ventana: %lu | En buffer RX: %u",
             bytes_read, (unsigned int)buffered_len);
    last_log_time = xTaskGetTickCount();
  }

  return false;
}

bool BNO086_RVC::readBytes(uint8_t *buf, size_t len, uint32_t timeout_ms) {
  size_t received = 0;
  TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

  while (received < len && xTaskGetTickCount() < deadline) {
    TickType_t remaining = deadline - xTaskGetTickCount();
    if (remaining == 0)
      break;

    int got =
        uart_read_bytes(uart_num_, buf + received, len - received, remaining);
    if (got > 0) {
      received += static_cast<size_t>(got);
    } else {
      vTaskDelay(1); // Ceder CPU si no se reciben datos
    }
  }

  return (received == len);
}

bool BNO086_RVC::verifyChecksum(const uint8_t *packet) {
  /*
   * Checksum = suma de los bytes desde el índice [2] hasta [17] (inclusive).
   * El byte [18] contiene el checksum calculado por el BNO086.
   * Comparamos el byte 8 LSB de la suma con el byte [18].
   */
  uint8_t sum = 0;
  for (int i = 2; i <= 17; i++) {
    sum += packet[i];
  }
  return (sum == packet[18]);
}

BNO086_RVC_Data BNO086_RVC::parsePacket(const uint8_t *p) {
  BNO086_RVC_Data d = {};

  // Índice de paquete
  d.index = p[2];

  // Ángulos: int16_t en formato little-endian, escala 0.01°/LSB
  int16_t raw_yaw = static_cast<int16_t>(static_cast<uint16_t>(p[3]) |
                                         (static_cast<uint16_t>(p[4]) << 8));
  int16_t raw_pitch = static_cast<int16_t>(static_cast<uint16_t>(p[5]) |
                                           (static_cast<uint16_t>(p[6]) << 8));
  int16_t raw_roll = static_cast<int16_t>(static_cast<uint16_t>(p[7]) |
                                          (static_cast<uint16_t>(p[8]) << 8));

  d.yaw_deg = static_cast<float>(raw_yaw) * ANGLE_SCALE;
  d.pitch_deg = static_cast<float>(raw_pitch) * ANGLE_SCALE;
  d.roll_deg = static_cast<float>(raw_roll) * ANGLE_SCALE;

  // Aceleraciones: int16_t little-endian, unidades mg → convertir a m/s²
  int16_t raw_ax = static_cast<int16_t>(static_cast<uint16_t>(p[9]) |
                                        (static_cast<uint16_t>(p[10]) << 8));
  int16_t raw_ay = static_cast<int16_t>(static_cast<uint16_t>(p[11]) |
                                        (static_cast<uint16_t>(p[12]) << 8));
  int16_t raw_az = static_cast<int16_t>(static_cast<uint16_t>(p[13]) |
                                        (static_cast<uint16_t>(p[14]) << 8));

  d.accel_x_ms2 = static_cast<float>(raw_ax) * MG_TO_MS2;
  d.accel_y_ms2 = static_cast<float>(raw_ay) * MG_TO_MS2;
  d.accel_z_ms2 = static_cast<float>(raw_az) * MG_TO_MS2;

  // Motion Intent / Motion Request (BNO086)
  d.motion_intent = p[15];
  d.motion_request = p[16];

  // El byte [17] es reservado y [18] es checksum (ya validado)
  d.valid = true;

  return d;
}
