/******************************************************************************
 * File    : bno086_rvc.h
 * Module  : BNO086 UART-RVC Driver for ESP-IDF (ESP32-S3)
 * Version : 1.0.0
 * License : MIT
 *
 * Summary
 * -------
 * Minimal driver for the BNO086 IMU operating in UART-RVC (Robot Vacuum
 * Cleaner) mode.  In this mode the sensor automatically streams 19-byte
 * orientation + acceleration packets at 100 Hz over UART at 115200 baud.
 * No initialisation commands are required — the host only needs to reset
 * the chip and start reading.
 *
 * Hardware prerequisites (from schematic)
 * ----------------------------------------
 *   PS1 = GND, PS0 = 3V3  →  selects UART-RVC mode
 *   BNO TX  → ESP32 GPIO 17 (UART1 RX)
 *   BNO RX  → ESP32 GPIO 18 (UART1 TX)
 *   BNO NRST → ESP32 GPIO 4 (active-low reset)
 *
 * RVC Packet (19 bytes)
 * ----------------------
 *   [0-1]   0xAA 0xAA        header
 *   [2]     index            monotonic counter 0-255
 *   [3-4]   yaw   (int16 LE) × 0.01°
 *   [5-6]   pitch (int16 LE) × 0.01°
 *   [7-8]   roll  (int16 LE) × 0.01°
 *   [9-10]  ax    (int16 LE) milli-g
 *   [11-12] ay    (int16 LE) milli-g
 *   [13-14] az    (int16 LE) milli-g
 *   [15]    motion intent
 *   [16]    motion request
 *   [17]    reserved
 *   [18]    checksum = (sum of bytes 2..17) & 0xFF
 ******************************************************************************/

#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== Hardware pin defines ========================== */

#define BNO086_UART_NUM       UART_NUM_1   /**< ESP32 UART peripheral        */
#define BNO086_UART_TX_PIN    18           /**< ESP32 GPIO → BNO RX          */
#define BNO086_UART_RX_PIN    17           /**< BNO TX → ESP32 GPIO          */
#define BNO086_UART_BAUD      115200       /**< Fixed baud for RVC mode      */
#define BNO086_RST_PIN        4            /**< NRST, active-low             */

#define BNO086_UART_RX_BUF    1024         /**< UART RX ring-buffer size     */
#define BNO086_RVC_PKT_LEN    19           /**< Fixed RVC packet length      */

/* ============================ Scaling factors ============================= */

#define BNO086_DEGREE_SCALE   0.01f        /**< Raw angle → degrees          */
#define BNO086_MILLI_G_TO_MS2 0.0098067f   /**< milli-g → m/s²              */

/* ============================== Data types ================================ */

/**
 * Decoded RVC data packet.
 *
 * Angles are in degrees, acceleration in m/s².
 */
typedef struct {
    uint8_t index;      /**< Monotonic packet counter (0-255)              */
    float   yaw_deg;    /**< Yaw   in degrees (-180 .. +180)               */
    float   pitch_deg;  /**< Pitch in degrees (-90  .. +90)                */
    float   roll_deg;   /**< Roll  in degrees (-180 .. +180)               */
    float   acc_x;      /**< Acceleration X in m/s²                        */
    float   acc_y;      /**< Acceleration Y in m/s²                        */
    float   acc_z;      /**< Acceleration Z in m/s²                        */
    uint8_t mi;         /**< Motion intent                                 */
    uint8_t mr;         /**< Motion request                                */
} bno086_rvc_data_t;

/* =============================== API ====================================== */

/**
 * Initialise the BNO086 driver.
 *
 * - Drives NRST low for 20 ms then releases (hardware reset).
 * - Configures UART1 at 115200-8N1 on GPIO 17 (RX) / GPIO 18 (TX).
 * - Waits 300 ms for the sensor to boot into RVC mode.
 *
 * @return ESP_OK on success, or an error code from the UART driver.
 */
esp_err_t bno086_rvc_init(void);

/**
 * Read and decode one valid RVC packet.
 *
 * Scans the UART stream for the 0xAA 0xAA header, reads the remaining
 * 17 bytes, validates the checksum, and decodes the fields into @p out.
 *
 * @param[out] out        Pointer to receive decoded data.
 * @param[in]  timeout_ms Maximum time (ms) to wait for a complete packet.
 *
 * @return ESP_OK            – valid packet received and decoded.
 * @return ESP_ERR_TIMEOUT   – no complete packet within the timeout.
 * @return ESP_ERR_INVALID_CRC – packet received but checksum mismatch.
 */
esp_err_t bno086_rvc_read(bno086_rvc_data_t *out, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
