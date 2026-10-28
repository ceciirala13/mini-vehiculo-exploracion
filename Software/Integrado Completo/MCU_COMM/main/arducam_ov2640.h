/**
 * @file arducam_ov2640.h
 * @brief Native ESP-IDF Driver for ArduCAM Mini 2MP Plus (OV2640)
 *
 * Ported to ESP-IDF v6: SCCB uses the new driver/i2c_master.h API, the SPI host
 * and I2C port are configurable, and frames are captured into a caller-owned
 * (DMA-capable) buffer instead of a heap allocation per frame.
 */

#ifndef ARDUCAM_OV2640_H
#define ARDUCAM_OV2640_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// ArduChip CPLD Registers
#define ARDUCHIP_TEST1       0x00
#define ARDUCHIP_FRAMES      0x01
#define ARDUCHIP_MODE        0x02
#define ARDUCHIP_TIM         0x03
#define ARDUCHIP_FIFO        0x04
#define ARDUCHIP_GPIO        0x06
#define BURST_FIFO_READ      0x3C
#define SINGLE_FIFO_READ     0x3D
#define ARDUCHIP_REV         0x40
#define ARDUCHIP_TRIG        0x41
#define FIFO_SIZE1           0x42
#define FIFO_SIZE2           0x43
#define FIFO_SIZE3           0x44

#define FIFO_CLEAR_MASK      0x01
#define FIFO_START_MASK      0x02
#define FIFO_RDPTR_RST_MASK  0x10
#define FIFO_WRPTR_RST_MASK  0x20
#define CAP_DONE_MASK        0x08

#define OV2640_CHIPID_HIGH   0x0A
#define OV2640_CHIPID_LOW    0x0B
#define OV2640_I2C_ADDR      0x30  // 7-bit I2C address (0x60 >> 1)

#define MAX_FIFO_SIZE        0x7FFFFF  // 8 MByte max buffer

// Supported Resolutions
typedef enum {
    OV2640_RES_160x120 = 0,
    OV2640_RES_176x144,
    OV2640_RES_320x240,
    OV2640_RES_352x288,
    OV2640_RES_640x480,
    OV2640_RES_800x600,
    OV2640_RES_1024x768,
    OV2640_RES_1280x1024,
    OV2640_RES_1600x1200
} arducam_resolution_t;

// Pin and bus configuration
typedef struct {
    gpio_num_t pin_cs;
    gpio_num_t pin_mosi;
    gpio_num_t pin_miso;
    gpio_num_t pin_sck;
    gpio_num_t pin_sda;
    gpio_num_t pin_scl;
    uint32_t spi_freq_hz;  // e.g. 8000000 (8 MHz)
    uint32_t i2c_freq_hz;  // e.g. 100000 (100 kHz)
    spi_host_device_t spi_host;  // e.g. SPI3_HOST (VSPI on ESP32)
    int i2c_port;                // I2C controller used for SCCB, e.g. 0
} arducam_config_t;

/**
 * @brief Initialize SPI and I2C buses, CPLD, and the OV2640 sensor.
 */
esp_err_t arducam_init(const arducam_config_t *config);

/**
 * @brief Test SPI bus integrity by writing and reading ARDUCHIP_TEST1 register (expects 0x55).
 */
esp_err_t arducam_test_spi(void);

/**
 * @brief Verify OV2640 sensor via I2C (expects VID 0x26 and PID 0x41 or 0x42).
 */
esp_err_t arducam_test_i2c(uint8_t *out_vid, uint8_t *out_pid);

/**
 * @brief Change JPEG resolution.
 */
esp_err_t arducam_set_resolution(arducam_resolution_t res);

/**
 * @brief Flush hardware FIFO buffer and clear flags.
 */
void arducam_flush_fifo(void);
void arducam_clear_fifo_flag(void);

/**
 * @brief Trigger a snapshot capture into the FIFO.
 */
void arducam_start_capture(void);

/**
 * @brief Check if snapshot capture is complete.
 */
bool arducam_check_capture_done(void);

/**
 * @brief Read total length of captured JPEG frame in FIFO (in bytes).
 */
uint32_t arducam_read_fifo_length(void);

/**
 * @brief Capture a complete JPEG frame from the camera FIFO into @p buf.
 *
 * Flushes the FIFO, triggers a capture, waits for completion, burst-reads the
 * FIFO over SPI and trims the data to the JPEG markers (0xFF 0xD8 .. 0xFF 0xD9),
 * moving the image to the start of @p buf.
 *
 * @param buf       Destination buffer (should be DMA-capable, MALLOC_CAP_DMA).
 * @param buf_size  Size of @p buf. ESP_ERR_INVALID_SIZE if the frame does not fit.
 * @param out_len   Receives the JPEG length in bytes.
 */
esp_err_t arducam_capture_into(uint8_t *buf, size_t buf_size, size_t *out_len);

/**
 * @brief Read and write CPLD registers over SPI.
 */
uint8_t arducam_read_reg(uint8_t addr);
void arducam_write_reg(uint8_t addr, uint8_t data);
uint8_t arducam_get_bit(uint8_t addr, uint8_t bit);
void arducam_set_bit(uint8_t addr, uint8_t bit);
void arducam_clear_bit(uint8_t addr, uint8_t bit);

/**
 * @brief Read and write sensor registers over I2C.
 */
esp_err_t arducam_write_sensor_reg(uint8_t reg, uint8_t val);
esp_err_t arducam_read_sensor_reg(uint8_t reg, uint8_t *val);

#ifdef __cplusplus
}
#endif

#endif // ARDUCAM_OV2640_H
