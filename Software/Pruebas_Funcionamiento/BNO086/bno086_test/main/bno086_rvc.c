/******************************************************************************
 * File    : bno086_rvc.c
 * Module  : BNO086 UART-RVC Driver for ESP-IDF (ESP32-S3)
 * Version : 1.0.0
 * License : MIT
 *
 * Implementation of the minimal BNO086 UART-RVC driver.
 * See bno086_rvc.h for protocol details and API documentation.
 ******************************************************************************/

#include "bno086_rvc.h"

#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bno086";

/* ========================= Internal constants ============================ */

/** RVC packet header bytes. */
#define RVC_HEADER_BYTE  0xAA

/** Size of the data portion after the 2-byte header (17 bytes). */
#define RVC_DATA_LEN     (BNO086_RVC_PKT_LEN - 2)

/* ========================= Hardware reset ================================= */

/**
 * Drive NRST low for 20 ms, then release high and wait 300 ms for the
 * BNO086 to finish its internal boot sequence into RVC mode.
 */
static void bno086_hw_reset(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BNO086_RST_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    /* Ensure pin starts HIGH before the reset pulse */
    gpio_set_level(BNO086_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Assert reset (active low) */
    gpio_set_level(BNO086_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(20));

    /* Release reset */
    gpio_set_level(BNO086_RST_PIN, 1);

    /* Wait for BNO086 to boot (datasheet recommends >= 100 ms, we use 300) */
    vTaskDelay(pdMS_TO_TICKS(300));

    ESP_LOGI(TAG, "Hardware reset complete (GPIO %d)", BNO086_RST_PIN);
}

/* ========================= UART configuration ============================= */

/**
 * Configure UART1 for RVC communication.
 *
 * - 115200 baud, 8N1
 * - RX on GPIO 17, TX on GPIO 18
 * - 1024-byte RX ring buffer, no TX buffer (we don't transmit in RVC mode)
 */
static esp_err_t bno086_uart_init(void)
{
    const uart_config_t uart_cfg = {
        .baud_rate  = BNO086_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err;

    err = uart_param_config(BNO086_UART_NUM, &uart_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_set_pin(BNO086_UART_NUM,
                       BNO086_UART_TX_PIN,   /* TX */
                       BNO086_UART_RX_PIN,   /* RX */
                       UART_PIN_NO_CHANGE,   /* RTS */
                       UART_PIN_NO_CHANGE);  /* CTS */
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        return err;
    }

    /* RX buffer = 1024 B, TX buffer = 0 (RVC is receive-only) */
    err = uart_driver_install(BNO086_UART_NUM,
                              BNO086_UART_RX_BUF, /* rx buf */
                              0,                   /* tx buf */
                              0,                   /* queue size */
                              NULL,                /* queue handle */
                              0);                  /* intr alloc flags */
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "UART%d configured: %d baud, RX=GPIO%d, TX=GPIO%d, buf=%d",
             BNO086_UART_NUM, BNO086_UART_BAUD,
             BNO086_UART_RX_PIN, BNO086_UART_TX_PIN,
             BNO086_UART_RX_BUF);
    return ESP_OK;
}

/* ============================ Packet reader =============================== */

/**
 * Read exactly @p len bytes from UART with a deadline.
 *
 * @return number of bytes actually read (may be < len on timeout).
 */
static int uart_read_exact(uint8_t *buf, size_t len, TickType_t deadline)
{
    size_t total = 0;

    while (total < len) {
        TickType_t now = xTaskGetTickCount();
        if ((int32_t)(deadline - now) <= 0) {
            break;  /* deadline expired */
        }
        TickType_t remaining = deadline - now;

        int n = uart_read_bytes(BNO086_UART_NUM,
                                buf + total,
                                len - total,
                                remaining);
        if (n > 0) {
            total += (size_t)n;
        } else {
            break;  /* timeout or error */
        }
    }
    return (int)total;
}

/* =============================== Public API =============================== */

esp_err_t bno086_rvc_init(void)
{
    /* 1. Hardware reset the BNO086 */
    bno086_hw_reset();

    /* 2. Configure UART */
    esp_err_t err = bno086_uart_init();
    if (err != ESP_OK) {
        return err;
    }

    /* 3. Flush any stale bytes that arrived during boot */
    uart_flush_input(BNO086_UART_NUM);

    ESP_LOGI(TAG, "BNO086 UART-RVC driver initialised");
    return ESP_OK;
}

esp_err_t bno086_rvc_read(bno086_rvc_data_t *out, uint32_t timeout_ms)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    uint8_t byte_buf;

    /*
     * Scan for the 0xAA 0xAA header.
     *
     * State machine:
     *   state 0 → waiting for first  0xAA
     *   state 1 → waiting for second 0xAA
     */
    int state = 0;

    while ((int32_t)(deadline - xTaskGetTickCount()) > 0) {
        TickType_t remaining = deadline - xTaskGetTickCount();
        int n = uart_read_bytes(BNO086_UART_NUM, &byte_buf, 1, remaining);
        if (n <= 0) {
            return ESP_ERR_TIMEOUT;
        }

        if (byte_buf == RVC_HEADER_BYTE) {
            state++;
            if (state >= 2) {
                break;  /* header found */
            }
        } else {
            state = 0;  /* reset — not part of header */
        }
    }

    if (state < 2) {
        return ESP_ERR_TIMEOUT;
    }

    /* Read the remaining 17 bytes (index + data + checksum) */
    uint8_t data[RVC_DATA_LEN];
    int got = uart_read_exact(data, RVC_DATA_LEN, deadline);
    if (got < (int)RVC_DATA_LEN) {
        ESP_LOGW(TAG, "Incomplete packet: got %d of %d bytes", got, RVC_DATA_LEN);
        return ESP_ERR_TIMEOUT;
    }

    /* Validate checksum: sum of bytes 2..17 (i.e. data[0..15]) mod 256 */
    uint8_t sum = 0;
    for (int i = 0; i < (RVC_DATA_LEN - 1); i++) {   /* data[0..15] */
        sum += data[i];
    }

    uint8_t received_cksum = data[RVC_DATA_LEN - 1];  /* data[16] = byte 18 */
    if (sum != received_cksum) {
        ESP_LOGW(TAG, "Checksum mismatch: computed 0x%02X, received 0x%02X",
                 sum, received_cksum);
        return ESP_ERR_INVALID_CRC;
    }

    /* Decode fields (all int16 values are little-endian) */
    out->index = data[0];

    int16_t raw_yaw   = (int16_t)(data[1]  | (data[2]  << 8));
    int16_t raw_pitch = (int16_t)(data[3]  | (data[4]  << 8));
    int16_t raw_roll  = (int16_t)(data[5]  | (data[6]  << 8));
    int16_t raw_ax    = (int16_t)(data[7]  | (data[8]  << 8));
    int16_t raw_ay    = (int16_t)(data[9]  | (data[10] << 8));
    int16_t raw_az    = (int16_t)(data[11] | (data[12] << 8));

    out->yaw_deg   = raw_yaw   * BNO086_DEGREE_SCALE;
    out->pitch_deg = raw_pitch * BNO086_DEGREE_SCALE;
    out->roll_deg  = raw_roll  * BNO086_DEGREE_SCALE;

    out->acc_x = raw_ax * BNO086_MILLI_G_TO_MS2;
    out->acc_y = raw_ay * BNO086_MILLI_G_TO_MS2;
    out->acc_z = raw_az * BNO086_MILLI_G_TO_MS2;

    out->mi = data[13];
    out->mr = data[14];

    return ESP_OK;
}
