/**
 * @file app_config.h
 * @brief MCU_COMM master configuration: Wi-Fi, pins, rates, task layout.
 *
 * Target: ESP32 (classic, Dev Board, no PSRAM assumed)   SDK: ESP-IDF v6.x
 */

#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * 1. WI-FI SOFT-AP
 * ========================================================================= */
#define WIFI_AP_SSID                    "ROVER_EXPLORER_AP"
#define WIFI_AP_PASS                    "rover1234"     /* >= 8 chars, "" = open network */
#define WIFI_AP_CHANNEL                 1
#define WIFI_AP_MAX_CONN                4
/* AP IP is the IDF default: 192.168.4.1 */

/* =========================================================================
 * 2. UART LINK TO MCU_CONTROL (ESP32-S3)
 * ========================================================================= */
#define LINK_UART_PORT                  UART_NUM_2
#define LINK_UART_TX_PIN                GPIO_NUM_17     /* -> S3 RX GPIO44 */
#define LINK_UART_RX_PIN                GPIO_NUM_16   /* <- S3 TX GPIO43 */
#define LINK_UART_BAUD_RATE             115200
#define LINK_UART_RX_BUF_SIZE           2048
#define LINK_UART_TX_BUF_SIZE           1024

/* CONTROL is considered offline if no valid telemetry for this long */
#define CTRL_LINK_TIMEOUT_MS            500

/* =========================================================================
 * 3. ARDUCAM MINI 2MP PLUS (OV2640)
 * ========================================================================= */
#define CAM_SPI_HOST                    SPI3_HOST       /* VSPI native pins */
#define CAM_PIN_SCK                     GPIO_NUM_18
#define CAM_PIN_MISO                    GPIO_NUM_19
#define CAM_PIN_MOSI                    GPIO_NUM_23
#define CAM_PIN_CS                      GPIO_NUM_5
#define CAM_SPI_FREQ_HZ                 8000000

#define CAM_I2C_PORT                    0               /* SCCB */
#define CAM_PIN_SDA                     GPIO_NUM_21
#define CAM_PIN_SCL                     GPIO_NUM_22
#define CAM_I2C_FREQ_HZ                 100000

/* Frame buffers (x2, DMA-capable internal RAM). Without PSRAM keep the
 * resolution at or below 640x480 so a JPEG fits. */
#define CAM_FRAME_BUF_SIZE              (40 * 1024)
#define CAM_DEFAULT_RES                 2               /* OV2640_RES_320x240 */
#define CAM_MAX_RES                     4               /* OV2640_RES_640x480 */
#define CAM_MIN_FRAME_INTERVAL_MS       40              /* cap at ~25 fps */

/* =========================================================================
 * 4. CONTROL INPUT / SAFETY
 * ========================================================================= */
/* No control message from any web/Python client for this long -> neutral */
#define GUI_TIMEOUT_MS                  500
#define DRIVE_MAX_SPEED_PCT             100.0f
#define DRIVE_MAX_STEER_DEG             45.0f

/* =========================================================================
 * 5. WEB SERVER
 * ========================================================================= */
#define WEB_HTTP_PORT                   80      /* GUI, WebSocket, REST, /capture */
#define WEB_STREAM_PORT                 81      /* MJPEG /stream (separate server) */
#define WEB_MAX_WS_CLIENTS              4

/* =========================================================================
 * 6. TASK LAYOUT
 * ========================================================================= */
#define TASK_UART_RX_CORE               1
#define TASK_UART_RX_PRIO               10
#define TASK_UART_RX_STACK              4096

#define TASK_DRIVE_TX_CORE              1
#define TASK_DRIVE_TX_PRIO              9
#define TASK_DRIVE_TX_STACK             3072
#define TASK_DRIVE_TX_PERIOD_MS         50      /* 20 Hz CMD_DRIVE (also heartbeat) */

#define TASK_WS_TLM_PRIO                5
#define TASK_WS_TLM_STACK               4096
#define TASK_WS_TLM_PERIOD_MS           50      /* 20 Hz telemetry push to browsers */

#define TASK_CAM_CORE                   1
#define TASK_CAM_PRIO                   4
#define TASK_CAM_STACK                  4096

#ifdef __cplusplus
}
#endif
