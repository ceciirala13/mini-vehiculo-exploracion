/******************************************************************************
 * File    : main.c
 * Project : bno086_test
 * Target  : ESP32-S3-WROOM-1 + BNO086 (UART-RVC mode)
 *
 * Summary
 * -------
 * Entry point for the BNO086 UART-RVC test application.
 *
 * - Initialises the BNO086 driver (GPIO reset + UART1 setup).
 * - Spawns a FreeRTOS task that continuously reads RVC packets and logs
 *   Yaw / Pitch / Roll (degrees) and Acceleration X / Y / Z (m/s²).
 ******************************************************************************/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "bno086_rvc.h"

static const char *TAG = "main";

/* ========================= BNO086 reader task ============================ */

/**
 * FreeRTOS task — continuously reads UART-RVC packets from the BNO086
 * and logs the decoded orientation and acceleration values.
 *
 * The BNO086 streams packets at 100 Hz in RVC mode.  We use a 50 ms
 * timeout per read so we never block the watchdog for too long.
 */
static void bno086_reader_task(void *arg)
{
    (void)arg;

    bno086_rvc_data_t imu;
    uint32_t pkt_count    = 0;
    uint32_t err_timeout  = 0;
    uint32_t err_checksum = 0;

    ESP_LOGI(TAG, "BNO086 reader task started");

    while (1) {
        esp_err_t err = bno086_rvc_read(&imu, 50 /* ms */);

        switch (err) {
        case ESP_OK:
            pkt_count++;

            ESP_LOGI(TAG,
                     "[#%04u | idx %3u]  "
                     "Yaw: %7.2f°  Pitch: %7.2f°  Roll: %7.2f°  |  "
                     "Ax: %6.3f  Ay: %6.3f  Az: %6.3f m/s²",
                     (unsigned)pkt_count,
                     imu.index,
                     imu.yaw_deg, imu.pitch_deg, imu.roll_deg,
                     imu.acc_x,   imu.acc_y,     imu.acc_z);
            break;

        case ESP_ERR_TIMEOUT:
            err_timeout++;
            if ((err_timeout % 20) == 1) {
                ESP_LOGW(TAG, "Read timeout (%u total)", (unsigned)err_timeout);
            }
            break;

        case ESP_ERR_INVALID_CRC:
            err_checksum++;
            ESP_LOGW(TAG, "Checksum error (%u total)", (unsigned)err_checksum);
            break;

        default:
            ESP_LOGE(TAG, "Unexpected error: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(100));
            break;
        }
    }
}

/* ============================== app_main ================================== */

void app_main(void)
{
    ESP_LOGI(TAG, "=== BNO086 UART-RVC Test (ESP32-S3) ===");

    /* Initialise BNO086 driver (reset + UART) */
    esp_err_t err = bno086_rvc_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BNO086 init failed: %s", esp_err_to_name(err));
        return;
    }

    /* Create the reader task — 4 KB stack is plenty for this workload */
    BaseType_t ret = xTaskCreate(
        bno086_reader_task,     /* function */
        "bno086_reader",        /* name */
        4096,                   /* stack (bytes) */
        NULL,                   /* parameter */
        5,                      /* priority (above tskIDLE_PRIORITY) */
        NULL                    /* handle (not needed) */
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create bno086_reader task");
    }
}
