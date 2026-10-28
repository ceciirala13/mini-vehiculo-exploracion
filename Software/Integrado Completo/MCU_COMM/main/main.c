/**
 * @file main.c
 * @brief MCU_COMM: Wi-Fi AP, camera, web GUI and UART bridge to MCU_CONTROL.
 *
 * Target: ESP32 (Dev Board)   SDK: ESP-IDF v6.x
 *
 * Data flow:
 *   Browser/Python --WebSocket/HTTP--> drive_command --(20 Hz CMD_DRIVE)--> UART --> MCU_CONTROL
 *   MCU_CONTROL --(20 Hz TLM_STATE)--> UART --> telemetry_store --(20 Hz JSON)--> WebSocket clients
 *   ArduCAM --> camera_service (double buffer) --> /stream (:81) and /capture (:80)
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"

#include "app_config.h"
#include "rover_proto.h"
#include "camera_service.h"
#include "drive_command.h"
#include "telemetry_store.h"
#include "uart_link.h"
#include "web_server.h"
#include "wifi_ap.h"

static const char *TAG = "MAIN_APP";

void app_main(void)
{
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "  MINI VEHICULO EXPLORACION - MCU_COMM (ESP32)   ");
    ESP_LOGI(TAG, "  Protocol v%d                                   ", ROVER_PROTO_VERSION);
    ESP_LOGI(TAG, "=================================================");

    /* 1. NVS (required by Wi-Fi) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 2. Shared state, then the UART link (starts sending neutral heartbeats) */
    drive_command_init();
    telemetry_store_init();
    ESP_ERROR_CHECK(uart_link_init());

    /* 3. Camera: not fatal, the GUI and driving work without it.
     *    Allocated before Wi-Fi so the DMA frame buffers get contiguous RAM. */
    if (camera_service_init() != ESP_OK) {
        ESP_LOGW(TAG, "Continuing without camera");
    }

    /* 4. Network */
    ESP_ERROR_CHECK(wifi_ap_start());
    ESP_ERROR_CHECK(web_server_start());

    ESP_LOGI(TAG, "Ready. Connect to '%s' and open http://192.168.4.1  (free heap: %u bytes)",
             WIFI_AP_SSID, (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT));
}
