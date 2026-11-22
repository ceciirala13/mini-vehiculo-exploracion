/**
 * @file wifi_ap.h
 * @brief Wi-Fi Soft-AP (SSID/password in app_config.h, IP 192.168.4.1).
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Init netif/event loop and start the access point. NVS must be initialized.
 */
esp_err_t wifi_ap_start(void);

/**
 * @brief Number of stations currently associated.
 */
int wifi_ap_station_count(void);

#ifdef __cplusplus
}
#endif
