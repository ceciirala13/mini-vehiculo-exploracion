/**
 * @file camera_service.h
 * @brief Single owner of the ArduCAM: one capture task, double-buffered "latest frame".
 *
 * HTTP handlers never talk to the camera directly. They acquire the most recent
 * published frame (optionally waiting for a newer one), send it, and release it.
 * The capture task only runs while there is demand (stream viewers or a pending
 * snapshot request), so the camera does not load the CPU/SPI when unused.
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t camera_service_init(void);

bool camera_service_available(void);

/** Stream viewers keep the capture loop running while registered. */
void camera_service_viewer_add(void);
void camera_service_viewer_remove(void);

/** Ask for one fresh frame even if nobody is streaming. */
void camera_service_request_snapshot(void);

/**
 * @brief Wait for a frame with id > @p after_id and lock it for reading.
 *        On ESP_OK the caller MUST call camera_service_release().
 */
esp_err_t camera_service_acquire(uint32_t after_id, uint32_t timeout_ms,
                                 const uint8_t **data, size_t *len, uint32_t *frame_id);
void camera_service_release(void);

/** Resolution index (arducam_resolution_t). Applied by the capture task. */
esp_err_t camera_service_set_resolution(int res);
int camera_service_get_resolution(void);

/** Measured capture rate (frames/s) and last JPEG size. */
float camera_service_get_fps(void);
size_t camera_service_get_last_size(void);

#ifdef __cplusplus
}
#endif
