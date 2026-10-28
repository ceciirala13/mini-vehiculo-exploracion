/**
 * @file web_server.h
 * @brief HTTP/WebSocket servers for the rover GUI.
 *
 * Server #1 (WEB_HTTP_PORT, 80):
 *   GET /               embedded index.html
 *   GET /ws             WebSocket: in  {"speed":S,"angle":A} | {"estop":1} | {"ping":T} | {"res":N}
 *                                  out telemetry JSON at 20 Hz, {"pong":T}
 *   GET /api/telemetry  telemetry JSON (same as WebSocket push)
 *   GET /api/control    ?speed=S&angle=A   (for scripts; must be repeated < 500 ms)
 *   GET /api/estop
 *   GET /capture        single JPEG
 *   GET /resolution     ?res=N (0..CAM_MAX_RES)
 *   GET /status         static info + heap
 *
 * Server #2 (WEB_STREAM_PORT, 81):
 *   GET /stream         MJPEG (multipart/x-mixed-replace)
 *
 * The stream lives on its own server because its handler never returns; on a
 * single server it would block every other request (including the WebSocket).
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t web_server_start(void);

#ifdef __cplusplus
}
#endif
