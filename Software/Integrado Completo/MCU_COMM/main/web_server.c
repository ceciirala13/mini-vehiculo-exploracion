/**
 * @file web_server.c
 * @brief HTTP/WebSocket servers for the rover GUI (see web_server.h).
 */

#include "web_server.h"
#include "app_config.h"
#include "camera_service.h"
#include "drive_command.h"
#include "telemetry_store.h"
#include "wifi_ap.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "WEB";

static httpd_handle_t s_http = NULL;
static httpd_handle_t s_stream = NULL;
static volatile bool  s_broadcast_pending = false;

extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[]   asm("_binary_index_html_end");

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

#define JSON_BUF_SIZE   1024
#define WS_RX_MAX       256

/* =========================================================================
 * Helpers
 * ========================================================================= */

/* Minimal JSON number lookup: finds "key" then parses the number after ':'. */
static bool json_get_number(const char *json, const char *key, float *out)
{
    char pattern[24];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *p = strstr(json, pattern);
    if (!p) return false;
    p = strchr(p + strlen(pattern), ':');
    if (!p) return false;
    char *end = NULL;
    float v = strtof(p + 1, &end);
    if (end == p + 1) {
        /* booleans are accepted as 0/1 */
        while (*++p == ' ') {}
        if (strncmp(p, "true", 4) == 0)  { *out = 1.0f; return true; }
        if (strncmp(p, "false", 5) == 0) { *out = 0.0f; return true; }
        return false;
    }
    *out = v;
    return true;
}

static bool query_get_float(httpd_req_t *req, const char *key, float *out)
{
    char query[96];
    char val[16];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) return false;
    if (httpd_query_key_value(query, key, val, sizeof(val)) != ESP_OK) return false;
    *out = strtof(val, NULL);
    return true;
}

static esp_err_t send_json(httpd_req_t *req, const char *json)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

/* =========================================================================
 * Port 80 handlers
 * ========================================================================= */
static esp_err_t index_handler(httpd_req_t *req)
{
    /* EMBED_TXTFILES appends a NUL terminator */
    const size_t len = (size_t)(index_html_end - index_html_start) - 1;
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, index_html_start, len);
}

static esp_err_t api_telemetry_handler(httpd_req_t *req)
{
    char *buf = malloc(JSON_BUF_SIZE);
    if (!buf) return httpd_resp_send_500(req);
    if (telemetry_store_to_json(buf, JSON_BUF_SIZE) == 0) {
        strcpy(buf, "{}");
    }
    esp_err_t ret = send_json(req, buf);
    free(buf);
    return ret;
}

static esp_err_t api_control_handler(httpd_req_t *req)
{
    float speed = 0.0f, angle = 0.0f;
    if (!query_get_float(req, "speed", &speed) || !query_get_float(req, "angle", &angle)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "speed and angle required");
        return ESP_FAIL;
    }
    drive_command_set(speed, angle);

    rp_tlm_state_t t;
    bool online = telemetry_store_get(&t, NULL);
    char resp[128];
    snprintf(resp, sizeof(resp), "{\"status\":\"ok\",\"speed\":%.1f,\"angle\":%.1f,\"fault\":%s,\"rover_connected\":%s}",
             speed, angle, (t.status & RP_ST_MOTOR_FAULT) ? "true" : "false", online ? "true" : "false");
    return send_json(req, resp);
}

static esp_err_t api_estop_handler(httpd_req_t *req)
{
    drive_command_estop();
    return send_json(req, "{\"status\":\"ok\",\"estop\":true}");
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    if (!camera_service_available()) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_send(req, "camera not available", HTTPD_RESP_USE_STRLEN);
    }

    const uint8_t *data;
    size_t len;
    uint32_t id;
    /* Wait for a frame captured after this request */
    if (camera_service_acquire(0, 0, &data, &len, &id) == ESP_OK) {
        camera_service_release();
    } else {
        id = 0;
    }
    camera_service_request_snapshot();
    if (camera_service_acquire(id, 3000, &data, &len, &id) != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    esp_err_t ret = httpd_resp_send(req, (const char *)data, len);
    camera_service_release();
    return ret;
}

static esp_err_t resolution_handler(httpd_req_t *req)
{
    float res = -1.0f;
    if (!query_get_float(req, "res", &res) || camera_service_set_resolution((int)res) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "res out of range");
        return ESP_FAIL;
    }
    char resp[64];
    snprintf(resp, sizeof(resp), "{\"status\":\"ok\",\"res\":%d}", (int)res);
    return send_json(req, resp);
}

static esp_err_t status_handler(httpd_req_t *req)
{
    char resp[256];
    snprintf(resp, sizeof(resp),
             "{\"sensor\":\"OV2640 2MP PLUS\",\"mode\":\"WiFi_AP\",\"ip\":\"192.168.4.1\","
             "\"ssid\":\"%s\",\"stations\":%d,\"free_heap\":%u,\"min_free_heap\":%u,"
             "\"stream_port\":%d,\"cam_max_res\":%d,\"proto\":%d}",
             WIFI_AP_SSID, wifi_ap_station_count(),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT),
             WEB_STREAM_PORT, CAM_MAX_RES, ROVER_PROTO_VERSION);
    return send_json(req, resp);
}

/* =========================================================================
 * WebSocket
 * ========================================================================= */
static void ws_handle_message(httpd_req_t *req, const char *msg)
{
    float v, speed, angle;

    if (json_get_number(msg, "estop", &v) && v != 0.0f) {
        drive_command_estop();
    } else if (json_get_number(msg, "speed", &speed) && json_get_number(msg, "angle", &angle)) {
        drive_command_set(speed, angle);
    }

    if (json_get_number(msg, "res", &v)) {
        camera_service_set_resolution((int)v);
    }

    const char *ping = strstr(msg, "\"ping\"");
    if (ping) {
        /* echo the numeric token verbatim (a float would lose Date.now() precision) */
        const char *p = strchr(ping, ':');
        char token[24] = "0";
        if (p) {
            p++;
            while (*p == ' ') p++;
            size_t i = 0;
            while (i < sizeof(token) - 1 && (p[i] == '-' || p[i] == '.' || (p[i] >= '0' && p[i] <= '9'))) {
                token[i] = p[i];
                i++;
            }
            token[i] = '\0';
            if (i == 0) strcpy(token, "0");
        }
        char resp[48];
        int n = snprintf(resp, sizeof(resp), "{\"pong\":%s}", token);
        httpd_ws_frame_t pkt = {
            .type = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t *)resp,
            .len = (size_t)n,
        };
        httpd_ws_send_frame(req, &pkt);
    }
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WebSocket client connected (fd %d)", httpd_req_to_sockfd(req));
        return ESP_OK;
    }

    httpd_ws_frame_t pkt = { .type = HTTPD_WS_TYPE_TEXT };
    esp_err_t ret = httpd_ws_recv_frame(req, &pkt, 0);       /* get length */
    if (ret != ESP_OK) return ret;
    if (pkt.len == 0 || pkt.len >= WS_RX_MAX) {
        return ESP_OK;                                        /* ignore oversize */
    }

    uint8_t buf[WS_RX_MAX];
    pkt.payload = buf;
    ret = httpd_ws_recv_frame(req, &pkt, pkt.len);
    if (ret != ESP_OK) return ret;
    buf[pkt.len] = '\0';

    if (pkt.type == HTTPD_WS_TYPE_TEXT) {
        ws_handle_message(req, (const char *)buf);
    }
    return ESP_OK;
}

/* Runs inside the httpd task (queued with httpd_queue_work) */
static void ws_broadcast_work(void *arg)
{
    (void)arg;
    static char json[JSON_BUF_SIZE];
    size_t len = telemetry_store_to_json(json, sizeof(json));

    size_t fds_n = WEB_MAX_WS_CLIENTS + 4;
    int fds[WEB_MAX_WS_CLIENTS + 4];
    if (len > 0 && httpd_get_client_list(s_http, &fds_n, fds) == ESP_OK) {
        httpd_ws_frame_t pkt = {
            .type = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t *)json,
            .len = len,
        };
        for (size_t i = 0; i < fds_n; i++) {
            if (httpd_ws_get_fd_info(s_http, fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
                httpd_ws_send_frame_async(s_http, fds[i], &pkt);
            }
        }
    }
    s_broadcast_pending = false;
}

static void ws_telemetry_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        if (s_http && !s_broadcast_pending) {
            s_broadcast_pending = true;
            if (httpd_queue_work(s_http, ws_broadcast_work, NULL) != ESP_OK) {
                s_broadcast_pending = false;
            }
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_WS_TLM_PERIOD_MS));
    }
}

/* =========================================================================
 * Port 81: MJPEG stream
 * ========================================================================= */
static esp_err_t stream_handler(httpd_req_t *req)
{
    if (!camera_service_available()) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_send(req, "camera not available", HTTPD_RESP_USE_STRLEN);
    }

    httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    ESP_LOGI(TAG, "MJPEG stream started (fd %d)", httpd_req_to_sockfd(req));
    camera_service_viewer_add();

    esp_err_t res = ESP_OK;
    uint32_t last_id = 0;
    int idle_timeouts = 0;
    char part[80];

    while (res == ESP_OK) {
        const uint8_t *data;
        size_t len;
        if (camera_service_acquire(last_id, 2000, &data, &len, &last_id) != ESP_OK) {
            if (++idle_timeouts >= 5) {     /* camera stalled for 10 s: drop the client */
                res = ESP_FAIL;
            }
            continue;
        }
        idle_timeouts = 0;
        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        if (res == ESP_OK) {
            int hlen = snprintf(part, sizeof(part), STREAM_PART, (unsigned)len);
            res = httpd_resp_send_chunk(req, part, hlen);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)data, len);
        }
        camera_service_release();
    }

    camera_service_viewer_remove();
    ESP_LOGI(TAG, "MJPEG stream ended");
    return res;
}

/* =========================================================================
 * Start
 * ========================================================================= */
static void register_get(httpd_handle_t h, const char *uri, esp_err_t (*fn)(httpd_req_t *))
{
    httpd_uri_t u = { .uri = uri, .method = HTTP_GET, .handler = fn, .user_ctx = NULL };
    ESP_ERROR_CHECK(httpd_register_uri_handler(h, &u));
}

esp_err_t web_server_start(void)
{
    /* --- Server #1: GUI, WebSocket, REST --- */
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port       = WEB_HTTP_PORT;
    cfg.ctrl_port         = ESP_HTTPD_DEF_CTRL_PORT;
    cfg.max_uri_handlers  = 12;
    cfg.max_open_sockets  = WEB_MAX_WS_CLIENTS + 2;
    cfg.stack_size        = 6144;
    cfg.lru_purge_enable  = true;
    cfg.send_wait_timeout = 2;
    cfg.recv_wait_timeout = 5;
    cfg.core_id           = 0;

    esp_err_t ret = httpd_start(&s_http, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        return ret;
    }

    register_get(s_http, "/", index_handler);
    register_get(s_http, "/api/telemetry", api_telemetry_handler);
    register_get(s_http, "/api/control", api_control_handler);
    register_get(s_http, "/api/estop", api_estop_handler);
    register_get(s_http, "/capture", capture_handler);
    register_get(s_http, "/resolution", resolution_handler);
    register_get(s_http, "/status", status_handler);

    httpd_uri_t ws = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .user_ctx = NULL,
        .is_websocket = true,
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http, &ws));

    /* --- Server #2: MJPEG stream --- */
    httpd_config_t scfg = HTTPD_DEFAULT_CONFIG();
    scfg.server_port       = WEB_STREAM_PORT;
    scfg.ctrl_port         = ESP_HTTPD_DEF_CTRL_PORT + 1;
    scfg.max_uri_handlers  = 2;
    scfg.max_open_sockets  = 3;
    scfg.stack_size        = 4096;
    scfg.lru_purge_enable  = true;
    scfg.send_wait_timeout = 2;
    scfg.core_id           = 1;

    ret = httpd_start(&s_stream, &scfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start stream server: %s", esp_err_to_name(ret));
        return ret;
    }
    register_get(s_stream, "/stream", stream_handler);

    xTaskCreate(ws_telemetry_task, "ws_tlm", TASK_WS_TLM_STACK, NULL, TASK_WS_TLM_PRIO, NULL);

    ESP_LOGI(TAG, "HTTP on :%d (GUI, /ws, /api), MJPEG on :%d/stream", WEB_HTTP_PORT, WEB_STREAM_PORT);
    return ESP_OK;
}
