/**
 * @file main.c
 * @brief ESP32-C6 Wi-Fi AP HTTP Server with ArduCAM OV2640 2MP Plus
 */

#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "arducam_ov2640.h"

static const char *TAG = "rover_camera_server";

// ================= CONFIGURACIÓN WI-FI AP =================
#define WIFI_AP_SSID      "ROVER_EXPLORER_AP"
#define WIFI_AP_PASS      "rover1234"
#define WIFI_AP_CHANNEL   1
#define WIFI_MAX_STA_CONN 4

// ================= CONFIGURACIÓN PINES ARDUCAM =================
#define ARDUCAM_PIN_MISO  GPIO_NUM_4
#define ARDUCAM_PIN_MOSI  GPIO_NUM_5
#define ARDUCAM_PIN_SCK   GPIO_NUM_6
#define ARDUCAM_PIN_CS    GPIO_NUM_7

#define ARDUCAM_PIN_SDA   GPIO_NUM_8
#define ARDUCAM_PIN_SCL   GPIO_NUM_9

static SemaphoreHandle_t s_camera_mutex = NULL;
static httpd_handle_t s_server = NULL;

// Referencia a index.html incrustado en el binario
extern const unsigned char index_html_start[] asm("_binary_index_html_start");
extern const unsigned char index_html_end[]   asm("_binary_index_html_end");

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";


// ================= MANEJADOR: GET / (Panel de Control Web) =================
static esp_err_t index_get_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Sirviendo index.html");
    const size_t index_html_len = index_html_end - index_html_start;
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "identity");
    return httpd_resp_send(req, (const char *)index_html_start, index_html_len);
}

// ================= MANEJADOR: GET /capture (Foto JPEG) =================
static esp_err_t capture_get_handler(httpd_req_t *req) {
    if (xSemaphoreTake(s_camera_mutex, pdMS_TO_TICKS(3000)) != pdTRUE) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Disparando captura de imagen...");
    uint8_t *jpeg_buf = NULL;
    size_t jpeg_len = 0;
    esp_err_t ret = arducam_capture_frame(&jpeg_buf, &jpeg_len);
    xSemaphoreGive(s_camera_mutex);

    if (ret != ESP_OK || !jpeg_buf || jpeg_len == 0) {
        ESP_LOGE(TAG, "Fallo al capturar frame: %s", esp_err_to_name(ret));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Transmitiendo imagen JPEG válida: %u bytes", (unsigned int)jpeg_len);

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");

    ret = httpd_resp_send(req, (const char *)jpeg_buf, jpeg_len);
    free(jpeg_buf);
    return ret;
}

// ================= MANEJADOR: GET /stream (MJPEG Live Stream) =================
static esp_err_t stream_get_handler(httpd_req_t *req) {
    esp_err_t res = ESP_OK;
    char part_buf[64];

    httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    ESP_LOGI(TAG, "Iniciando streaming MJPEG");

    while (true) {
        if (xSemaphoreTake(s_camera_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        uint8_t *jpeg_buf = NULL;
        size_t jpeg_len = 0;
        esp_err_t ret = arducam_capture_frame(&jpeg_buf, &jpeg_len);
        xSemaphoreGive(s_camera_mutex);

        if (ret != ESP_OK || !jpeg_buf || jpeg_len == 0) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        if (res == ESP_OK) {
            size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, (unsigned int)jpeg_len);
            res = httpd_resp_send_chunk(req, part_buf, hlen);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)jpeg_buf, jpeg_len);
        }

        free(jpeg_buf);

        if (res != ESP_OK) {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(30)); // Control de tasa de cuadros
    }

    ESP_LOGI(TAG, "Streaming MJPEG finalizado");
    return res;
}

// ================= MANEJADOR: GET /resolution (Cambio de resolución) =================
static esp_err_t resolution_get_handler(httpd_req_t *req) {
    char buf[32];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char param[8];
        if (httpd_query_key_value(buf, "res", param, sizeof(param)) == ESP_OK) {
            int res_val = atoi(param);
            if (res_val >= 0 && res_val <= 8) {
                if (xSemaphoreTake(s_camera_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                    arducam_set_resolution((arducam_resolution_t)res_val);
                    xSemaphoreGive(s_camera_mutex);
                    ESP_LOGI(TAG, "Resolución cambiada a: %d", res_val);
                    httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
                    return ESP_OK;
                }
            }
        }
    }
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Parámetro inválido");
    return ESP_FAIL;
}

// ================= MANEJADOR: GET /status (Telemetría / Estado) =================
static esp_err_t status_get_handler(httpd_req_t *req) {
    const char *resp = "{\"sensor\":\"OV2640 2MP PLUS\",\"mode\":\"WiFi_AP\",\"ip\":\"192.168.4.1\"}";
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
}

// ================= INICIALIZAR SERVIDOR HTTP =================
static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8;
    config.stack_size = 8192;
    config.lru_purge_enable = true;

    httpd_handle_t server = NULL;
    ESP_LOGI(TAG, "Iniciando servidor HTTP en el puerto %d", config.server_port);

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uri_root = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = index_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_root);

        httpd_uri_t uri_capture = {
            .uri = "/capture",
            .method = HTTP_GET,
            .handler = capture_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_capture);

        httpd_uri_t uri_stream = {
            .uri = "/stream",
            .method = HTTP_GET,
            .handler = stream_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_stream);

        httpd_uri_t uri_res = {
            .uri = "/resolution",
            .method = HTTP_GET,
            .handler = resolution_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_res);

        httpd_uri_t uri_status = {
            .uri = "/status",
            .method = HTTP_GET,
            .handler = status_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_status);

        return server;
    }

    ESP_LOGE(TAG, "Error iniciando servidor HTTP!");
    return NULL;
}

// ================= INICIALIZAR WI-FI EN MODO ACCESS POINT =================
static void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = strlen(WIFI_AP_SSID),
            .channel = WIFI_AP_CHANNEL,
            .password = WIFI_AP_PASS,
            .max_connection = WIFI_MAX_STA_CONN,
            .authmode = (strlen(WIFI_AP_PASS) == 0) ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi AP iniciado. SSID: [%s] | Password: [%s] | IP: http://192.168.4.1",
             WIFI_AP_SSID, WIFI_AP_PASS);
}

// ================= PUNTO DE ENTRADA PRINCIPAL =================
void app_main(void) {
    // 1. Inicializar almacenamiento no volátil (NVS)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_camera_mutex = xSemaphoreCreateMutex();

    ESP_LOGI(TAG, "--- ROVER EXPLORER ESP32-C6 CAM SERVER ---");

    // 2. Inicializar ArduCAM OV2640
    arducam_config_t cam_cfg = {
        .pin_cs = ARDUCAM_PIN_CS,
        .pin_mosi = ARDUCAM_PIN_MOSI,
        .pin_miso = ARDUCAM_PIN_MISO,
        .pin_sck = ARDUCAM_PIN_SCK,
        .pin_sda = ARDUCAM_PIN_SDA,
        .pin_scl = ARDUCAM_PIN_SCL,
        .spi_freq_hz = 8000000,  // 8 MHz SPI Clock
        .i2c_freq_hz = 100000,  // 100 kHz I2C Clock
    };

    ret = arducam_init(&cam_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando hardware ArduCAM. Verifica las conexiones.");
    } else {
        ESP_LOGI(TAG, "Hardware ArduCAM OV2640 inicializado con éxito.");
    }

    // 3. Iniciar red Wi-Fi en modo Punto de Acceso (AP)
    wifi_init_softap();

    // 4. Iniciar Servidor HTTP
    s_server = start_webserver();
    if (s_server != NULL) {
        ESP_LOGI(TAG, "Servidor web activo. Conéctate a la red '%s' y abre http://192.168.4.1", WIFI_AP_SSID);
    }
}
