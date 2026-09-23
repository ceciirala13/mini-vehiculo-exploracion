#include "uart_bridge.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "UART_BRIDGE_S3";
#define UART_PORT UART_NUM_1
#define UART_BUF_SIZE 4096
#define CAMERA_CHUNK_SIZE 1024

static uart_control_cb_t s_control_cb = NULL;
static uart_cam_cmd_cb_t s_cam_cmd_cb = NULL;
static SemaphoreHandle_t s_tx_mutex = NULL;
static uint32_t s_frame_counter = 0;

static uint8_t calc_checksum(uint8_t type, uint16_t len, const uint8_t *payload) {
    uint8_t csum = type ^ (uint8_t)(len & 0xFF) ^ (uint8_t)((len >> 8) & 0xFF);
    if (payload != NULL) {
        for (uint16_t i = 0; i < len; i++) {
            csum ^= payload[i];
        }
    }
    return csum;
}

static void uart_rx_task(void *pvParameters) {
    uint8_t rx_byte;
    typedef enum {
        STATE_WAIT_MAGIC_0,
        STATE_WAIT_MAGIC_1,
        STATE_GET_TYPE,
        STATE_GET_LEN_0,
        STATE_GET_LEN_1,
        STATE_GET_PAYLOAD,
        STATE_GET_CHECKSUM
    } rx_state_t;

    rx_state_t state = STATE_WAIT_MAGIC_0;
    uint8_t pkt_type = 0;
    uint16_t pkt_len = 0;
    uint16_t bytes_read = 0;
    static uint8_t payload_buf[512];

    ESP_LOGI(TAG, "Tarea de recepción UART iniciada.");

    while (true) {
        int len = uart_read_bytes(UART_PORT, &rx_byte, 1, portMAX_DELAY);
        if (len <= 0) continue;

        switch (state) {
            case STATE_WAIT_MAGIC_0:
                if (rx_byte == UART_PKT_MAGIC_0) {
                    state = STATE_WAIT_MAGIC_1;
                }
                break;

            case STATE_WAIT_MAGIC_1:
                if (rx_byte == UART_PKT_MAGIC_1) {
                    state = STATE_GET_TYPE;
                } else if (rx_byte == UART_PKT_MAGIC_0) {
                    state = STATE_WAIT_MAGIC_1;
                } else {
                    state = STATE_WAIT_MAGIC_0;
                }
                break;

            case STATE_GET_TYPE:
                pkt_type = rx_byte;
                state = STATE_GET_LEN_0;
                break;

            case STATE_GET_LEN_0:
                pkt_len = rx_byte;
                state = STATE_GET_LEN_1;
                break;

            case STATE_GET_LEN_1:
                pkt_len |= ((uint16_t)rx_byte << 8);
                if (pkt_len > sizeof(payload_buf)) {
                    ESP_LOGW(TAG, "Paquete UART descartado: longitud excesiva (%u bytes)", pkt_len);
                    state = STATE_WAIT_MAGIC_0;
                } else if (pkt_len == 0) {
                    state = STATE_GET_CHECKSUM;
                } else {
                    bytes_read = 0;
                    state = STATE_GET_PAYLOAD;
                }
                break;

            case STATE_GET_PAYLOAD:
                payload_buf[bytes_read++] = rx_byte;
                if (bytes_read >= pkt_len) {
                    state = STATE_GET_CHECKSUM;
                }
                break;

            case STATE_GET_CHECKSUM: {
                uint8_t expected_csum = calc_checksum(pkt_type, pkt_len, payload_buf);
                if (rx_byte == expected_csum) {
                    // Procesar paquete según tipo
                    if (pkt_type == PKT_TYPE_CONTROL && pkt_len == sizeof(s3_control_pkt_t)) {
                        s3_control_pkt_t *ctrl = (s3_control_pkt_t *)payload_buf;
                        if (s_control_cb != NULL) {
                            s_control_cb(ctrl->speed, ctrl->angle);
                        }
                    } else if (pkt_type == PKT_TYPE_CAM_COMMAND && pkt_len == sizeof(s3_cam_cmd_pkt_t)) {
                        s3_cam_cmd_pkt_t *cam_cmd = (s3_cam_cmd_pkt_t *)payload_buf;
                        if (s_cam_cmd_cb != NULL) {
                            s_cam_cmd_cb(cam_cmd->cmd, cam_cmd->param);
                        }
                    }
                } else {
                    ESP_LOGW(TAG, "Error de Checksum en paquete UART tipo 0x%02X (Rec: 0x%02X != Exp: 0x%02X)",
                             pkt_type, rx_byte, expected_csum);
                }
                state = STATE_WAIT_MAGIC_0;
                break;
            }
        }
    }
}

esp_err_t uart_bridge_init(gpio_num_t tx_pin, gpio_num_t rx_pin, uint32_t baud_rate,
                          uart_control_cb_t control_cb, uart_cam_cmd_cb_t cam_cmd_cb) {
    s_control_cb = control_cb;
    s_cam_cmd_cb = cam_cmd_cb;

    if (s_tx_mutex == NULL) {
        s_tx_mutex = xSemaphoreCreateMutex();
    }

    uart_config_t uart_config = {
        .baud_rate = (int)baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(UART_PORT, &uart_config);
    if (err != ESP_OK) return err;

    err = uart_set_pin(UART_PORT, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;

    err = uart_driver_install(UART_PORT, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 0, NULL, 0);
    if (err != ESP_OK) return err;

    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "UART1 inicializado correctamente en TX: GPIO %d, RX: GPIO %d a %lu baud",
             tx_pin, rx_pin, (unsigned long)baud_rate);

    return ESP_OK;
}

static esp_err_t send_packet_raw(uint8_t type, const uint8_t *payload, uint16_t len) {
    if (s_tx_mutex == NULL) return ESP_ERR_INVALID_STATE;

    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);

    uint8_t header[5];
    header[0] = UART_PKT_MAGIC_0;
    header[1] = UART_PKT_MAGIC_1;
    header[2] = type;
    header[3] = (uint8_t)(len & 0xFF);
    header[4] = (uint8_t)((len >> 8) & 0xFF);

    uint8_t csum = calc_checksum(type, len, payload);

    uart_write_bytes(UART_PORT, (const char *)header, sizeof(header));
    if (len > 0 && payload != NULL) {
        uart_write_bytes(UART_PORT, (const char *)payload, len);
    }
    uart_write_bytes(UART_PORT, (const char *)&csum, 1);

    xSemaphoreGive(s_tx_mutex);
    return ESP_OK;
}

esp_err_t uart_bridge_send_telemetry(float temp, float hum, float press, uint32_t gas,
                                     float v_motors, float pct_motors,
                                     float v_esp, float pct_esp,
                                     bool fault) {
    s3_telemetry_pkt_t pkt = {
        .temp = temp,
        .hum = hum,
        .press = press,
        .gas = gas,
        .v_motors = v_motors,
        .pct_motors = pct_motors,
        .v_esp = v_esp,
        .pct_esp = pct_esp,
        .fault = (uint8_t)(fault ? 1 : 0)
    };

    return send_packet_raw(PKT_TYPE_TELEMETRY, (const uint8_t *)&pkt, sizeof(pkt));
}

esp_err_t uart_bridge_send_camera_frame(const uint8_t *jpeg_buf, size_t jpeg_len) {
    if (jpeg_buf == NULL || jpeg_len == 0) return ESP_ERR_INVALID_ARG;

    s_frame_counter++;
    uint32_t current_frame_id = s_frame_counter;
    size_t offset = 0;

    static uint8_t chunk_payload[sizeof(s3_cam_chunk_hdr_t) + CAMERA_CHUNK_SIZE];

    while (offset < jpeg_len) {
        size_t chunk_len = jpeg_len - offset;
        if (chunk_len > CAMERA_CHUNK_SIZE) {
            chunk_len = CAMERA_CHUNK_SIZE;
        }

        s3_cam_chunk_hdr_t *hdr = (s3_cam_chunk_hdr_t *)chunk_payload;
        hdr->frame_id = current_frame_id;
        hdr->total_len = (uint32_t)jpeg_len;
        hdr->offset = (uint32_t)offset;
        hdr->chunk_len = (uint16_t)chunk_len;

        memcpy(chunk_payload + sizeof(s3_cam_chunk_hdr_t), jpeg_buf + offset, chunk_len);

        uint16_t total_payload_len = sizeof(s3_cam_chunk_hdr_t) + chunk_len;
        send_packet_raw(PKT_TYPE_CAMERA_CHUNK, chunk_payload, total_payload_len);

        offset += chunk_len;
        vTaskDelay(pdMS_TO_TICKS(1)); // Ceder brevemente para no saturar buffer
    }

    return ESP_OK;
}
