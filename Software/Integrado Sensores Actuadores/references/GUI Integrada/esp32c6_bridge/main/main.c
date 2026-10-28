// ==============================================================================
//  ROVER EXPLORER - FIRMWARE PUENTE DE RED Y TELEMETRÍA (ESP32-C6 NATIVO ESP-IDF)
//  - Wi-Fi SoftAP: SSID="Rover_WiFi_AP", Pass="rover1234" (IP: 192.168.4.1:8080)
//  - UART1 hacia ESP32-S3: TX=IO17 (a RX IO44 S3), RX=IO16 (a TX IO43 S3) @ 921600 baud
//  - IMU MPU6050: I2C (SDA=IO21, SCL=IO22 @ 400kHz) -> Telemetría 25 Hz hacia PC
// ==============================================================================

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "mpu6050_c6.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "ESP32C6_BRIDGE";

// ================= CONFIGURACIÓN HARDWARE =================
#define UART_BRIDGE_PORT      UART_NUM_1
#define UART_BRIDGE_TX_PIN    GPIO_NUM_17 // Conectar a RX (GPIO 44) de ESP32-S3
#define UART_BRIDGE_RX_PIN    GPIO_NUM_16 // Conectar a TX (GPIO 43) de ESP32-S3
#define UART_BRIDGE_BAUD      921600
#define UART_BUF_SIZE         4096

#define I2C_MASTER_PORT       I2C_NUM_0
#define I2C_MASTER_SDA_IO     GPIO_NUM_21
#define I2C_MASTER_SCL_IO     GPIO_NUM_22
#define I2C_MASTER_FREQ_HZ    400000
#define I2C_MASTER_TIMEOUT_MS 50

#define WIFI_AP_SSID          "Rover_WiFi_AP"
#define WIFI_AP_PASS          "rover1234"
#define WIFI_AP_CHANNEL       1
#define WIFI_AP_MAX_CONN      4
#define TCP_SERVER_PORT       8080

// ================= VARIABLES GLOBALES Y SEMÁFOROS =================
static int s_client_sock = -1;
static SemaphoreHandle_t s_socket_mutex = NULL;
static bool s_mpu_detected = false;

// ================= I2C / DRIVER MPU6050 =================
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_PORT, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
}

static esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t val) {
    uint8_t write_buf[2] = { reg, val };
    return i2c_master_write_to_device(I2C_MASTER_PORT, MPU6050_I2C_ADDR, write_buf, sizeof(write_buf), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

static esp_err_t mpu6050_read_bytes(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_PORT, MPU6050_I2C_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

static void mpu6050_init(void) {
    // 1. Despertar MPU6050 escribiendo 0 en PWR_MGMT_1 (0x6B)
    esp_err_t err = mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x00);
    if (err == ESP_OK) {
        s_mpu_detected = true;
        ESP_LOGI(TAG, "MPU6050 inicializado correctamente en I2C (SDA=IO%d, SCL=IO%d)", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    } else {
        s_mpu_detected = false;
        ESP_LOGW(TAG, "MPU6050 no detectado en I2C (Dirección 0x%02X). Verifique conexiones.", MPU6050_I2C_ADDR);
    }
}

static void mpu6050_read_data(imu_pkt_t *out_data) {
    if (!s_mpu_detected) {
        out_data->ax = 0.0f; out_data->ay = 0.0f; out_data->az = 1.0f;
        out_data->gx = 0.0f; out_data->gy = 0.0f; out_data->gz = 0.0f;
        out_data->roll = 0.0f; out_data->pitch = 0.0f;
        return;
    }

    uint8_t raw_buf[14];
    if (mpu6050_read_bytes(MPU6050_REG_ACCEL_XOUT, raw_buf, 14) == ESP_OK) {
        int16_t raw_ax = (int16_t)((raw_buf[0] << 8) | raw_buf[1]);
        int16_t raw_ay = (int16_t)((raw_buf[2] << 8) | raw_buf[3]);
        int16_t raw_az = (int16_t)((raw_buf[4] << 8) | raw_buf[5]);
        // Bytes 6 y 7 corresponden a temperatura interna
        int16_t raw_gx = (int16_t)((raw_buf[8] << 8) | raw_buf[9]);
        int16_t raw_gy = (int16_t)((raw_buf[10] << 8) | raw_buf[11]);
        int16_t raw_gz = (int16_t)((raw_buf[12] << 8) | raw_buf[13]);

        out_data->ax = (float)raw_ax / 16384.0f; // Escala ±2g
        out_data->ay = (float)raw_ay / 16384.0f;
        out_data->az = (float)raw_az / 16384.0f;
        out_data->gx = (float)raw_gx / 131.0f;   // Escala ±250°/s
        out_data->gy = (float)raw_gy / 131.0f;
        out_data->gz = (float)raw_gz / 131.0f;

        // Estimación de inclinación (Roll y Pitch)
        out_data->roll  = atan2f(out_data->ay, out_data->az) * (180.0f / (float)M_PI);
        out_data->pitch = atan2f(-out_data->ax, sqrtf(out_data->ay * out_data->ay + out_data->az * out_data->az)) * (180.0f / (float)M_PI);
    }
}

// ================= TRANSMISIÓN SEGURA POR SOCKET TCP =================
static int send_tcp_data(const void *data, size_t len) {
    if (s_socket_mutex == NULL) return -1;

    if (xSemaphoreTake(s_socket_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        int sock = s_client_sock;
        if (sock >= 0) {
            int sent = send(sock, data, len, 0);
            xSemaphoreGive(s_socket_mutex);
            return sent;
        }
        xSemaphoreGive(s_socket_mutex);
    }
    return -1;
}

static void send_imu_packet(const imu_pkt_t *imu) {
    uint16_t len = sizeof(imu_pkt_t);
    uint8_t packet[5 + sizeof(imu_pkt_t) + 1];

    packet[0] = PKT_MAGIC_0;
    packet[1] = PKT_MAGIC_1;
    packet[2] = PKT_TYPE_IMU;
    packet[3] = (uint8_t)(len & 0xFF);
    packet[4] = (uint8_t)((len >> 8) & 0xFF);

    uint8_t csum = PKT_TYPE_IMU ^ (uint8_t)(len & 0xFF) ^ (uint8_t)((len >> 8) & 0xFF);
    const uint8_t *payload = (const uint8_t *)imu;

    for (uint16_t i = 0; i < len; i++) {
        packet[5 + i] = payload[i];
        csum ^= payload[i];
    }
    packet[5 + len] = csum;

    send_tcp_data(packet, sizeof(packet));
}

// ================= TAREA: MUESTREO PERIÓDICO IMU MPU6050 (25 HZ) =================
static void imu_task(void *pvParameters) {
    imu_pkt_t imu_data;
    while (1) {
        if (s_client_sock >= 0) {
            mpu6050_read_data(&imu_data);
            send_imu_packet(&imu_data);
        }
        vTaskDelay(pdMS_TO_TICKS(40)); // 25 Hz
    }
}

// ================= TAREA: RETRANSMISIÓN UART -> TCP (ESP32-S3 -> PC) =================
static void uart_to_tcp_task(void *pvParameters) {
    uint8_t rx_buf[2048];
    while (1) {
        int bytes_read = uart_read_bytes(UART_BRIDGE_PORT, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(10));
        if (bytes_read > 0) {
            if (s_client_sock >= 0) {
                int sent = send_tcp_data(rx_buf, bytes_read);
                if (sent < 0) {
                    ESP_LOGW(TAG, "Error enviando flujo UART hacia TCP.");
                }
            }
        }
    }
}

// ================= TAREA: SERVIDOR TCP Y RETRANSMISIÓN TCP -> UART (PC -> S3) =================
static void tcp_server_task(void *pvParameters) {
    char rx_buf[1024];
    int listen_sock = -1;

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(TCP_SERVER_PORT);

    listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "No se pudo crear el socket TCP servidor: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Falla en bind del socket TCP: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Falla en listen del socket TCP: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Servidor TCP a la escucha en puerto %d", TCP_SERVER_PORT);

    while (1) {
        struct sockaddr_in source_addr;
        socklen_t addr_len = sizeof(source_addr);
        int client_sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (client_sock < 0) {
            ESP_LOGE(TAG, "Falla en accept: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Configuración de socket para baja latencia (TCP_NODELAY)
        int nodelay = 1;
        setsockopt(client_sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

        char addr_str[128];
        inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        ESP_LOGI(TAG, "Cliente PC conectado desde: %s", addr_str);

        xSemaphoreTake(s_socket_mutex, portMAX_DELAY);
        if (s_client_sock >= 0) {
            close(s_client_sock);
        }
        s_client_sock = client_sock;
        xSemaphoreGive(s_socket_mutex);

        // Bucle de recepción TCP -> UART (Comandos de control desde la PC hacia el ESP32-S3)
        while (1) {
            int len = recv(client_sock, rx_buf, sizeof(rx_buf), 0);
            if (len < 0) {
                ESP_LOGW(TAG, "Error en recv: errno %d", errno);
                break;
            } else if (len == 0) {
                ESP_LOGI(TAG, "Cliente PC desconectado.");
                break;
            } else {
                // Reenviar directamente al ESP32-S3 vía UART
                uart_write_bytes(UART_BRIDGE_PORT, rx_buf, len);
            }
        }

        // Cierre y limpieza de conexión
        xSemaphoreTake(s_socket_mutex, portMAX_DELAY);
        if (s_client_sock == client_sock) {
            s_client_sock = -1;
        }
        close(client_sock);
        xSemaphoreGive(s_socket_mutex);
    }
}

// ================= INICIALIZACIÓN WI-FI SOFTAP =================
static void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    // Configuración de IP estática por defecto (192.168.4.1)
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    esp_netif_dhcps_stop(ap_netif);
    esp_netif_set_ip_info(ap_netif, &ip_info);
    esp_netif_dhcps_start(ap_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = strlen(WIFI_AP_SSID),
            .channel = WIFI_AP_CHANNEL,
            .password = WIFI_AP_PASS,
            .max_connection = WIFI_AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    if (strlen(WIFI_AP_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi SoftAP '%s' iniciado. IP: 192.168.4.1", WIFI_AP_SSID);
}

// ================= INICIALIZACIÓN UART1 =================
static void uart_bridge_init_hw(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BRIDGE_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_BRIDGE_PORT, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_BRIDGE_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_BRIDGE_PORT, UART_BRIDGE_TX_PIN, UART_BRIDGE_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART1 configurada en TX=IO%d, RX=IO%d a %d baud", UART_BRIDGE_TX_PIN, UART_BRIDGE_RX_PIN, UART_BRIDGE_BAUD);
}

// ================= APP_MAIN =================
void app_main(void) {
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "  INICIANDO ESP32-C6 BRIDGE (ESP-IDF NATIVO)      ");
    ESP_LOGI(TAG, "==================================================");

    // 1. Inicializar NVS Flash
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Crear mutex de socket
    s_socket_mutex = xSemaphoreCreateMutex();

    // 3. Inicializar I2C y sensor MPU6050
    ESP_ERROR_CHECK(i2c_master_init());
    mpu6050_init();

    // 4. Inicializar UART1 hacia el ESP32-S3
    uart_bridge_init_hw();

    // 5. Iniciar Wi-Fi SoftAP
    wifi_init_softap();

    // 6. Lanzar tareas FreeRTOS
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
    xTaskCreate(uart_to_tcp_task, "uart2tcp", 4096, NULL, 5, NULL);
    xTaskCreate(imu_task, "imu_task", 3072, NULL, 4, NULL);

    ESP_LOGI(TAG, "Sistema de puente ESP32-C6 iniciado y listo.");
}
