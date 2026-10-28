/* UART asynchronous inter-board communication test - COMM Node (ESP32)
 *
 * Configured Pins for ESP32:
 *   GPIO 17: TX (Connect to ESP32-S3 RX - GPIO 44)
 *   GPIO 16: RX (Connect to ESP32-S3 TX - GPIO 43)
 *   GND:     Connect to ESP32-S3 GND
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

static const int RX_BUF_SIZE = 1024;
static const char *TAG = "COMM_ESP32";

#ifndef CONFIG_EXAMPLE_UART_TXD
#define TXD_PIN (GPIO_NUM_17)
#else
#define TXD_PIN (CONFIG_EXAMPLE_UART_TXD)
#endif

#ifndef CONFIG_EXAMPLE_UART_RXD
#define RXD_PIN (GPIO_NUM_16)
#else
#define RXD_PIN (CONFIG_EXAMPLE_UART_RXD)
#endif

#ifndef CONFIG_EXAMPLE_UART_BAUD_RATE
#define UART_BAUD_RATE (115200)
#else
#define UART_BAUD_RATE (CONFIG_EXAMPLE_UART_BAUD_RATE)
#endif

#ifndef CONFIG_EXAMPLE_TASK_STACK_SIZE
#define TASK_STACK_SIZE (3072)
#else
#define TASK_STACK_SIZE (CONFIG_EXAMPLE_TASK_STACK_SIZE)
#endif

void init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // Keep RX idle-high if the wire is loose, so a floating pin doesn't read as a stream of 0x00
    gpio_set_pull_mode(RXD_PIN, GPIO_PULLUP_ONLY);
    ESP_LOGI(TAG, "UART_NUM_1 initialized (TX: GPIO %d, RX: GPIO %d, Baud: %d)", TXD_PIN, RXD_PIN, UART_BAUD_RATE);
}

static void tx_task(void *arg)
{
    uint32_t tx_counter = 0;
    char payload[64];
    char tx_wire[80];

    while (1) {
        ++tx_counter;
        snprintf(payload, sizeof(payload), "HELLO_FROM_ESP32_COMM #%lu", (unsigned long)tx_counter);
        snprintf(tx_wire, sizeof(tx_wire), "%s\n", payload);

        int tx_bytes = uart_write_bytes(UART_NUM_1, tx_wire, strlen(tx_wire));
        ESP_LOGI(TAG, ">>> [TX #%lu] Sent to CTRL (ESP32-S3): \"%s\" (%d bytes)",
                 (unsigned long)tx_counter, payload, tx_bytes);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void rx_task(void *arg)
{
    uint32_t rx_counter = 0;
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);

    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, pdMS_TO_TICKS(100));
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            // Trim trailing newlines and carriage returns for clean single-line logging
            int len = rxBytes;
            while (len > 0 && (data[len - 1] == '\n' || data[len - 1] == '\r')) {
                data[--len] = 0;
            }
            if (len > 0) {
                ++rx_counter;
                ESP_LOGI(TAG, "<<< [RX #%lu] Recv from CTRL (ESP32-S3): \"%s\" (%d bytes)",
                         (unsigned long)rx_counter, data, rxBytes);
                // Raw bytes: readable text = link OK, 00/FF/random = wiring, GND or level problem
                ESP_LOG_BUFFER_HEXDUMP(TAG, data, rxBytes > 32 ? 32 : rxBytes, ESP_LOG_INFO);
            }
        }
    }
    free(data);
}

void app_main(void)
{
    init();
    xTaskCreate(rx_task, "uart_rx_task", TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(tx_task, "uart_tx_task", TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 2, NULL);
}
