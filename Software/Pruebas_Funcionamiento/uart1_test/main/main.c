#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_18)

static const int RX_BUF_SIZE = 1024;
static const char *TAG = "UART_TEST";

void init_uart(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void tx_task(void *arg) {
    int counter = 0;
    while (1) {
        char data[32];
        int len = snprintf(data, sizeof(data), "Test data: %d\r\n", counter++);
        
        // Send data
        uart_write_bytes(UART_NUM_1, data, len);
        ESP_LOGI(TAG, "Sent: %s", data);
        
        // Wait for 10ms to achieve 100Hz frequency
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    init_uart();
    xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES - 1, NULL);
}
