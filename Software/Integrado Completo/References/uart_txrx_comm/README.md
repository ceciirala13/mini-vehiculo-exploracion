# UART Communication Node (ESP32)

This project configures the **ESP32** as the communication node (`uart_txrx_comm`) communicating asynchronously with the **ESP32-S3** control node (`uart_txrx_control`) using `UART_NUM_1`.

## Pin Configuration (ESP32)

| ESP32 Signal | Pin | Destination on ESP32-S3 |
|---|---|---|
| **TX** | **GPIO 17** | **GPIO 18 (RX)** |
| **RX** | **GPIO 16** | **GPIO 17 (TX)** |
| **GND** | **GND** | **GND (Common Ground)** |

> **IMPORTANT**: Both boards must share a common ground (GND) for UART signal levels to be stable.

## How it Works

1. **`tx_task`**: Sends `HELLO_FROM_ESP32_COMM #<count>` periodically (every 2 seconds) over UART1 at 115200 baud.
2. **`rx_task`**: Listens asynchronously for incoming messages from the ESP32-S3 (`HELLO_FROM_ESP32S3_CTRL #<count>`), printing a formatted single-line log with packet counter and byte count.

## How to Build and Flash

In your terminal or ESP-IDF VS Code extension for `uart_txrx_comm`:

```bash
# Set target to esp32 (if reconfiguring from esp32s3)
idf.py set-target esp32

# Build, flash, and open monitor (replace COM_PORT with your ESP32 port)
idf.py -p COM_PORT flash monitor
```

To exit monitor: `Ctrl + ]`

## Expected Serial Monitor Output

```
I (...) COMM_ESP32: UART_NUM_1 initialized (TX: GPIO 17, RX: GPIO 16, Baud: 115200)
I (...) COMM_ESP32: >>> [TX #1] Sent to CTRL (ESP32-S3): "HELLO_FROM_ESP32_COMM #1" (28 bytes)
I (...) COMM_ESP32: <<< [RX #1] Recv from CTRL (ESP32-S3): "HELLO_FROM_ESP32S3_CTRL #1" (29 bytes)
I (...) COMM_ESP32: >>> [TX #2] Sent to CTRL (ESP32-S3): "HELLO_FROM_ESP32_COMM #2" (28 bytes)
I (...) COMM_ESP32: <<< [RX #2] Recv from CTRL (ESP32-S3): "HELLO_FROM_ESP32S3_CTRL #2" (29 bytes)
```
