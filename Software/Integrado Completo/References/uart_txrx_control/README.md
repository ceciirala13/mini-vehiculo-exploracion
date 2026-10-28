# UART Control Node (ESP32-S3)

This project configures the **ESP32-S3** as the control node (`uart_txrx_control`) communicating asynchronously with the **ESP32** communication node (`uart_txrx_comm`) using `UART_NUM_1`.

## Pin Configuration (ESP32-S3)

| ESP32-S3 Signal | Pin | Destination on ESP32 |
|---|---|---|
| **TX** | **GPIO 17** | **GPIO 16 (RX)** |
| **RX** | **GPIO 18** | **GPIO 17 (TX)** |
| **GND** | **GND** | **GND (Common Ground)** |

> **IMPORTANT**: Both boards must share a common ground (GND) for UART signal levels to be stable.

## How it Works

1. **`tx_task`**: Sends `HELLO_FROM_ESP32S3_CTRL #<count>` periodically (every 2 seconds, with an initial 1-second delay so logs interleave smoothly) over UART1 at 115200 baud.
2. **`rx_task`**: Listens asynchronously for incoming messages from the ESP32 (`HELLO_FROM_ESP32_COMM #<count>`), printing a formatted single-line log with packet counter and byte count.

## How to Build and Flash

In your terminal or ESP-IDF VS Code extension for `uart_txrx_control`:

```bash
# Set target to esp32s3 (if reconfiguring from esp32)
idf.py set-target esp32s3

# Build, flash, and open monitor (replace COM_PORT with your ESP32-S3 port)
idf.py -p COM_PORT flash monitor
```

To exit monitor: `Ctrl + ]`

## Expected Serial Monitor Output

```
I (...) CTRL_ESP32S3: UART_NUM_1 initialized (TX: GPIO 17, RX: GPIO 18, Baud: 115200)
I (...) CTRL_ESP32S3: <<< [RX #1] Recv from COMM (ESP32): "HELLO_FROM_ESP32_COMM #1" (28 bytes)
I (...) CTRL_ESP32S3: >>> [TX #1] Sent to COMM (ESP32): "HELLO_FROM_ESP32S3_CTRL #1" (29 bytes)
I (...) CTRL_ESP32S3: <<< [RX #2] Recv from COMM (ESP32): "HELLO_FROM_ESP32_COMM #2" (28 bytes)
I (...) CTRL_ESP32S3: >>> [TX #2] Sent to COMM (ESP32): "HELLO_FROM_ESP32S3_CTRL #2" (29 bytes)
```
