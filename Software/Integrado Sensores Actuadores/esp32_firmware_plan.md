# ESP-IDF Modular Firmware Implementation Plan (ESP32-S3 WROOM)

## 1. Executive Summary & Architecture Overview
This plan establishes a production-grade, modular C firmware architecture for a custom **ESP32-S3 WROOM** exploration vehicle. The firmware integrates:
- **Two Independent I2C Master Buses** (`driver/i2c_master.h`):
  - **I2C Bus 0** (`I2C_NUM_0`): Dedicated to **MPU6050** (6-axis IMU) on SDA=GPIO 40, SCL=GPIO 41.
  - **I2C Bus 1** (`I2C_NUM_1`): Dedicated to **BME690** (Bosch SensorAPI: Temp, Press, Hum, Gas) on SDA=GPIO 5, SCL=GPIO 6.
- **LEDC PWM** controller for precision positioning of a Servo motor (0°, 90°, 180° at 50 Hz on GPIO 39).
- **MCPWM** dual-motor driver for dual DC motors with forward/reverse/brake and hardware `nFAULT` detection (using DRV8873H / H-bridge drivers on GPIO 7, 12, 21 & 14, 8, 38).
- **UART Telemetry Engine** streaming structured binary sensor telemetry packets to a companion microcontroller on UART_NUM_1 (TX=GPIO 17, RX=GPIO 18).
- Multi-mode preprocessor execution with **`MODE_TESTING`** as the active operational mode, and architectural stubs for future camera/Wi-Fi modes.

---

## 2. Hardware Mapping & Pin Assignments (`project_config.h`)

| Peripheral / Module | Signal / Function | ESP32-S3 GPIO | Bus / Peripheral Identifier |
| :--- | :--- | :--- | :--- |
| **MPU6050 I2C Bus** | `SDA` | **GPIO 40** | `I2C_NUM_0` (400 kHz) |
| | `SCL` | **GPIO 41** | `I2C_NUM_0` |
| | Addr | `0x68` | AD0 to GND |
| **BME690 I2C Bus** | `SDA` | **GPIO 5** | `I2C_NUM_1` (400 kHz) |
| | `SCL` | **GPIO 6** | `I2C_NUM_1` |
| | Addr | `0x76` | SDO to GND (Default) |
| **LEDC Servo** | `SERVO_PWM` | **GPIO 39** | `LEDC_TIMER_0`, `LEDC_CHANNEL_0` (50 Hz, 13-bit) |
| **MCPWM Motor 1** | `M1_IN1` | **GPIO 7** | MCPWM Group 0, Op 0 |
| | `M1_IN2` | **GPIO 12** | MCPWM Group 0, Op 0 |
| | `M1_NFAULT` | **GPIO 21** | Input with pull-up |
| **MCPWM Motor 2** | `M2_IN1` | **GPIO 14** | MCPWM Group 0, Op 1 |
| | `M2_IN2` | **GPIO 8** | MCPWM Group 0, Op 1 |
| | `M2_NFAULT` | **GPIO 38** | Input with pull-up |
| **UART Telemetry** | `UART_TX` | **GPIO 17** | `UART_NUM_1` (115200 Baud) |
| | `UART_RX` | **GPIO 18** | `UART_NUM_1` |
