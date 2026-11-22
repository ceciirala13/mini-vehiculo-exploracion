# MCU_CONTROL — ESP32-S3 Custom Board Pinout

This document describes the complete pin mapping for the **ESP32-S3-WROOM-1** custom controller board used in the Rover Explorer.

---

## 1. Schematic Reference

![ESP32-S3 Schematic](../References/Control_MCU_esp32s3_pinout.png)

---

## 2. Subsystem Pin Allocation

| Subsystem | Signal | ESP32-S3 GPIO | Module Pin | Hardware Header / Device | Notes |
|---|---|:---:|:---:|---|---|
| **Motors (DRV8873H #1 - Left)** | M1_IN1 | **7** | 7 | Driver 1 IN1 | MCPWM forward/reverse |
| | M1_IN2 | **8** | 12 | Driver 1 IN2 | MCPWM forward/reverse |
| | M1_DISABLE | **9** | 17 | Driver 1 DISABLE | Active HIGH (Hi-Z output) |
| | M1_nFAULT | **21** | 23 | Driver 1 nFAULT | Active LOW fault flag |
| **Motors (DRV8873H #2 - Right)**| M2_IN1 | **14** | 22 | Driver 2 IN1 | MCPWM forward/reverse |
| | M2_IN2 | **15** | 8 | Driver 2 IN2 | MCPWM forward/reverse |
| | M2_DISABLE | **16** | 9 | Driver 2 DISABLE | Active HIGH (Hi-Z output) |
| | M2_nFAULT | **38** | 31 | Driver 2 nFAULT | Active LOW fault flag |
| **Steering Servo** | PWM | **39** | 32 | Servo header | LEDC PWM 50 Hz |
| **IMU (MPU6050 - Bus 0)** | SDA | **40** | 33 | AUX1 header | 400 kHz I²C |
| | SCL | **41** | 34 | AUX2 header | 400 kHz I²C |
| **Environment (BME690 - Bus 1)**| SDA | **5** | 5 | I2C1 header | 400 kHz I²C |
| | SCL | **6** | 6 | I2C1 header | 400 kHz I²C |
| **Wheel Encoders** | ENC_LEFT_A | **10** | 18 | 8-Pin Header Pin 1 | Hardware PCNT |
| | ENC_LEFT_B | **11** | 19 | 8-Pin Header Pin 2 | Hardware PCNT |
| | ENC_RIGHT_A | **13** | 21 | 8-Pin Header Pin 3 | Hardware PCNT |
| | ENC_RIGHT_B | **12** | 20 | 8-Pin Header Pin 4 | Hardware PCNT |
| **Auxiliary Header Expansion** | SPARE_SDA | **47** | 24 | 8-Pin Header Pin 7 | General I/O, I²C, ADC |
| | SPARE_SCL | **48** | 25 | 8-Pin Header Pin 8 | General I/O, I²C, ADC |
| **UART Link to MCU_COMM** | TX (TX_RC) | **43** | 37 (TXD0) | RC header TX pad | → ESP32 RX (GPIO 16) |
| | RX (RX_RC) | **44** | 36 (RXD0) | RC header RX pad | ← ESP32 TX (GPIO 17) |
| **UART BNO (Auxiliary)** | TX_1 | **17** | 10 | BNO header | Optional external IMU |
| | RX_1 | **18** | 11 | BNO header | Optional external IMU |
| **Battery Monitoring** | ADC_BAT | **2** | 38 | Voltage Divider 1 | Analog input (ADC1_CH1) |
| | ADC_BAT2 | **42** | 35 | Voltage Divider 2 | Analog input (ADC1_CH9) |
| **System** | EN | **EN** | 3 | Reset Circuit / Button | — |
| | BOOT | **0** | 27 | ESP_IO0 / Boot Button | Strapping pin (Pull-up) |
| | RST_BTTN | **4** | 4 | Button | User input |
| | BNO_INT | **1** | 39 | Interrupt pad | External interrupt |
| | USB D- | **USB_D-** | 13 | USB-Serial-JTAG | Flashing & debug console |
| | USB D+ | **USB_D+** | 14 | USB-Serial-JTAG | Flashing & debug console |

---

## 3. 8-Pin Encoder & Auxiliary Expansion Header (Encoder Wiring)

The custom board has an 8-pin inline connector that serves as the dedicated interface for both motor quadrature encoders and auxiliary expansion:

```
                      8-PIN ENCODER / EXPANSION CONNECTOR
 ┌──────┬─────────────────────────┬─────────────┬───────────────────────────────────────┐
 │ Pin  │ Pin / Net Designation   │ ESP32-S3 IO │ Assigned Function                     │
 ├──────┼─────────────────────────┼─────────────┼───────────────────────────────────────┤
 │  1   │ Pin 1 (CS_cam net)      │ GPIO 10     │ Left Wheel Encoder Phase A            │
 │  2   │ Pin 2 (MOSI_cam net)    │ GPIO 11     │ Left Wheel Encoder Phase B            │
 │  3   │ Pin 3 (MISO_cam net)    │ GPIO 13     │ Right Wheel Encoder Phase A           │
 │  4   │ Pin 4 (SCK_cam net)     │ GPIO 12     │ Right Wheel Encoder Phase B           │
 │  5   │ GND                     │ GND         │ Common GND for encoders               │
 │  6   │ +3.3V                   │ 3.3V        │ Common +3.3V for encoders             │
 │  7   │ Pin 7 (SDA_cam net)     │ GPIO 47     │ Spare GPIO / Auxiliary Expansion      │
 │  8   │ Pin 8 (SCL_cam net)     │ GPIO 48     │ Spare GPIO / Auxiliary Expansion      │
 └──────┴─────────────────────────┴─────────────┴───────────────────────────────────────┘
```

### Physical Header Connection

- **Left Motor Encoder Cable:**
  - Signal A → Pin 1 (`GPIO 10`)
  - Signal B → Pin 2 (`GPIO 11`)
  - GND → Pin 5 (`GND`)
  - VCC → Pin 6 (`+3.3V`)
- **Right Motor Encoder Cable:**
  - Signal A → Pin 3 (`GPIO 13`)
  - Signal B → Pin 4 (`GPIO 12`)
  - GND → Pin 5 (`GND`)
  - VCC → Pin 6 (`+3.3V`)

> [!NOTE]
> On the ESP32-S3, GPIOs 10, 11, 12, and 13 support internal pull-up/pull-down resistors, edge interrupts, and can be mapped directly to hardware **PCNT** (Pulse Counter) units for hardware quadrature decoding.

---

## 4. Internal / Reserved Pins on ESP32-S3

The following pins are reserved internally on the ESP32-S3 module and must **never** be used:
- **GPIO 35, 36, 37:** Connected to internal Octal/Quad PSRAM.
- **GPIO 45:** Strapping pin for `VDD_SPI` flash voltage.
- **GPIO 46:** Strapping pin for ROM message printing.
- **GPIO 3:** JTAG source strapping.
