# MCU_COMM — ESP32 DevKit 38-Pin Pinout

This document describes the complete pin mapping for the **ESP32 DevKit (38-pin version)** used as the communications board in the Rover Explorer.

---

## 1. Board Pinout Reference

![ESP32 DevKit 38-pin Pinout](../References/esp32_devkit_38p.png)

---

## 2. Active Pin Allocation

| Peripheral | Signal | ESP32 GPIO | Board Position | Notes |
|---|---|:---:|---|---|
| **ArduCAM OV2640 (SPI)** 
  | CS | **5** | Right side, Row 10 | General purpose output |
| | SCK | **18** | Right side, Row 9 | **VSPI Native SCK** (8 MHz DMA) |
| | MISO | **19** | Right side, Row 8 | **VSPI Native MISO** |
| | MOSI | **23** | Right side, Row 2 | **VSPI Native MOSI** |
| **ArduCAM OV2640 (I²C / SCCB)** | SDA | **21** | Right side, Row 6 | Default I²C SDA (100 kHz) |
| | SCL | **22** | Right side, Row 3 | Default I²C SCL (100 kHz) |
| **Camera Power** | VCC | **3.3V / 5V** | Power header | ArduCAM module power |
| | GND | **GND** | Right side, Row 1 or 7 | Power ground |
| **UART Link to ESP32-S3** | TX | **17** | Right side, Row 11 | **UART2 Native TX** → S3 GPIO 44 |
| | RX | **16** | Right side, Row 12 | **UART2 Native RX** ← S3 GPIO 43 |
| | GND | **GND** | Header | Common logic ground with S3 |
| **Debug / Flashing Console** | TXD | **1** | Right side, Row 4 | On-board CP2102 USB |
| | RXD | **3** | Right side, Row 5 | On-board CP2102 USB |

---

## 3. Physical Layout Map (38-Pin DevKit)

```
             LEFT SIDE                                 RIGHT SIDE
   ┌───────────────────────────┐             ┌───────────────────────────┐
   │ 3V3       - Power 3.3V    │             │ GND       - Ground        │
   │ EN        - Reset button  │             │ G23       - CAM MOSI      │
   │ G36 (SP)  - Free (input)  │             │ G22       - CAM SCL       │
   │ G39 (SN)  - Free (input)  │             │ TXD (G1)  - USB TX (CP2102)
   │ G34       - Free (input)  │             │ RXD (G3)  - USB RX (CP2102)
   │ G35       - Free (input)  │             │ G21       - CAM SDA       │
   │ G32       - Free (I/O)    │             │ GND       - Ground        │
   │ G33       - Free (I/O)    │             │ G19       - CAM MISO      │
   │ G25       - Free (I/O)    │             │ G18       - CAM SCK       │
   │ G26       - Free (I/O)    │             │ G5        - CAM CS        │
   │ G27       - Free (I/O)    │             │ G17       - LINK UART TX  │
   │ G14       - Free (I/O)    │             │ G16       - LINK UART RX  │
   │ G12       - Free (strapp) │             │ G4        - Free (I/O)    │
   │ GND       - Ground        │             │ G0        - Boot button   │
   │ G13       - Free (I/O)    │             │ G2        - Free (LED)    │
   │ SD2 (G9)  - SPI Flash BUS │             │ G15       - Free (strapp) │
   │ SD3 (G10) - SPI Flash BUS │             │ SD1 (G8)  - SPI Flash BUS │
   │ CMD (G11) - SPI Flash BUS │             │ SD0 (G7)  - SPI Flash BUS │
   │ 5V        - USB 5V VBUS   │             │ CLK (G6)  - SPI Flash BUS │
   └───────────────────────────┘             └───────────────────────────┘
```

> [!TIP]
> Notice that **all active rover connections (Camera SPI/I²C, UART link)** are concentrated on the **right side** of the board. This keeps cabling tidy and leaves the entire **left side** unobstructed for future peripherals.

---

## 4. GPIO Rules & Restrictions

1. **Internal SPI Flash (GPIOs 6–11):** Connected directly to the on-module SPI flash. **Never connect or reconfigure these pins.**
2. **Input-Only Pins (GPIOs 34, 35, 36, 39):** These pins have no internal pull-up or pull-down resistors and cannot be used as outputs. They are ideal for analog sensors (ADC1) or external signals with hardware pull-ups.
3. **Strapping Pins:**
   - **GPIO 0:** Pulls high by default. Kept low to enter bootloader mode.
   - **GPIO 2:** Connected to on-board LED on many DevKits. Must be floating or low at boot.
   - **GPIO 12:** Controls flash voltage (MTDI). Leave floating or low at boot.
   - **GPIO 15:** Silences boot log if pulled low.
