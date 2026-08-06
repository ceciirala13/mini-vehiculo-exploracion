# 7Semi BNO08x Arduino Library

Arduino library for the **7Semi BNO08x IMU sensor** (BNO080/BNO085) with **I2C, SPI, and UART support**. Works over **I²C, SPI, and UART**, enabling common **SH-2 sensor reports** and providing **cached sensor data** for non-blocking reads.  
Tiny footprint — suitable for **AVR, ESP32, and ESP8266** boards.

---

## ✨ Features

✅ Supports **I²C**, **SPI**, and **UART** transports  
✅ Supports common SH-2 sensor reports:

- Accelerometer  
- Gyroscope  
- Magnetometer  
- Linear Acceleration  
- Gravity  
- Rotation Vector  
- Game Rotation Vector  
- Geomagnetic Rotation Vector  

✅ Simple API:
- `enableReport(reportId, intervalMs)`
- `processData()` inside `loop()`

✅ Non-blocking driver using `processData()`  
✅ Cached getters:
- `getAccelerometer()`
- `getGyroscope()`
- `getQuaternion()`


---
## ⚡ Getting Started

### 🔌 Hardware Connections

---

### ✅ I²C

| BNO08x | Arduino |
|-------|---------|
| VIN   | 3.3V *(or 5V if supported)* |
| GND   | GND |
| SDA   | SDA *(A4 on UNO)* |
| SCL   | SCL *(A5 on UNO)* |

📌 Default address is `0x4B` *(some breakouts use `0x4A`)*.

---

### ✅ SPI

| BNO08x | Arduino |
|-------|---------|
| VIN   | 3.3V |
| GND   | GND |
| SCK   | SCK *(13)* |
| MOSI  | MOSI *(12)* |
| MISO  | MISO *(11)* |
| CS    | Any digital pin *(ex: 10)* |
| INTN  | Optional *(LOW = data ready)* |
| RST   | Optional |

📌 `INTN` is **recommended** for efficient SPI polling.  
📌 `RST` is optional but useful for resetting the sensor.

---

### ✅ UART (for framed SHTP breakouts)

| BNO08x | Arduino |
|-------|---------|
| VIN   | 3.3V |
| GND   | GND |
| TX(MISO)    | RX *(Serial1 RX)* |
| RX(SCL)    | TX *(Serial1 TX)* |
| INTN  | Optional |

---

| Interface Mode           | PS1 | PS0 | Notes                                   |
| ------------------------ | --- | --- | --------------------------------------- |
| **I²C**                  | 0   | 0   | Default mode on most breakouts          |
| **Reserved / Undefined** | 0   | 1   | Not recommended / not used              |
| **UART**                 | 1   | 0   | Uses TX(MISO)/RX(SCL)                   |
| **SPI**                  | 1   | 1   | Requires CS + SCK + MOSI + MISO         |



---

## 📦 Installation

1. Download this repository as a **ZIP**
2. In Arduino IDE:  
   **Sketch → Include Library → Add .ZIP Library…**
3. Select the downloaded ZIP file

---

## 📁 Library Files

Core driver:

- `7Semi_BNO08x.h`
- `7Semi_BNO08x.cpp`

Transport interface:

- `BnoBus.h`

---

