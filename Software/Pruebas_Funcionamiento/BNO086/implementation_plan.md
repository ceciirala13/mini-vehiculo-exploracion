# BNO086 UART-RVC Driver for ESP32-S3 (ESP-IDF)

## Background

The hardware schematic shows **PS1 = GND, PS0 = 3V3**, which selects **UART-RVC mode** (Robot Vacuum Cleaner mode) — NOT the SHTP-UART mode used in the 7Semi reference library. This significantly simplifies the driver since:

- UART-RVC outputs a **fixed 19-byte packet** at **100 Hz** automatically — no need to send enable commands
- Baud rate is **115200** (fixed by the RVC protocol)
- No SHTP framing, HDLC escaping, or SH-2 command negotiation required

> [!IMPORTANT]
> The reference library (`7Semi-BNO08x`) implements **SHTP-over-UART** with HDLC framing, which is a **different** protocol mode (PS1=HIGH, PS0=LOW). Our hardware uses RVC mode, so the implementation will be much simpler — a direct stream parser rather than the SHTP state machine.

## Hardware Mapping (from schematics)

| Signal | ESP32-S3 GPIO | BNO086 Pin | Notes |
|--------|--------------|------------|-------|
| UART TX | GPIO 18 | H_SCL/SCK/RX (pin 19) | ESP TX → BNO RX |
| UART RX | GPIO 17 | H_SDA/H_MISO/TX (pin 20) | BNO TX → ESP RX |
| Reset | GPIO 4 | NRST (pin 11) | Active low, pulled up via R7 |
| Interrupt | GPIO 0 | H_INTN (pin 14) | Not required for RVC mode |
| PS1 | — | GND | Selects UART-RVC |
| PS0 | — | 3V3 (via R7 10k) | Selects UART-RVC |

## UART-RVC Packet Structure (19 bytes)

| Byte(s) | Field | Format |
|---------|-------|--------|
| 0–1 | Header | `0xAA 0xAA` |
| 2 | Index | uint8, monotonic counter 0–255 |
| 3–4 | Yaw | int16 LE, × 0.01° |
| 5–6 | Pitch | int16 LE, × 0.01° |
| 7–8 | Roll | int16 LE, × 0.01° |
| 9–10 | Accel X | int16 LE, in milli-g (×0.0098067 → m/s²) |
| 11–12 | Accel Y | int16 LE, in milli-g |
| 13–14 | Accel Z | int16 LE, in milli-g |
| 15 | MI (Motion Intent) | uint8 |
| 16 | MR (Motion Request) | uint8 |
| 17 | Reserved | uint8 |
| 18 | Checksum | uint8 — sum of bytes 2..17 mod 256 |

## Proposed Changes

### Main Component

#### [MODIFY] [CMakeLists.txt](file:///c:/Users/sanie/Documents/GitHub/mini-vehiculo-exploracion/Software/Pruebas_Funcionamiento/BNO086/bno086_test/main/CMakeLists.txt)
- Add `bno086_rvc.c` to `SRCS`

#### [NEW] [bno086_rvc.h](file:///c:/Users/sanie/Documents/GitHub/mini-vehiculo-exploracion/Software/Pruebas_Funcionamiento/BNO086/bno086_test/main/bno086_rvc.h)
Header with:
- `bno086_rvc_data_t` struct (yaw, pitch, roll in degrees + accel x/y/z in m/s²)
- `bno086_rvc_init()` — configures GPIO 4 reset + UART1 on GPIO 17/18
- `bno086_rvc_read()` — blocking read of one valid 19-byte packet with checksum validation
- Pin/baud defines

#### [NEW] [bno086_rvc.c](file:///c:/Users/sanie/Documents/GitHub/mini-vehiculo-exploracion/Software/Pruebas_Funcionamiento/BNO086/bno086_test/main/bno086_rvc.c)
Implementation:
1. **Reset sequence**: GPIO 4 → OUTPUT, drive HIGH 10ms, LOW 20ms, HIGH, then wait 300ms for BNO boot
2. **UART config**: UART_NUM_1, 115200 baud, 8N1, RX=GPIO17, TX=GPIO18, 1024-byte RX ring buffer
3. **Read function**: Scans incoming UART stream byte-by-byte looking for `0xAA 0xAA` header, reads remaining 17 bytes, validates checksum (sum of bytes 2–17 mod 256 == byte 18), extracts and scales values

#### [MODIFY] [main.c](file:///c:/Users/sanie/Documents/GitHub/mini-vehiculo-exploracion/Software/Pruebas_Funcionamiento/BNO086/bno086_test/main/main.c)
- Call `bno086_rvc_init()` on startup
- Create a FreeRTOS task (`bno086_reader_task`) that:
  - Calls `bno086_rvc_read()` in a loop
  - On success: logs Yaw/Pitch/Roll (°) and Accel X/Y/Z (m/s²) via `ESP_LOGI`
  - Handles timeouts gracefully

## Open Questions

> [!IMPORTANT]
> **Interrupt pin (GPIO 0)**: The BNO_INT signal is wired to GPIO 0 in the schematic. In UART-RVC mode the sensor streams automatically, so interrupt-driven reads aren't strictly needed. Should I add optional interrupt support, or keep it simple with polling the UART buffer?

> [!NOTE]
> **Baud rate**: UART-RVC mode is fixed at 115200 baud by the BNO086 specification. The user mentioned "115200 or lower if possible" — 115200 is the only option in RVC mode, which works well for the 19 bytes × 100 Hz = ~1900 bytes/s throughput.

## Verification Plan

### Manual Verification
- Flash to ESP32-S3 and observe serial monitor output
- Verify Yaw/Pitch/Roll values change when rotating the sensor
- Verify Accel Z ≈ 9.8 m/s² when the board is stationary and level
- Confirm checksum validation catches corrupted packets
