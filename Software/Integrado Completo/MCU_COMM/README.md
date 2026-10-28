# MCU_COMM — ESP32 Dev Board (Wi-Fi, camera, web GUI)

Hosts the Wi-Fi access point and the web GUI, owns the ArduCAM OV2640, and bridges
browser/Python commands and telemetry to **MCU_CONTROL** over UART.

```
Browser / Python ──WebSocket/HTTP──> drive_command ──CMD_DRIVE 20 Hz──> UART ──> MCU_CONTROL
MCU_CONTROL ──TLM_STATE 20 Hz──> UART ──> telemetry_store ──JSON 20 Hz──> WebSocket clients
ArduCAM ──> camera_service (double buffer) ──> :81/stream, :80/capture
```

## Wiring

| Function | GPIO | Notes |
|---|---|---|
| UART2 TX → S3 RX (GPIO18) | 17 | 115200 8N1, common GND |
| UART2 RX ← S3 TX (GPIO17) | 16 | |
| ArduCAM SCK / MISO / MOSI / CS | 18 / 19 / 23 / 5 | VSPI (`SPI3_HOST`), 8 MHz |
| ArduCAM SDA / SCL (SCCB) | 21 / 22 | I2C0, 100 kHz |

Pins, Wi-Fi credentials and limits: [main/app_config.h](main/app_config.h).

## Build / flash (ESP-IDF v6.0.1)

```
idf.py set-target esp32
idf.py build
idf.py -p COMx flash monitor
```

Then join Wi-Fi **ROVER_EXPLORER_AP** (password `rover1234`) and open **http://192.168.4.1**.
Opening `main/index.html` directly from disk (`file://`) starts the GUI in simulation mode.

## Endpoints

| Port | Path | Use |
|---|---|---|
| 80 | `/` | GUI (embedded `index.html`) |
| 80 | `/ws` | WebSocket. In: `{"speed":S,"angle":A}`, `{"estop":1}`, `{"ping":T}`, `{"res":N}`. Out: telemetry JSON at 20 Hz, `{"pong":T}` |
| 80 | `/api/telemetry` | Same JSON as the WebSocket push |
| 80 | `/api/control?speed=S&angle=A` | For scripts. **Repeat faster than every 500 ms** |
| 80 | `/api/estop` | Emergency stop (held for 1 s) |
| 80 | `/capture`, `/resolution?res=N`, `/status` | Snapshot JPEG, resolution 0..4, info |
| 81 | `/stream` | MJPEG. It runs on its own server so it never blocks the GUI/WebSocket |

## Code map

| File | Role |
|---|---|
| `rover_proto.c/.h` | UART protocol (**identical copy in MCU_CONTROL**) |
| `uart_link.c/.h` | RX task → `telemetry_store`; TX task sends `CMD_DRIVE` at a fixed 20 Hz (heartbeat) |
| `drive_command.c/.h` | Latest client command + **GUI watchdog** (no client input for 500 ms → neutral) |
| `telemetry_store.c/.h` | Latest CONTROL telemetry, online flag, JSON rendering |
| `camera_service.c/.h` | Single capture task, double-buffered latest frame, runs only while there are viewers |
| `arducam_ov2640.c/.h` | ArduCAM driver ported to IDF v6 (`i2c_master`, configurable SPI host) |
| `web_server.c/.h` | Both HTTP servers, WebSocket, REST |
| `wifi_ap.c/.h` | Soft-AP |
| `index.html` | GUI (from `References/Integrado GUI`, adapted) |
| `tools/rover_client.py` | Python client: telemetry/CSV, scripted driving, snapshots, serial bench test, protocol self-test |

## GUI changes vs. the reference

- Commands are sent at a fixed **20 Hz** while connected, not only when an input changes. Before this, holding a key sent nothing, and the rover's 500 ms watchdog would have stopped it.
- The WebSocket **auto-reconnects** (0.5→5 s backoff). HTTP polling is used only while the WebSocket is down.
- Losing window focus or hiding the tab triggers an **E-STOP**, so keys can't stay "stuck". Space, the B button and E-STOP all send `{"estop":1}`.
- Telemetry is live: BME690, IMU (now including yaw), the actual ramped wheel duty and servo angle, motion state, FAULT/FAILSAFE banner, UART CRC counters, camera fps and WebSocket RTT.
- The batteries show **N/D**, because the hardware has no battery measurement yet. The JSON accepts an optional `battery` field if one is added later.
- The stream comes from `:81/stream`. Resolutions are limited to ≤ 640×480 because the ESP32 has no PSRAM (two 40 KB frame buffers).

## Python client

```
python tools/rover_client.py selftest
python tools/rover_client.py telemetry --csv log.csv
python tools/rover_client.py drive 30 10 --duration 2
python tools/rover_client.py snapshot --res 4
```
