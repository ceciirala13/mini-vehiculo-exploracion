# Integrado Completo — Rover Explorer (2 MCUs)

Complete firmware for the exploration rover, split across two microcontrollers that talk over UART:

| Project | Board | What it does |
|---|---|---|
| [MCU_CONTROL](MCU_CONTROL/) | ESP32-S3 (custom board) | Reads the sensors (BME690, MPU6050), drives the steering servo and both rear DC motors (2× DRV8873H), applies the safety logic, and sends telemetry to MCU_COMM. |
| [MCU_COMM](MCU_COMM/) | ESP32 Dev Board | Creates the Wi-Fi network, serves the web GUI, handles the ArduCAM OV2640 camera, and forwards the driving commands from the GUI or Python to MCU_CONTROL. |

`References/` holds the earlier test projects these were built from. They are not used by the build.

---

## 1. How the system works

```
                 Wi-Fi "ROVER_EXPLORER_AP"                       UART 115200
 ┌──────────────┐  WebSocket / HTTP  ┌──────────────────┐  CMD_DRIVE 20 Hz  ┌──────────────────────┐
 │ Browser GUI  │ ─────────────────> │  MCU_COMM        │ ────────────────> │  MCU_CONTROL         │
 │ (keyboard,   │ <───────────────── │  ESP32           │ <──────────────── │  ESP32-S3            │
 │  joystick,   │  telemetry 20 Hz   │  web server      │  TLM_STATE 20 Hz  │  motors, servo,      │
 │  Xbox pad)   │  MJPEG :81/stream  │  camera OV2640   │                   │  BME690, MPU6050     │
 └──────────────┘                    └──────────────────┘                   └──────────────────────┘
   Python client (tools/rover_client.py) uses the same HTTP API
```

### MCU_CONTROL (ESP32-S3)

- **Sensors:** the MPU6050 is read at 50 Hz with an orientation filter (roll, pitch, yaw). The BME690 (temperature, humidity, pressure, gas) is read at 1 Hz. They are on separate I2C buses.
- **Motion control** ([motion_control.c](MCU_CONTROL/main/motion_control.c)) runs every 20 ms and is the only code that touches the motors and the servo. It runs these steps in order:
  1. Takes the latest command from MCU_COMM.
  2. Applies the safety state machine.
  3. Ramps speed and steering (smooth acceleration and braking).
  4. Applies Ackermann steering with an electronic differential: in a turn, the inner rear wheel slows down and the outer one speeds up.
  5. Compensates the motor deadband.
  6. Sends PWM to the DRV8873H drivers and the angle to the servo.
- **Safety states:**

  | State | Meaning |
  |---|---|
  | `DISARMED` | At boot. Motors are off until a "stop" (neutral) command arrives. |
  | `ARMED` | Normal driving. |
  | `FAILSAFE` | No valid command for **> 500 ms**, or E-STOP. The motors brake to 0 and switch off, and the steering centers. |
  | `FAULT` | A DRV8873 reported a fault (nFAULT). The motors stop immediately. |

  It only re-arms from FAILSAFE or FAULT after it receives a neutral command, so the rover never jumps forward on reconnect.
- **Telemetry:** a 69-byte status frame goes to MCU_COMM 20 times per second.

All pins and tuning values (ramps, wheelbase, track, differential gain, motor polarity, servo trim) are in [MCU_CONTROL/main/app_config.h](MCU_CONTROL/main/app_config.h).

### MCU_COMM (ESP32)

- **Wi-Fi access point:** SSID `ROVER_EXPLORER_AP`, password `rover1234`, IP `192.168.4.1`.
- **Web server on port 80:**
  - The GUI page.
  - A WebSocket `/ws` that receives controls and sends telemetry 20 times per second.
  - A REST API (`/api/telemetry`, `/api/control`, `/api/estop`, `/capture`, `/resolution`, `/status`).
- **Camera stream:** a second server on port 81 serves the MJPEG stream (`/stream`). It runs separately so the video never freezes the controls.
- **Camera:** one task owns the camera and only captures while someone is watching. The maximum resolution is 640×480 (no PSRAM).
- **UART link:** the drive command goes to MCU_CONTROL 20 times per second, at a fixed rate whatever the browser does. If no browser or Python input has arrived for **500 ms**, it sends "stop" instead. That is a second safety layer.

Pins and Wi-Fi settings are in [MCU_COMM/main/app_config.h](MCU_COMM/main/app_config.h).

### UART protocol

```
| 0xAA | 0x55 | MSG_ID | SEQ | LEN | PAYLOAD[LEN] | CRC16 (lo, hi) |
```

The checksum is CRC16-CCITT. Bad frames are discarded and the parser resynchronizes on the next header. Message types:
- `0x01 CMD_DRIVE` (COMM → CONTROL): speed and steering. This frame is also the heartbeat.
- `0x10 TLM_STATE` (CONTROL → COMM): all the telemetry.

The definition is in `rover_proto.h`, which **must stay identical in both projects**.

### Controls in the GUI

| Input | Action |
|---|---|
| `W`/`S` or ↑/↓ | Forward / reverse at the selected speed limit |
| `A`/`D` or ←/→ | Steer left / right |
| On-screen joystick | Proportional speed and steering |
| Xbox pad right stick | Proportional; LB/RB change the speed limit |
| `Space`, pad **B** | Emergency stop |
| Clicking outside the window / switching tabs | Automatic emergency stop |

---

## 2. Wiring between the boards

| ESP32-S3 (CONTROL) | | ESP32 (COMM) |
|---|---|---|
| GPIO17 (TX) | → | GPIO16 (RX) |
| GPIO18 (RX) | ← | GPIO17 (TX) |
| GND | — | GND |

- TX always goes to the other board's RX.
- **Grounds must be connected.** Without a common GND the UART won't work.
- Both boards use 3.3 V logic. If each board has its own power supply, connect only the GND, not the 3V3 or 5V rails.

### ArduCAM Mini 2MP Plus → ESP32 (COMM)

| ArduCAM | ESP32 GPIO |
|---|---|
| CS | 5 |
| MOSI | 23 |
| MISO | 19 |
| SCK | 18 |
| SDA | 21 |
| SCL | 22 |
| VCC / GND | 3.3 V (or 5 V) / GND |

### ESP32-S3 (CONTROL) peripherals

| Device | GPIO |
|---|---|
| MPU6050 SDA / SCL | 40 / 41 |
| BME690 SDA / SCL | 5 / 6 |
| Steering servo signal | 39 |
| Motor 1 IN1 / IN2 / DISABLE / nFAULT | 7 / 8 / 9 / 21 |
| Motor 2 IN1 / IN2 / DISABLE / nFAULT | 14 / 15 / 16 / 38 |

---

## 3. Build and flash

Both projects use **ESP-IDF v6.0.1**. Use the ESP-IDF terminal, or the ESP-IDF VS Code extension with the matching target.

```powershell
# Control board (ESP32-S3)
cd MCU_CONTROL
idf.py set-target esp32s3
idf.py build
idf.py -p COM_S3 flash monitor

# Communications board (ESP32)
cd ..\MCU_COMM
idf.py set-target esp32
idf.py build
idf.py -p COM_ESP32 flash monitor
```

- `set-target` is only needed the first time.
- Exit the monitor with `Ctrl+]`.
- Use two terminals so you can watch both monitors at the same time during the tests.

---

## 4. Testing with the hardware

> ⚠️ **Put the rover on a stand with the wheels off the ground** for every test until step 4.6.
> Keep the motor battery switch within reach.

Do the steps in order. Each one checks one layer before the next.

### 4.1 MCU_CONTROL alone (no UART connected yet)

1. Flash MCU_CONTROL and open the monitor. Keep the rover still during boot, because the MPU6050 calibrates its offsets then.
2. Expected log lines:
   ```
   DC_MOTORS: MCPWM dual motor driver initialized (outputs disabled until armed).
   SERVO: Servo initialized, steering centered (servo 90.0 deg)
   MPU6050: MPU6050 successfully initialized and calibrated.
   BME690: BME690 successfully initialized in Forced Mode.
   MAIN_APP: Initialization complete. Motors DISARMED until a neutral command arrives.
   ```
3. Every 2 s a `TELEMETRY` line is printed. Check that:
   - `st=0` (DISARMED), and the motors don't move.
   - Tilting the rover changes `R:`/`P:` (roll/pitch).
   - `T:`, `H:` and `P:` show plausible room values.
4. If a sensor fails, the log names the SDA/SCL pins to check. The rest of the system keeps working.

### 4.2 MCU_CONTROL with the PC as the "COMM board" (optional, recommended)

This tests the protocol, the motors and the failsafe without Wi-Fi.

1. Connect a USB-UART adapter (3.3 V):
   - adapter TX → S3 GPIO18
   - adapter RX ← S3 GPIO17
   - GND ↔ GND
2. Install pyserial: `pip install pyserial`
3. Run:
   ```powershell
   python MCU_COMM\tools\rover_client.py serial --port COMx --speed 20 --angle 15 --duration 3
   ```
4. The script prints the rover state about 4 times per second. Expected sequence:

   | Time | Expected |
   |---|---|
   | 0–1 s | Neutral is sent and the state goes `DISARMED → ARMED`. |
   | 1–4 s | The wheels turn forward. `L` and `R` ramp up smoothly. With +15° (right), **L > R**. The servo turns right. |
   | after 4 s | The script stops sending. **Within ~0.5 s** the state becomes `FAILSAFE`, L/R drop to 0 and the servo centers. |
   | throughout | Telemetry at ~20 Hz, `crc=0`. |

5. On the S3 monitor you'll see `State ARMED -> FAILSAFE (link timeout)`.
6. While this runs, check:
   - **Direction:** a positive speed must move the rover forward. If a wheel turns backwards, set `MOTOR_LEFT_INVERT` / `MOTOR_RIGHT_INVERT`.
   - **Sides:** with +angle (right), the left wheel must be the faster one. If it's reversed, flip `MOTOR_LEFT_IS_M1`.
   - **Steering:** at 0° the front wheels must be straight. If not, adjust `SERVO_CENTER_DEG`. If +angle turns left, set `SERVO_INVERT 1`.

   All of these are in [app_config.h](MCU_CONTROL/main/app_config.h). Rebuild and reflash after any change.

### 4.3 MCU_COMM alone (camera and Wi-Fi)

1. Flash MCU_COMM and open the monitor. Expected log lines:
   ```
   arducam_ov2640: SPI Bus Test OK! (ARDUCHIP_TEST1 = 0x55)
   arducam_ov2640: OV2640 Detected! VID: 0x26, PID: 0x42
   WIFI_AP: Soft-AP started. SSID: [ROVER_EXPLORER_AP] ...
   WEB: HTTP on :80 (GUI, /ws, /api), MJPEG on :81/stream
   MAIN_APP: Ready. Connect to 'ROVER_EXPLORER_AP' and open http://192.168.4.1
   ```
2. Connect a PC or phone to **ROVER_EXPLORER_AP** (`rover1234`) and open **http://192.168.4.1**.
3. Check that:
   - The status badge shows **"COMM OK · CONTROL OFFLINE"**. That is correct, because the S3 isn't connected yet.
   - **📸 Tomar Foto** shows an image, and **▶ Iniciar Stream** shows live video.
   - The "Cámara" row in the system box shows fps and frame size.
   - Changing the resolution (up to VGA) works.
4. If the SPI test fails, recheck CS/MOSI/MISO/SCK. If the OV2640 isn't detected, recheck SDA/SCL and the camera power.

### 4.4 Full system (both boards connected)

1. Wire the UART between the boards (section 2) and power both.
2. Open the GUI. Within a second the badge shows **"ROVER ONLINE · ARMED"**:
   - "MCU Control: Online (xx ms)" with an age below ~100 ms
   - "Estado: ARMED"
   - "UART CRC err: S3:0 · ESP32:0"
   - "Latencia" around 5–50 ms
3. **Live telemetry:** tilt the rover and the artificial horizon, roll/pitch/yaw and the accelerometer bars move. Breathe on the BME690 and humidity rises within a few seconds.
4. **Driving (still on the stand):**
   - Hold `W`: the wheels accelerate smoothly up to the speed limit and stay there **while you hold the key**. "Real: L xx% · R xx%" follows.
   - Release: the wheels slow down to 0.
   - `A`/`D` steer. While moving and steering, the inner wheel is slower.
   - Repeat with the on-screen joystick and the Xbox controller (right stick).
5. **Video plus control together:** start the stream and drive with the keyboard at the same time. The controls must stay responsive and the CRC counters must stay at 0.

### 4.5 Safety tests (do all of them)

| Test | How | Expected |
|---|---|---|
| E-STOP | Drive forward, press `Space` (or pad B) | Immediate stop. The GUI shows the red **FAILSAFE** banner. It re-arms after about 1 s once you release the controls. |
| Lost focus | Drive with `W`, click another window | The rover stops. |
| Browser closed | Drive with the joystick, close the tab | Stops within ~0.5 s (COMM's GUI watchdog). |
| Wi-Fi lost | Drive, then turn off the PC's Wi-Fi | Stops within ~0.5 s. The GUI reconnects by itself when Wi-Fi returns. |
| UART cut | Drive, unplug the COMM TX → S3 RX wire | Stops within ~0.5 s. The S3 log shows `ARMED -> FAILSAFE (link timeout)`. The GUI shows "CONTROL OFFLINE" if you unplug the other wire. |
| COMM reset | Drive, press EN/RESET on the ESP32 | The S3 goes to FAILSAFE. After the reboot, reconnect to the Wi-Fi and it re-arms from neutral. |
| Driver fault | Briefly short nFAULT (GPIO21 or 38) to GND | The motors stop immediately, the state is `FAULT` and the GUI shows the nFAULT banner. It clears when released and the controls are neutral. |

A test fails if the wheels keep turning, or if the rover starts moving by itself after reconnecting.

### 4.6 On the ground

1. Start with the **20 %** speed limit.
2. Drive in an open area and check that straight-line driving is straight. If not, trim `SERVO_CENTER_DEG`.
3. In tight turns the tires shouldn't scrub. If they do, measure and set `VEHICLE_WHEELBASE_M` / `VEHICLE_TRACK_M`, or tune `EDIFF_GAIN` (0 = both wheels at the same speed).
4. Tune the feel with `MOTION_ACCEL_PCT_PER_S`, `MOTION_DECEL_PCT_PER_S` and `MOTION_STEER_DEG_PER_S`. If the rover doesn't move at low speed, raise `MOTOR_MIN_DUTY_PCT`.

---

## 5. Python client (optional)

Connect the PC to `ROVER_EXPLORER_AP`, then:

```powershell
python MCU_COMM\tools\rover_client.py selftest                   # protocol check, no hardware
python MCU_COMM\tools\rover_client.py telemetry --csv log.csv    # live values + CSV log
python MCU_COMM\tools\rover_client.py drive 30 10 --duration 2   # 30 % forward, 10° right, 2 s
python MCU_COMM\tools\rover_client.py estop
python MCU_COMM\tools\rover_client.py snapshot --res 4           # save a VGA photo in .\images
```

Scripts must repeat drive commands more often than every 500 ms (`drive` sends at 20 Hz). Otherwise the rover stops on purpose.

---

## 6. Troubleshooting

| Symptom | Likely cause |
|---|---|
| GUI says "CONTROL OFFLINE" | TX/RX swapped, missing common GND, or MCU_CONTROL not running. Check that the S3 log prints `TELEMETRY` lines. |
| S3 stays `DISARMED` / `FAILSAFE` | No commands are arriving (check the COMM TX → S3 RX wire), or a control is still pressed. Release everything so it sees a neutral command. |
| CRC error counters keep increasing | A loose or long UART wire, missing GND, or noise from the motor wiring. Twist UART with GND and keep it away from the motor cables. |
| Motors never move but the state is `ARMED` | Motor battery off, DISABLE/nFAULT wiring, or `MOTOR_MIN_DUTY_PCT` too low. |
| A wheel turns the wrong way / wrong side faster | `MOTOR_*_INVERT` / `MOTOR_LEFT_IS_M1` in `app_config.h`. |
| Camera: "Sin señal" / `No detectada` | Check the camera wiring and power, and the COMM boot log (SPI test / OV2640 detection). |
| Large resolution doesn't load | Frames bigger than 40 KB don't fit without PSRAM. Use VGA or lower. |
| `idf.py` not found | Open the ESP-IDF terminal, or run `C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1` first. |

More detail per board: [MCU_CONTROL/README.md](MCU_CONTROL/README.md) · [MCU_COMM/README.md](MCU_COMM/README.md)
