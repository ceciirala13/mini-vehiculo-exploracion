# Integrado Control WiFi — Rover Explorer (single board)

Firmware for the exploration rover. One **ESP32-S3 board does everything**: sensors, motors with wheel encoders,
steering, the ArduCAM camera, the Wi-Fi access point and the web GUI.

The camera uses the pins from the original control-board schematic
([pinout_contro_original.png](../pinout_contro_original.png)). The wheel encoders use GPIO43/44 (the chip's UART0
TX/RX pins) and the DRV8873 nFAULT pins (GPIO21/38), so the firmware does not read nFAULT.

| | |
|---|---|
| Module | ESP32-S3-WROOM-1-**N8R8** (8 MB flash, 8 MB octal PSRAM) |
| SDK | ESP-IDF v6.0.1 |
| Wi-Fi | SSID `ROVER_EXPLORER_AP`, password `rover1234`, http://192.168.4.1 |

---

## 1. How it works

```
 ┌──────────────┐   Wi-Fi AP           ┌──────────────────────────────────────────────────────────┐
 │ Browser GUI  │   WebSocket / HTTP   │  ESP32-S3                                                │
 │ (keyboard,   │ ───────────────────> │  core 0: Wi-Fi, web servers, camera, drive bridge        │
 │  joystick,   │ <─────────────────── │  core 1: motion control (50 Hz), MPU6050 (50 Hz), BME690 │
 │  Xbox pad)   │   telemetry 20 Hz    │  PCNT:   wheel encoders (counted in hardware)            │
 └──────────────┘   MJPEG :81/stream   └──────────────────────────────────────────────────────────┘
   Python client (tools/rover_client.py) uses the same HTTP API
```

- **Command path:** the GUI sends `{"speed","angle"}` at 20 Hz → `drive_command` (keeps the latest value) →
  the **drive bridge** task submits it to `motion_control` at a fixed 20 Hz → ramps, Ackermann, speed control,
  motors and servo.
- **Core split:** Wi-Fi, both HTTP servers, the camera and the bridge run on core 0. Motion control and the
  sensors run on core 1, so video and network load never delay the motor loop.
- **Camera:** one task owns the ArduCAM and only captures while someone is watching. The two frame buffers
  (512 KB each) are in PSRAM, so resolutions up to **1600×1200** work. If PSRAM is missing it falls back to
  40 KB internal buffers and VGA.
- **Encoders:** the S3's pulse counter (PCNT) counts both edges of both phases of each wheel encoder in hardware.
  The motion task samples the totals every 20 ms and computes wheel RPM, m/s and distance over a 100 ms window.
  They feed the wheel speed control below and the telemetry.
- **Turns:** the outer rear wheel keeps the commanded speed and the inner one slows down by the geometric
  ratio (R − T/2)/(R + T/2), never below `EDIFF_MIN_INNER_RATIO` (0.6) of the outer wheel, so it never stops.
- **Wheel speed control:** each wheel's duty = its command (feed-forward) + a PI correction from the encoder
  speed (`SPEED_CTRL_*` in [main/app_config.h](main/app_config.h)). Both wheels then turn at the same speed
  going straight, and the slow inner wheel keeps turning in curves. A 100 % command means `WHEEL_MAX_RPM`
  (90 rpm). The correction is limited to ±30 % duty, and the wheel is never driven backwards by it. If the
  encoders fail to start, it drives open-loop. `SPEED_CTRL_ENABLED 0` turns it off.
- **IMU mounting:** the sensor axes are rotated 90° about z before the orientation filter (`IMU_ROTATE_Z_90`):
  new x = −old y, new y = old x, matching the robot frame (x = width, y = length, z = normal). The horizon,
  roll/pitch and the accelerometer/gyro values in the GUI all use the rotated frame. If roll or pitch moves the
  wrong way, set `IMU_ROLL_SIGN` / `IMU_PITCH_SIGN` to −1.

### Safety layers

| Layer | Trigger | Result |
|---|---|---|
| GUI watchdog (`drive_command.c`) | No command from any browser/Python client for **500 ms** | Command becomes neutral, the rover slows down to 0 |
| E-STOP | `Space`, pad **B**, the E-STOP button, `/api/estop`, losing window focus | FAILSAFE: fast brake, motors off, steering centered. Re-arms after ~1 s once the controls are neutral |
| Bridge watchdog (`motion_control.c`) | The bridge task stops submitting for **500 ms** | FAILSAFE |

The motion states are `DISARMED`, `ARMED` and `FAILSAFE`. There is no driver-fault state: the DRV8873 nFAULT pins
are used as encoder inputs. The DRV8873H still protects itself (over-current, over-temperature), but the firmware
isn't told, so a motor can stop without any message in the GUI.
The motion task is on the task watchdog (3 s, panic).

---

## 2. Pinout (ESP32-S3)

| Device | Signal | GPIO | Notes |
|---|---|:---:|---|
| **ArduCAM Mini 2MP Plus (SPI)** | CS / MOSI / SCK / MISO | **10 / 11 / 12 / 13** | `SPI2_HOST` (FSPI IO_MUX pins), 8 MHz |
| **ArduCAM (SCCB)** | SDA / SCL | **47 / 48** | Software (bit-banged) SCCB, 100 kHz (`CAM_I2C_PORT -1`) |
| MPU6050 | SDA / SCL | 40 / 41 | I2C0, 400 kHz |
| BME690 | SDA / SCL | 5 / 6 | I2C1, 400 kHz |
| Steering servo | PWM | 39 | LEDC 50 Hz |
| Motor 1 (DRV8873H) | IN1 / IN2 / DISABLE | 7 / 8 / 9 | nFAULT not connected any more |
| Motor 2 (DRV8873H) | IN1 / IN2 / DISABLE | 14 / 15 / 16 | nFAULT not connected any more |
| **Left wheel encoder** | A / B | **44 / 43** | Module pins 36 (RXD0) / 37 (TXD0), PCNT unit 0 |
| **Right wheel encoder** | A / B | **21 / 38** | Old M1/M2 nFAULT pins, PCNT unit 1 |
| Console / flashing | USB D− / D+ | 19 / 20 | USB-Serial-JTAG |

- **Why software SCCB:** the S3 has only two hardware I2C controllers, and the MPU6050 and the BME690 use both.
  SCCB is only used at boot and when the resolution changes, so bit-banging it costs nothing at runtime.
- **Fallback:** if software SCCB gives trouble, set `MPU6050_ENABLED 0` and `CAM_I2C_PORT 0` in
  [main/app_config.h](main/app_config.h): the camera then uses the hardware I2C0 controller and the IMU is off.
  The build stops with an error if the camera and an enabled sensor are on the same port.
- **Wiring test:** `CAM_WIRING_TEST 1` disables the camera and steps through its six lines (LOW/HIGH every 4 s,
  with short detection) so they can be checked with a multimeter at the camera pins. Every boot also logs a quick
  `Pin check` of SDA, SCL and MISO.
- **The camera needs pull-ups on SDA/SCL.** The ArduCAM module normally has them. The internal pull-ups are also
  enabled, but they are weak.
- **Encoders:** Hall, 11 PPR on the motor shaft, 1:45 gearbox. Counting both edges of both phases gives
  **1980 counts per wheel revolution**. They have built-in pull-ups to their own supply, so **power them from
  3.3 V** (header pin 6), never 5 V: at 5 V the pull-ups would put 5 V on the S3's pins (rated to 3.6 V).
- **GPIO43 at reset:** it is UART0 TX, and the ROM prints its boot messages there for ~100 ms after every reset,
  while the encoder also drives the line. Put a **1 kΩ series resistor** in the left encoder's B line.
  The firmware takes the pin back as soon as `app_main` starts. Disabling the ROM log with an eFuse would also
  work, but it can't be undone.
- Settings in [main/app_config.h](main/app_config.h): `ENC_*_PIN_*`, `ENC_LEFT_INVERT` / `ENC_RIGHT_INVERT`
  (direction), `ENC_MOTOR_PPR`, `ENC_GEAR_RATIO`, `WHEEL_DIAMETER_M` (**65 mm placeholder, measure it**),
  `ENC_GLITCH_NS` (1 µs noise filter), `ENC_SPEED_WINDOW_SAMPLES`.
- Free for later: 17/18 (BNO UART), 1, 2, 4, 42.
- Never use GPIO26–32 (flash) or GPIO35–37 (octal PSRAM).

---

## 3. Build and flash

```powershell
cd "Integrado Control WiFi"
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

- `set-target` is only needed the first time. It generates `sdkconfig` from `sdkconfig.defaults`
  (8 MB flash, large app partition, octal PSRAM, WebSocket support, USB console).
- If you change `sdkconfig.defaults`, delete `sdkconfig` and run `idf.py reconfigure`.
- Exit the monitor with `Ctrl+]`.

---

## 4. Testing with the hardware

> ⚠️ **Put the rover on a stand with the wheels off the ground** for every test until step 4.6.

### 4.1 Boot

Keep the rover still during boot (the MPU6050 calibrates). Expected log lines:

```
MAIN_APP: PSRAM: 8xxx KB free
DC_MOTORS: MCPWM dual motor driver initialized (outputs disabled until armed).
SERVO: Servo initialized, steering centered (servo 90.0 deg)
ENCODERS: left encoder: A=GPIO44 B=GPIO43
ENCODERS: right encoder: A=GPIO21 B=GPIO38
ENCODERS: 1980 counts per wheel revolution (11 PPR x4 x 1:45), wheel 65 mm, speed window 100 ms
MPU6050: MPU6050 successfully initialized and calibrated.
BME690: BME690 successfully initialized in Forced Mode.
CAM_SVC: Frame buffers: 2 x 512 KB in PSRAM, max resolution index 8
arducam_ov2640: Pin check  SDA  (GPIO47): HIGH  [expected HIGH]
arducam_ov2640: Pin check  SCL  (GPIO48): HIGH  [expected HIGH]
arducam_ov2640: SCCB in software (bit-banged)
arducam_ov2640: SPI Bus Test OK! (ARDUCHIP_TEST1 = 0x55)
arducam_ov2640: OV2640 Detected! VID: 0x26, PID: 0x42
WIFI_AP: Soft-AP started. SSID: [ROVER_EXPLORER_AP] ...
WEB: HTTP on :80 (GUI, /ws, /api), MJPEG on :81/stream
MAIN_APP: Ready. Connect to 'ROVER_EXPLORER_AP' and open http://192.168.4.1
```

A `TELEMETRY` line is printed every 2 s. Check that tilting the rover changes `R:`/`P:` and that `T:`, `H:`, `P:`
show plausible room values. The `enc L:… R:…` part shows each wheel's RPM, its raw count and the distance.

- `PSRAM: 0 KB free` → PSRAM is not enabled. Check `CONFIG_SPIRAM_MODE_OCT` in `sdkconfig`.
- SPI test fails → recheck CS/MOSI/SCK/MISO (10/11/12/13), the camera power and GND. The SPI test runs before
  the sensor check, so a failure here has nothing to do with I2C.
- OV2640 not detected (`OV2640 Detection Failed! VID: 0x.. PID: 0x..`) → recheck SDA/SCL (47/48), the pull-ups and
  the camera power. To rule out the software SCCB, try the hardware fallback in section 2.

### 4.2 GUI and camera

1. Connect a PC or phone to **ROVER_EXPLORER_AP** (`rover1234`) and open **http://192.168.4.1**.
2. The badge shows **"ROVER ONLINE · ARMED"** (it arms as soon as it sees a neutral command).
3. "Comandos" shows "Cliente activo (xx ms)". "Memoria" shows the free internal RAM and PSRAM.
4. **📸 Tomar Foto** shows an image, **▶ Iniciar Stream** shows live video, the "Cámara" row shows fps and size.
5. Try every resolution up to **UXGA (1600×1200)**. Higher resolutions are slower: the ArduCAM SPI is limited to
   8 MHz (~1 MB/s, so a 100 KB frame takes ~0.1 s to read), and the sensor itself runs slower at full size.
   Expect a few fps at UXGA.

### 4.3 Encoders (on the stand, motors not driving)

1. Turn each rear wheel **forward** by hand, one full turn. Its count in the `TELEMETRY` line (or "Encoders" in
   the GUI) must go **up by about 1980**. If it goes down, set `ENC_LEFT_INVERT` / `ENC_RIGHT_INVERT` to 1.
   If turning the left wheel changes the right count, swap the `ENC_LEFT_*` and `ENC_RIGHT_*` pins.
2. Turning a wheel slowly back and forth must make the count go up and down with no jumps.
3. Reset the board about 10 times. The left encoder (GPIO43/44) must keep counting correctly after each boot.

### 4.4 Driving (on the stand)

- Hold `W`: the wheels accelerate smoothly to the speed limit and hold while the key is down. "Real: L/R" follows,
  and "Encoders" shows the measured RPM (at 100 % and 12 V, around 100–130 rpm, the motor's rated speed).
- Hold `W` with steering straight: both "Encoders" RPM values must match within a few rpm (speed control).
- Release: the wheels slow down to 0. `A`/`D` steer. While moving and steering, the inner wheel turns slower
  (about 60 % of the outer one at full lock) but never stops.
- **Tilt test for the horizon:** tilt the rover sideways (about its length) → the horizon rotates; lift the nose
  → the horizon moves up/down. If it's the other way round or reversed, see "IMU mounting" in section 1.
- Repeat with the on-screen joystick and the Xbox controller (right stick).
- **Video plus control:** start the stream at VGA and drive at the same time. The controls must stay responsive.
  Repeat at UXGA.
- Check direction, sides and steering trim (`MOTOR_*_INVERT`, `MOTOR_LEFT_IS_M1`, `SERVO_CENTER_DEG`,
  `SERVO_INVERT` in [main/app_config.h](main/app_config.h)).

### 4.5 Safety tests (do all of them)

| Test | How | Expected |
|---|---|---|
| E-STOP | Drive forward, press `Space` (or pad B) | Immediate stop, red **FAILSAFE** banner. Re-arms ~1 s after you release the controls. |
| Lost focus | Drive with `W`, click another window | The rover stops. |
| Browser closed | Drive with the joystick, close the tab | Slows to 0 within ~0.5 s (GUI watchdog). |
| Wi-Fi lost | Drive, turn off the PC's Wi-Fi | Slows to 0 within ~0.5 s. The GUI reconnects by itself. |
| Long run | Stream at VGA for 10+ minutes while driving now and then | No resets, no task watchdog messages, "Memoria" stays stable. |

### 4.6 On the ground

Start at **20 %**, check straight-line driving (`SERVO_CENTER_DEG`), tune
`VEHICLE_WHEELBASE_M` / `VEHICLE_TRACK_M` / `EDIFF_GAIN` / `EDIFF_MIN_INNER_RATIO` and the ramps if needed.

**Tuning the speed control:**
1. Drive at 100 % on the ground and note the RPM of the slower wheel. Set `WHEEL_MAX_RPM` a little below it.
2. If the speed oscillates (RPM swings, wheels "hunt"), lower `SPEED_CTRL_KP` / `SPEED_CTRL_KI`.
3. If a wheel takes long to catch up (e.g. after a bump or at the start of a turn), raise `SPEED_CTRL_KI`.

---

## 5. Web API

| Port | Path | Use |
|---|---|---|
| 80 | `/` | GUI (embedded `main/index.html`) |
| 80 | `/ws` | WebSocket. In: `{"speed":S,"angle":A}`, `{"estop":1}`, `{"ping":T}`, `{"res":N}`. Out: telemetry JSON at 20 Hz, `{"pong":T}` |
| 80 | `/api/telemetry` | Same JSON as the WebSocket push |
| 80 | `/api/control?speed=S&angle=A` | For scripts. **Repeat faster than every 500 ms** |
| 80 | `/api/estop` | Emergency stop (held for 1 s) |
| 80 | `/capture`, `/resolution?res=N`, `/status` | Snapshot JPEG, resolution 0..8, board info |
| 81 | `/stream` | MJPEG, on its own server so it never blocks the GUI/WebSocket |

Telemetry JSON fields:

| Field | Content |
|---|---|
| `temp`, `hum`, `press`, `gas`, `gas_valid`, `bme_ok` | BME690: °C, %RH, hPa, gas resistance (kΩ) |
| `imu` (`ax`…`gz`, `roll`, `pitch`, `yaw`), `mpu_ok` | MPU6050 in the robot frame |
| `motors` (`left`, `right`), `servo` | Applied duty (%) and steering angle (°) |
| `enc` (`ok`, `left_rpm`, `right_rpm`, `left_mps`, `right_mps`, `left_count`, `right_count`, `dist_m`) | Wheel encoders |
| `state`, `armed`, `failsafe` | Motion state |
| `cmd` (`speed`, `angle`, `estop`, `client_active`, `age_ms`, `bridge_ok`) | Effective drive command |
| `sys` (`heap`, `min_heap`, `psram`, `stations`), `cam` (`ok`, `fps`, `size`, `res`, `max_res`) | System and camera |
| `rover_connected`, `uptime_ms`, `proto` | Always `true`, uptime, firmware version |

### Python client

```powershell
python tools\rover_client.py telemetry --csv log.csv
python tools\rover_client.py drive 30 10 --duration 2
python tools\rover_client.py estop
python tools\rover_client.py snapshot --res 8
python tools\rover_client.py status
```

`telemetry` prints and logs the encoder values too.

---

## 6. Code map

| File | Role |
|---|---|
| `app_config.h` | All pins, Wi-Fi settings, camera limits, task cores/priorities, motion tunables |
| `main.c` | Init order (actuators safe first → encoders → sensors → motion → camera → Wi-Fi/web), IMU/ENV tasks, status log |
| `motion_control.c/.h` | **Only owner of the actuators.** Safety state machine → ramps → kinematics → dead-band → outputs |
| `kinematics.c/.h`, `dc_motors.c/.h`, `servo.c/.h` | Ackermann math, DRV8873H MCPWM driver, LEDC servo |
| `encoders.c/.h` | Quadrature decoding on PCNT, wheel RPM / m/s / distance (sampled by the motion task) |
| `mpu6050.c/.h`, `bme690.c/.h`, `BME690_SensorAPI/` | Sensor drivers |
| `vehicle_state.c/.h` | Thread-safe latest sensor snapshot |
| `drive_command.c/.h` | Latest client command, GUI watchdog, E-STOP hold, **20 Hz bridge task to motion_control** |
| `telemetry.c/.h` | Builds the telemetry snapshot and the GUI JSON |
| `rover_types.h` | Motion state enum, status bits, telemetry struct |
| `camera_service.c/.h` | Capture task, double-buffered latest frame in PSRAM |
| `arducam_ov2640.c/.h` | ArduCAM driver: SPI2 burst read through a DMA bounce buffer, SCCB on hardware I2C or bit-banged |
| `web_server.c/.h`, `wifi_ap.c/.h`, `index.html` | HTTP + WebSocket + MJPEG servers, Soft-AP, GUI |

### Next iteration

- Measure `WHEEL_DIAMETER_M` so the m/s and distance values are right.
- Battery measurement: ADC_BAT on GPIO2 (ADC1, works with Wi-Fi). ADC_BAT2 is routed to GPIO42, which has **no ADC**
  on the ESP32-S3: it needs a free ADC1 pin (GPIO1 or GPIO4). The GUI already accepts a `battery` field.
- PID motor control: tune the wheel speed loop (add a filtered D term if needed) and add heading hold with the IMU.
- Air quality: compensate and baseline the BME690 gas resistance, or integrate Bosch BSEC2 for an IAQ index.
