# MCU_CONTROL — ESP32-S3 (sensors + motion)

Reads the BME690 and MPU6050, drives the steering servo and the two rear DC motors
(DRV8873H), and exchanges binary frames with **MCU_COMM** over UART.

## Wiring

| Function | GPIO | Notes |
|---|---|---|
| UART1 TX → COMM RX (GPIO16) | 17 | 115200 8N1, common GND |
| UART1 RX ← COMM TX (GPIO17) | 18 | |
| MPU6050 SDA / SCL (I2C0) | 40 / 41 | 0x68, 400 kHz |
| BME690 SDA / SCL (I2C1) | 5 / 6 | 0x76, 400 kHz |
| Steering servo PWM | 39 | 50 Hz, 500–2500 µs |
| M1 IN1 / IN2 / DISABLE / nFAULT | 7 / 8 / 9 / 21 | |
| M2 IN1 / IN2 / DISABLE / nFAULT | 14 / 15 / 16 / 38 | |

All pins and tunables live in [main/app_config.h](main/app_config.h).

## Build / flash (ESP-IDF v6.0.1)

```
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

## Code map

| File | Role |
|---|---|
| `rover_proto.c/.h` | UART frame encoder + stream parser (**identical copy in MCU_COMM**) |
| `uart_link.c/.h` | UART driver, RX task → `motion_control_submit()`, framed TX |
| `motion_control.c/.h` | **Only owner of the actuators.** Safety state machine → ramps → kinematics → dead-band → outputs |
| `kinematics.c/.h` | Pure math: Ackermann + electronic differential |
| `dc_motors.c/.h` | Low-level MCPWM H-bridge driver (M1/M2, signed duty, enable, nFAULT) |
| `servo.c/.h` | LEDC servo, continuous angle + steering trim/limit |
| `mpu6050.c/.h`, `bme690.c/.h` | Sensor drivers (`i2c_master`), Bosch API in `BME690_SensorAPI/` |
| `vehicle_state.c/.h` | Thread-safe latest sensor snapshot |
| `main.c` | Init + IMU (50 Hz), ENV (1 Hz) and telemetry (20 Hz) tasks |

### Changing the motor behaviour

Everything is in `motion_control.c`. Each stage is its own static function:

1. `read_command()`: newest command plus its freshness (link watchdog).
2. `update_state()`: DISARMED / ARMED / FAILSAFE / FAULT.
3. `apply_ramps()`: slew-rate limits (`MOTION_ACCEL/DECEL/ESTOP_DECEL/STEER_*`).
4. `kinematics_ackermann()`: wheel speeds from speed and steering (`EDIFF_GAIN`, wheelbase, track).
5. `deadband_comp()`: `MOTOR_DEADBAND_PCT` / `MOTOR_MIN_DUTY_PCT`.
6. `drive_outputs()`: wheel→bridge mapping (`MOTOR_LEFT_IS_M1`, `*_INVERT`), enable, servo.

**Measure `VEHICLE_WHEELBASE_M` and `VEHICLE_TRACK_M`** on the rover; the defaults are placeholders.
`EDIFF_GAIN = 0` gives pure Ackermann (both rear wheels at the same speed).

## Safety

| State | Entered when | Outputs | Leaves when |
|---|---|---|---|
| DISARMED | boot | bridges disabled (Hi-Z), servo centered | fresh command with \|speed\| < 1 % |
| ARMED | armed | normal | link lost / E-STOP → FAILSAFE, nFAULT → FAULT |
| FAILSAFE | no valid `CMD_DRIVE` for **> 500 ms**, or E-STOP flag | ramps to 0 at 1000 %/s, then disabled; servo centers | stopped **and** fresh neutral command |
| FAULT | DRV8873 nFAULT low | immediate stop, disabled | nFAULT released **and** fresh neutral command |

The motion task is registered with the task watchdog (3 s, panic).

## Bench test without MCU_COMM

Connect a USB-UART adapter (adapter TX → GPIO18, RX ← GPIO17, GND) and run:

```
python ../MCU_COMM/tools/rover_client.py serial --port COM5 --speed 20 --angle 15 --duration 3
```

Expected: DISARMED → ARMED during the 1 s neutral phase, then ramped L/R duty (the outer wheel is faster),
then **FAILSAFE within ~0.5 s** of the script going silent, and telemetry at ~20 Hz throughout.

Also verify on the bench that "both inputs low" (stop) and DISABLE high (coast) behave as expected for this
board's DRV8873H wiring (see the datasheet's PWM-mode truth table).
