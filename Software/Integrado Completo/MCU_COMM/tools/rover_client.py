#!/usr/bin/env python3
"""
ROVER EXPLORER - Python client

Talks to MCU_COMM over Wi-Fi (HTTP API, stdlib only) or directly to MCU_CONTROL
over a USB-UART adapter (bench testing, needs `pip install pyserial`).

Wi-Fi (connect the PC to ROVER_EXPLORER_AP first):
    python rover_client.py telemetry                 # print live telemetry
    python rover_client.py telemetry --csv log.csv   # ... and log it to CSV
    python rover_client.py drive 30 -10 --duration 2 # 30 % forward, 10 deg left, 2 s
    python rover_client.py estop
    python rover_client.py snapshot --res 4 --out images

Bench test of MCU_CONTROL alone (adapter TX -> S3 GPIO18, RX <- S3 GPIO17, GND):
    python rover_client.py serial --port COM5 --speed 20 --angle 0 --duration 3

Protocol self-test (CRC / framing, mirrors main/rover_proto.c):
    python rover_client.py selftest

Driving commands must be repeated faster than every 500 ms or the rover stops
(GUI watchdog on MCU_COMM and link watchdog on MCU_CONTROL); `drive` sends at 20 Hz.
"""

import argparse
import csv
import json
import os
import struct
import sys
import time
import urllib.error
import urllib.request

DEFAULT_HOST = "192.168.4.1"

# ---------------------------------------------------------------------------
# Protocol (keep in sync with MCU_*/main/rover_proto.h)
# ---------------------------------------------------------------------------
ROVER_PROTO_VERSION = 1
SOF0, SOF1 = 0xAA, 0x55
MAX_PAYLOAD = 96
MSG_CMD_DRIVE = 0x01
MSG_TLM_STATE = 0x10
DRIVE_FLAG_ESTOP = 0x01

DRIVE_FMT = "<hhB"                      # speed_x10, steer_x10, flags
TLM_FMT = "<I9f3fIhhhBBHHB"             # rp_tlm_state_t (69 bytes)
TLM_FIELDS = ("uptime_ms",
              "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps",
              "roll_deg", "pitch_deg", "yaw_deg",
              "temp_c", "hum_pct", "press_hpa", "gas_ohm",
              "motor_left_x10", "motor_right_x10", "steer_x10",
              "status", "motion_state", "rx_ok", "rx_crc_err", "proto_version")
STATE_NAMES = {0: "DISARMED", 1: "ARMED", 2: "FAILSAFE", 3: "FAULT"}


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def encode_frame(msg_id: int, seq: int, payload: bytes) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    body = bytes([msg_id, seq & 0xFF, len(payload)]) + payload
    crc = crc16_ccitt(body)
    return bytes([SOF0, SOF1]) + body + bytes([crc & 0xFF, crc >> 8])


class FrameParser:
    """Stream parser equivalent to rp_parser_feed() (without the inter-byte timeout)."""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes):
        self.buf.extend(data)
        frames = []
        while True:
            start = self.buf.find(bytes([SOF0, SOF1]))
            if start < 0:
                # keep a trailing SOF0 that may start the next frame
                self.buf = self.buf[-1:] if self.buf[-1:] == bytes([SOF0]) else bytearray()
                return frames
            del self.buf[:start]
            if len(self.buf) < 5:
                return frames
            length = self.buf[4]
            if length > MAX_PAYLOAD:
                del self.buf[:1]
                continue
            total = 5 + length + 2
            if len(self.buf) < total:
                return frames
            body = bytes(self.buf[2:5 + length])
            crc_rx = self.buf[5 + length] | (self.buf[6 + length] << 8)
            if crc_rx == crc16_ccitt(body):
                frames.append((body[0], body[1], body[3:]))
                del self.buf[:total]
            else:
                self.crc_errors += 1
                del self.buf[:1]


def decode_tlm(payload: bytes) -> dict:
    t = dict(zip(TLM_FIELDS, struct.unpack(TLM_FMT, payload)))
    t["state"] = STATE_NAMES.get(t["motion_state"], "?")
    return t


# ---------------------------------------------------------------------------
# HTTP helpers (MCU_COMM API)
# ---------------------------------------------------------------------------
def http_get(host: str, path: str, timeout: float = 2.0) -> bytes:
    with urllib.request.urlopen(f"http://{host}{path}", timeout=timeout) as r:
        return r.read()


def http_json(host: str, path: str, timeout: float = 2.0) -> dict:
    return json.loads(http_get(host, path, timeout))


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------
def cmd_telemetry(args):
    period = 1.0 / args.rate
    writer = None
    fh = None
    if args.csv:
        fh = open(args.csv, "w", newline="", encoding="utf-8")
        writer = csv.writer(fh)
        writer.writerow(["t", "state", "temp", "hum", "press", "gas_kohm",
                         "ax", "ay", "az", "gx", "gy", "gz", "roll", "pitch", "yaw",
                         "left", "right", "servo", "fault", "failsafe"])
    try:
        while True:
            t0 = time.time()
            try:
                d = http_json(args.host, "/api/telemetry")
                imu = d.get("imu", {})
                mot = d.get("motors", {})
                print(f"[{d.get('state', '?'):8}] T={d.get('temp', 0):5.1f}C H={d.get('hum', 0):5.1f}% "
                      f"P={d.get('press', 0):7.1f}hPa | R={imu.get('roll', 0):6.1f} P={imu.get('pitch', 0):6.1f} "
                      f"Y={imu.get('yaw', 0):6.1f} | L={mot.get('left', 0):6.1f}% R={mot.get('right', 0):6.1f}% "
                      f"servo={d.get('servo', 0):5.1f}")
                if writer:
                    writer.writerow([f"{t0:.3f}", d.get("state"), d.get("temp"), d.get("hum"), d.get("press"),
                                     d.get("gas"), imu.get("ax"), imu.get("ay"), imu.get("az"),
                                     imu.get("gx"), imu.get("gy"), imu.get("gz"),
                                     imu.get("roll"), imu.get("pitch"), imu.get("yaw"),
                                     mot.get("left"), mot.get("right"), d.get("servo"),
                                     d.get("fault"), d.get("failsafe")])
                    fh.flush()
            except (urllib.error.URLError, OSError, ValueError) as e:
                print(f"[no link] {e}")
            time.sleep(max(0.0, period - (time.time() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        if fh:
            fh.close()


def cmd_drive(args):
    end = time.time() + args.duration
    print(f"Driving speed={args.speed}% angle={args.angle} deg for {args.duration}s (Ctrl+C = stop)")
    try:
        while time.time() < end:
            t0 = time.time()
            try:
                http_get(args.host, f"/api/control?speed={args.speed}&angle={args.angle}", timeout=0.4)
            except (urllib.error.URLError, OSError) as e:
                print(f"[send failed] {e}")
            time.sleep(max(0.0, 0.05 - (time.time() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        try:
            http_get(args.host, "/api/control?speed=0&angle=0", timeout=0.5)
        except (urllib.error.URLError, OSError):
            pass
        print("Neutral sent.")


def cmd_estop(args):
    print(http_json(args.host, "/api/estop"))


def cmd_snapshot(args):
    if args.res is not None:
        print(http_json(args.host, f"/resolution?res={args.res}"))
        time.sleep(0.5)
    os.makedirs(args.out, exist_ok=True)
    data = http_get(args.host, f"/capture?t={int(time.time() * 1000)}", timeout=5.0)
    path = os.path.join(args.out, time.strftime("rover_capture_%Y%m%d_%H%M%S.jpg"))
    with open(path, "wb") as f:
        f.write(data)
    print(f"Saved {len(data)} bytes -> {path}")


def cmd_serial(args):
    try:
        import serial  # pyserial
    except ImportError:
        sys.exit("pyserial is required: pip install pyserial")

    ser = serial.Serial(args.port, args.baud, timeout=0.01)
    parser = FrameParser()
    seq = 0
    start = time.time()
    next_tx = start
    last_print = 0.0
    tlm_count = 0
    print(f"Emulating MCU_COMM on {args.port}: neutral for 1 s (arming), then "
          f"speed={args.speed}% angle={args.angle} for {args.duration}s, then silence to test FAILSAFE.")
    try:
        while True:
            now = time.time()
            elapsed = now - start
            sending = elapsed < 1.0 + args.duration
            if sending and now >= next_tx:
                speed, angle = (0.0, 0.0) if elapsed < 1.0 else (args.speed, args.angle)
                payload = struct.pack(DRIVE_FMT, int(speed * 10), int(angle * 10), 0)
                ser.write(encode_frame(MSG_CMD_DRIVE, seq, payload))
                seq += 1
                next_tx += 0.05
            if elapsed > 1.0 + args.duration + 2.0:
                break

            for msg_id, _seq, payload in parser.feed(ser.read(256)):
                if msg_id == MSG_TLM_STATE and len(payload) == struct.calcsize(TLM_FMT):
                    tlm_count += 1
                    t = decode_tlm(payload)
                    if now - last_print >= 0.25:
                        last_print = now
                        print(f"{elapsed:6.2f}s {'TX' if sending else '--'} [{t['state']:8}] "
                              f"L={t['motor_left_x10'] / 10:6.1f}% R={t['motor_right_x10'] / 10:6.1f}% "
                              f"steer={t['steer_x10'] / 10:5.1f} st=0x{t['status']:02X} "
                              f"rx_ok={t['rx_ok']} crc={t['rx_crc_err']} | tlm {tlm_count / max(elapsed, 1e-3):4.1f} Hz")
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print(f"Done. Telemetry frames: {tlm_count}, local CRC errors: {parser.crc_errors}")


def cmd_selftest(_args):
    assert crc16_ccitt(b"123456789") == 0x29B1, "CRC16-CCITT check value mismatch"
    assert struct.calcsize(TLM_FMT) == 69, "rp_tlm_state_t size mismatch"
    assert struct.calcsize(DRIVE_FMT) == 5, "rp_cmd_drive_t size mismatch"

    drive = struct.pack(DRIVE_FMT, 305, -100, DRIVE_FLAG_ESTOP)
    frame = encode_frame(MSG_CMD_DRIVE, 7, drive)
    p = FrameParser()
    # garbage + split frame + corrupted frame + good frame
    bad = bytearray(frame)
    bad[6] ^= 0xFF
    got = p.feed(b"\x00\xAA\x13" + frame[:4]) + p.feed(frame[4:] + bytes(bad) + frame)
    assert len(got) == 2 and got[0] == (MSG_CMD_DRIVE, 7, drive), got
    assert p.crc_errors == 1
    print("selftest OK: CRC16 0x29B1, frame sizes 5/69, parser resync + CRC rejection")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default=DEFAULT_HOST, help="MCU_COMM address (default 192.168.4.1)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("telemetry", help="print / log live telemetry")
    p.add_argument("--rate", type=float, default=5.0, help="polls per second")
    p.add_argument("--csv", help="CSV output file")
    p.set_defaults(func=cmd_telemetry)

    p = sub.add_parser("drive", help="drive for a while, then neutral")
    p.add_argument("speed", type=float, help="-100..100 %%")
    p.add_argument("angle", type=float, help="-45..45 deg (+ = right)")
    p.add_argument("--duration", type=float, default=1.0)
    p.set_defaults(func=cmd_drive)

    p = sub.add_parser("estop", help="emergency stop")
    p.set_defaults(func=cmd_estop)

    p = sub.add_parser("snapshot", help="save a JPEG")
    p.add_argument("--res", type=int, help="resolution index 0..4")
    p.add_argument("--out", default="images")
    p.set_defaults(func=cmd_snapshot)

    p = sub.add_parser("serial", help="emulate MCU_COMM on a USB-UART adapter (bench test MCU_CONTROL)")
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--speed", type=float, default=20.0)
    p.add_argument("--angle", type=float, default=0.0)
    p.add_argument("--duration", type=float, default=3.0)
    p.set_defaults(func=cmd_serial)

    p = sub.add_parser("selftest", help="protocol CRC/framing self-test (no hardware)")
    p.set_defaults(func=cmd_selftest)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
