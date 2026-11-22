#!/usr/bin/env python3
"""
ROVER EXPLORER - Python client

Talks to the single-board rover (ESP32-S3) over Wi-Fi using its HTTP API.
Standard library only.

Wi-Fi (connect the PC to ROVER_EXPLORER_AP first):
    python rover_client.py telemetry                 # print live telemetry
    python rover_client.py telemetry --csv log.csv   # ... and log it to CSV
    python rover_client.py drive 30 -10 --duration 2 # 30 % forward, 10 deg left, 2 s
    python rover_client.py estop
    python rover_client.py snapshot --res 8 --out images   # 1600x1200 (needs PSRAM)
    python rover_client.py status

Driving commands must be repeated faster than every 500 ms or the rover goes back
to neutral (GUI watchdog); `drive` sends at 20 Hz.
"""

import argparse
import csv
import json
import os
import time
import urllib.error
import urllib.request

DEFAULT_HOST = "192.168.4.1"

# ---------------------------------------------------------------------------
# HTTP helpers
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
                         "left", "right", "servo", "failsafe",
                         "enc_left_rpm", "enc_right_rpm", "enc_left_mps", "enc_right_mps",
                         "enc_left_count", "enc_right_count", "dist_m"])
    try:
        while True:
            t0 = time.time()
            try:
                d = http_json(args.host, "/api/telemetry")
                imu = d.get("imu", {})
                mot = d.get("motors", {})
                enc = d.get("enc", {})
                print(f"[{d.get('state', '?'):8}] T={d.get('temp', 0):5.1f}C H={d.get('hum', 0):5.1f}% "
                      f"P={d.get('press', 0):7.1f}hPa | R={imu.get('roll', 0):6.1f} P={imu.get('pitch', 0):6.1f} "
                      f"Y={imu.get('yaw', 0):6.1f} | L={mot.get('left', 0):6.1f}% R={mot.get('right', 0):6.1f}% "
                      f"servo={d.get('servo', 0):5.1f} | enc L={enc.get('left_rpm', 0):6.1f} "
                      f"R={enc.get('right_rpm', 0):6.1f} rpm {enc.get('dist_m', 0):7.2f} m")
                if writer:
                    writer.writerow([f"{t0:.3f}", d.get("state"), d.get("temp"), d.get("hum"), d.get("press"),
                                     d.get("gas"), imu.get("ax"), imu.get("ay"), imu.get("az"),
                                     imu.get("gx"), imu.get("gy"), imu.get("gz"),
                                     imu.get("roll"), imu.get("pitch"), imu.get("yaw"),
                                     mot.get("left"), mot.get("right"), d.get("servo"),
                                     d.get("failsafe"),
                                     enc.get("left_rpm"), enc.get("right_rpm"), enc.get("left_mps"),
                                     enc.get("right_mps"), enc.get("left_count"), enc.get("right_count"),
                                     enc.get("dist_m")])
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
    data = http_get(args.host, f"/capture?t={int(time.time() * 1000)}", timeout=10.0)
    path = os.path.join(args.out, time.strftime("rover_capture_%Y%m%d_%H%M%S.jpg"))
    with open(path, "wb") as f:
        f.write(data)
    print(f"Saved {len(data)} bytes -> {path}")


def cmd_status(args):
    print(json.dumps(http_json(args.host, "/status"), indent=2))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default=DEFAULT_HOST, help="rover address (default 192.168.4.1)")
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
    p.add_argument("--res", type=int, help="resolution index 0..8 (0..4 without PSRAM)")
    p.add_argument("--out", default="images")
    p.set_defaults(func=cmd_snapshot)

    p = sub.add_parser("status", help="board info: stations, free heap/PSRAM, camera limits")
    p.set_defaults(func=cmd_status)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
