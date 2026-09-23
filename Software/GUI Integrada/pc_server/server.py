#!/usr/bin/env python3
"""
ROVER EXPLORER - SERVIDOR DE PROCESAMIENTO EN PC (PYTHON + OPENCV)
- Conexión TCP Wi-Fi con ESP32-C6 (192.168.4.1:8080)
- Procesamiento de Imágenes con OpenCV (Stream MJPEG, Captura, Filtros)
- Procesamiento de Sensores (BME690 IAQ + IMU MPU6050)
- Servidor Web Local (FastAPI) y Control de Movimiento (Mando Xbox One S)
"""

import asyncio
import socket
import struct
import threading
import time
import os
import cv2
import numpy as np
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Response
from fastapi.responses import HTMLResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

# Configuración de Red
ESP32_IP = "192.168.4.1"
ESP32_PORT = 8080
HTTP_PORT = 8000

# Constantes del Protocolo Binario
PKT_MAGIC_0 = 0xAA
PKT_MAGIC_1 = 0x55

PKT_TYPE_HEARTBEAT    = 0x00
PKT_TYPE_CONTROL      = 0x01
PKT_TYPE_TELEMETRY    = 0x02
PKT_TYPE_CAMERA_CHUNK = 0x03
PKT_TYPE_CAM_COMMAND  = 0x04
PKT_TYPE_IMU          = 0x05

# Estado Global del Sistema
telemetry_state = {
    "temp": 24.5,
    "hum": 50.0,
    "press": 1013.2,
    "gas": 75000,
    "iaq": 50,
    "iaq_ratio": 1.0,
    "v_motors": 11.8,
    "pct_motors": 85,
    "v_esp": 4.12,
    "pct_esp": 92,
    "fault": False,
    "imu": {
        "ax": 0.0, "ay": 0.0, "az": 1.0,
        "gx": 0.0, "gy": 0.0, "gz": 0.0,
        "roll": 0.0, "pitch": 0.0
    },
    "rover_connected": False
}

# Almacén de Frame de Cámara
current_jpeg_bytes = None
frame_lock = threading.Lock()
frame_assembly = {"frame_id": 0, "total_len": 0, "buffer": bytearray()}

# Socket TCP
tcp_socket = None
socket_lock = threading.Lock()

def calc_checksum(pkt_type: int, length: int, payload: bytes) -> int:
    csum = pkt_type ^ (length & 0xFF) ^ ((length >> 8) & 0xFF)
    for b in payload:
        csum ^= b
    return csum & 0xFF

def send_tcp_packet(pkt_type: int, payload: bytes):
    global tcp_socket
    with socket_lock:
        if tcp_socket is None:
            return False
        try:
            length = len(payload)
            csum = calc_checksum(pkt_type, length, payload)
            header = bytes([PKT_MAGIC_0, PKT_MAGIC_1, pkt_type, length & 0xFF, (length >> 8) & 0xFF])
            tcp_socket.sendall(header + payload + bytes([csum]))
            return True
        except Exception as e:
            print(f"[TCP ERROR] Error enviando paquete: {e}")
            return False

def send_control_command(speed: float, angle: float):
    """Envía comando de velocidad y ángulo hacia el ESP32-S3 a través del ESP32-C6."""
    payload = struct.pack("<ff", float(speed), float(angle))
    return send_tcp_packet(PKT_TYPE_CONTROL, payload)

def send_camera_command(cmd: int, param: int = 0):
    """Envía comandos de cámara (1=snapshot, 2=stream_on, 3=stream_off, 4=res)."""
    payload = struct.pack("<BB", int(cmd), int(param))
    return send_tcp_packet(PKT_TYPE_CAM_COMMAND, payload)

def process_camera_frame(raw_jpeg: bytes):
    """Procesamiento de imagen con OpenCV en la PC."""
    global current_jpeg_bytes
    try:
        np_arr = np.frombuffer(raw_jpeg, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            return

        # --- PIPELINE DE PROCESAMIENTO OPENCV EN PC ---
        # 1. Mejora de contraste / nitidez opcional
        # 2. Overlay de Telemetría o información en tiempo real
        h, w, _ = img.shape
        cv2.putText(img, f"ROVER PC-LINK | {w}x{h}", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 240, 255), 1, cv2.LINE_AA)

        # Re-codificar a JPEG para streaming web
        _, encoded_img = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        with frame_lock:
            current_jpeg_bytes = encoded_img.tobytes()
    except Exception as e:
        print(f"[OPENCV ERROR] {e}")

def process_bme_iaq(temp, hum, press, gas_ohm):
    """Calcula el índice IAQ (Calidad de Aire) en base al sensor BME690."""
    r_gas_kohm = gas_ohm / 1000.0
    baseline_kohm = 75.0 # Resistencia base en aire limpio

    ratio = r_gas_kohm / max(baseline_kohm, 1.0)
    iaq_raw = (1.0 - ratio) * 200.0 + 50.0
    iaq = max(0, min(500, int(iaq_raw)))

    return iaq, round(ratio, 2)

def tcp_worker_thread():
    """Hilo secundario para mantener conexión TCP con el ESP32-C6."""
    global tcp_socket, telemetry_state, frame_assembly
    while True:
        try:
            print(f"[TCP] Conectando a Rover Wi-Fi AP ({ESP32_IP}:{ESP32_PORT})...")
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5.0)
            s.connect((ESP32_IP, ESP32_PORT))
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

            with socket_lock:
                tcp_socket = s
            telemetry_state["rover_connected"] = True
            print("[TCP] ¡Conexión establecida con Rover Explorer!")

            # Solicitar inicio de stream
            time.sleep(0.2)
            send_camera_command(2, 4) # Stream ON, VGA

            s.settimeout(3.0)
            buffer = bytearray()

            while True:
                data = s.recv(4096)
                if not data:
                    break
                buffer.extend(data)

                # Procesar tramas binarias
                while len(buffer) >= 6:
                    if buffer[0] != PKT_MAGIC_0 or buffer[1] != PKT_MAGIC_1:
                        # Descartar hasta encontrar el magic delimiter
                        del buffer[0]
                        continue

                    pkt_type = buffer[2]
                    pkt_len = buffer[3] | (buffer[4] << 8)
                    total_packet_size = 5 + pkt_len + 1

                    if len(buffer) < total_packet_size:
                        break # Esperar más datos

                    payload = bytes(buffer[5:5 + pkt_len])
                    csum = buffer[5 + pkt_len]

                    expected_csum = calc_checksum(pkt_type, pkt_len, payload)
                    if csum == expected_csum:
                        # Procesar paquete
                        if pkt_type == PKT_TYPE_TELEMETRY and len(payload) == 33:
                            t, h, p, g, vm, pm, ve, pe, fault = struct.unpack("<fffIffffB", payload)
                            iaq, ratio = process_bme_iaq(t, h, p, g)
                            telemetry_state["temp"] = round(t, 1)
                            telemetry_state["hum"] = round(h, 1)
                            telemetry_state["press"] = round(p, 1)
                            telemetry_state["gas"] = g
                            telemetry_state["iaq"] = iaq
                            telemetry_state["iaq_ratio"] = ratio
                            telemetry_state["v_motors"] = round(vm, 1)
                            telemetry_state["pct_motors"] = int(pm)
                            telemetry_state["v_esp"] = round(ve, 2)
                            telemetry_state["pct_esp"] = int(pe)
                            telemetry_state["fault"] = bool(fault)

                        elif pkt_type == PKT_TYPE_IMU and len(payload) == 32:
                            ax, ay, az, gx, gy, gz, roll, pitch = struct.unpack("<ffffffff", payload)
                            telemetry_state["imu"]["ax"] = round(ax, 2)
                            telemetry_state["imu"]["ay"] = round(ay, 2)
                            telemetry_state["imu"]["az"] = round(az, 2)
                            telemetry_state["imu"]["gx"] = round(gx, 1)
                            telemetry_state["imu"]["gy"] = round(gy, 1)
                            telemetry_state["imu"]["gz"] = round(gz, 1)
                            telemetry_state["imu"]["roll"] = round(roll, 1)
                            telemetry_state["imu"]["pitch"] = round(pitch, 1)

                        elif pkt_type == PKT_TYPE_CAMERA_CHUNK and len(payload) >= 14:
                            frame_id, total_len, offset, chunk_len = struct.unpack("<IIIH", payload[:14])
                            chunk_data = payload[14:14 + chunk_len]

                            if frame_assembly["frame_id"] != frame_id:
                                frame_assembly["frame_id"] = frame_id
                                frame_assembly["total_len"] = total_len
                                frame_assembly["buffer"] = bytearray(total_len)

                            if offset + chunk_len <= len(frame_assembly["buffer"]):
                                frame_assembly["buffer"][offset:offset + chunk_len] = chunk_data

                                # Frame completo
                                if offset + chunk_len >= total_len:
                                    process_camera_frame(bytes(frame_assembly["buffer"]))

                    del buffer[:total_packet_size]

        except Exception as e:
            print(f"[TCP EXCEPTION] Desconectado del Rover: {e}")
        finally:
            with socket_lock:
                if tcp_socket:
                    try:
                        tcp_socket.close()
                    except:
                        pass
                    tcp_socket = None
            telemetry_state["rover_connected"] = False
            time.sleep(2.0) # Reintento

# Iniciar hilo de comunicación TCP
t = threading.Thread(target=tcp_worker_thread, daemon=True)
t.start()

# ================= APLICACIÓN FASTAPI / WEB DASHBOARD =================
app = FastAPI(title="Rover Explorer PC Server")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

HTML_FILE_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "index.html"))

@app.get("/", response_class=HTMLResponse)
async def get_index():
    """Sirve la página web dashboard."""
    if os.path.exists(HTML_FILE_PATH):
        with open(HTML_FILE_PATH, "r", encoding="utf-8") as f:
            return f.read()
    return "<h1>Archivo index.html no encontrado</h1>"

@app.get("/api/telemetry")
async def get_telemetry():
    return telemetry_state

@app.get("/api/control")
async def api_control(speed: float = 0.0, angle: float = 0.0):
    send_control_command(speed, angle)
    return {"status": "ok", "speed": speed, "angle": angle}

@app.get("/api/cam_cmd")
async def api_cam_cmd(cmd: int = 1, param: int = 0):
    send_camera_command(cmd, param)
    return {"status": "ok", "cmd": cmd, "param": param}

@app.get("/capture")
async def get_capture():
    with frame_lock:
        if current_jpeg_bytes:
            return Response(content=current_jpeg_bytes, media_type="image/jpeg")
    
    # Placeholder si aún no hay frame
    blank = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(blank, "SIN SENAL DE CAMARA", (180, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    _, buf = cv2.imencode('.jpg', blank)
    return Response(content=buf.tobytes(), media_type="image/jpeg")

def generate_mjpeg_stream():
    while True:
        frame = None
        with frame_lock:
            if current_jpeg_bytes:
                frame = current_jpeg_bytes
        if frame:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.04) # ~25 fps

@app.get("/stream")
async def get_stream():
    return StreamingResponse(generate_mjpeg_stream(), media_type="multipart/x-mixed-replace; boundary=frame")

# WebSocket para telemetría en vivo y recepción de comandos de mando Xbox
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        async def send_telemetry_loop():
            while True:
                await websocket.send_json(telemetry_state)
                await asyncio.sleep(0.05) # 20 Hz telemetría

        task = asyncio.create_task(send_telemetry_loop())

        while True:
            data = await websocket.receive_json()
            if "speed" in data and "angle" in data:
                send_control_command(data["speed"], data["angle"])
            elif "cam_cmd" in data:
                send_camera_command(data.get("cam_cmd", 1), data.get("param", 0))

    except WebSocketDisconnect:
        pass
    except Exception as e:
        print(f"[WS ERROR] {e}")
    finally:
        task.cancel()

if __name__ == "__main__":
    print("\n==========================================================")
    print("  ROVER EXPLORER - SERVIDOR DE PROCESAMIENTO EN PC")
    print(f"  Dashboard Web disponible en: http://localhost:{HTTP_PORT}")
    print(f"  Conectando con Rover en: {ESP32_IP}:{ESP32_PORT}")
    print("==========================================================\n")
    uvicorn.run(app, host="0.0.0.0", port=HTTP_PORT, log_level="warning")
