"""
Script de Descarga y Almacenamiento Automático de Fotos en Computadora
Conexión: Wi-Fi AP de ESP32-C6 (http://192.168.4.1)
Cámara: ArduCAM Mini 2MP Plus (OV2640)
"""

import os
import time
import argparse
import requests
from datetime import datetime

# ================= CONFIGURACIÓN =================
DEFAULT_HOST = "192.168.4.1"       # IP del punto de acceso Wi-Fi del ESP32
DEFAULT_PORT = 80
DEFAULT_OUTPUT_DIR = "images"       # Carpeta local en el PC
TIMEOUT_SECONDS = 10
# =================================================

def capture_and_save(host=DEFAULT_HOST, port=DEFAULT_PORT, output_dir=DEFAULT_OUTPUT_DIR):
    os.makedirs(output_dir, exist_ok=True)
    endpoint_url = f"http://{host}:{port}/capture"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"rover_capture_{timestamp}.jpg"
    filepath = os.path.join(output_dir, filename)

    print(f"[+] Solicitando captura a: {endpoint_url}")
    start_time = time.time()

    try:
        response = requests.get(endpoint_url, timeout=TIMEOUT_SECONDS)
        
        if response.status_code == 200:
            content_type = response.headers.get("Content-Type", "")
            if "image" not in content_type and not response.content.startswith(b"\xff\xd8"):
                print("[-] Advertencia: La respuesta recibida no parece ser una imagen JPEG válida.")

            with open(filepath, "wb") as f:
                f.write(response.content)

            elapsed = time.time() - start_time
            size_kb = len(response.content) / 1024.0
            print(f"[✓] Foto guardada exitosamente: {filepath}")
            print(f"    Tamaño: {size_kb:.2f} KB | Tiempo de transferencia: {elapsed:.2f} s")
            return filepath
        else:
            print(f"[-] Error del servidor HTTP: Código {response.status_code}")
            print(f"    Respuesta: {response.text}")
            return None

    except requests.exceptions.RequestException as e:
        print(f"[-] Error de conexión de red con el ESP32: {e}")
        print("    Verifica que tu PC esté conectado a la red Wi-Fi 'ROVER_EXPLORER_AP'.")
        return None

def change_resolution(res_code, host=DEFAULT_HOST, port=DEFAULT_PORT):
    url = f"http://{host}:{port}/resolution?res={res_code}"
    try:
        r = requests.get(url, timeout=5)
        if r.status_code == 200:
            print(f"[+] Resolución cambiada con éxito (Código {res_code})")
        else:
            print(f"[-] Error cambiando resolución: {r.status_code}")
    except Exception as e:
        print(f"[-] Error: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Descargador de fotos ArduCAM ESP32-C6")
    parser.add_argument("--host", default=DEFAULT_HOST, help="Dirección IP del ESP32")
    parser.add_argument("--dir", default=DEFAULT_OUTPUT_DIR, help="Carpeta de destino")
    parser.add_argument("--loop", type=int, default=0, help="Intervalo en segundos para captura continua (0 = una sola foto)")
    parser.add_argument("--res", type=int, choices=range(0, 9), help="Código de resolución (0=160x120, 4=640x480, 8=1600x1200)")

    args = parser.parse_args()

    if args.res is not None:
        change_resolution(args.res, host=args.host)

    if args.loop > 0:
        print(f"[+] Modo ráfaga continuo activo cada {args.loop} segundos. Presiona Ctrl+C para detener.")
        try:
            while True:
                capture_and_save(host=args.host, output_dir=args.dir)
                time.sleep(args.loop)
        except KeyboardInterrupt:
            print("\n[+] Captura continua detenida por el usuario.")
    else:
        capture_and_save(host=args.host, output_dir=args.dir)
