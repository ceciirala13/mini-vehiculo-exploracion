# ESP32-C6 Network & Telemetry Bridge (ESP-IDF)

Este subproyecto implementa el puente de comunicaciones de alta velocidad y telemetría inercial para el **ESP32-C6** utilizando **ESP-IDF v5.x**.

---

## 📡 Funciones del Firmware

1. **Wi-Fi SoftAP**:
   - **SSID**: `Rover_WiFi_AP`
   - **Password**: `rover1234`
   - **IP Servidor**: `192.168.4.1`
   - **Puerto TCP**: `8080`

2. **Enlace Serial UART1 hacia ESP32-S3**:
   - **Baudrate**: `921600` baud
   - **TX Pin (ESP32-C6)**: `GPIO 17` $\rightarrow$ Conectar a RX (`GPIO 44`) del ESP32-S3
   - **RX Pin (ESP32-C6)**: `GPIO 16` $\rightarrow$ Conectar a TX (`GPIO 43`) del ESP32-S3
   - **GND**: Conectar GND de ambos ESP32 juntos.

3. **Sensor Inercial IMU MPU6050 (I2C)**:
   - **SDA Pin**: `GPIO 21`
   - **SCL Pin**: `GPIO 22`
   - **Frecuencia I2C**: `400 kHz`
   - **Frecuencia de Muestreo**: `25 Hz` (40 ms) transmitidos por TCP hacia la PC.

---

## 🚀 Instrucciones de Compilación y Flasheo

Abre la terminal de **ESP-IDF (ESP-IDF 5.x CMD / PowerShell)**:

### 1. Navegar a esta carpeta (o mover la carpeta fuera):
```bash
cd esp32c6_bridge
```

### 2. Establecer el Target a ESP32-C6:
```bash
idf.py set-target esp32c6
```

### 3. Compilar el proyecto:
```bash
idf.py build
```

### 4. Flashear y Monitorear (reemplazar `COMx` por el puerto COM del ESP32-C6):
```bash
idf.py -p COMx flash monitor
```
*(Para salir del monitor pulsa `Ctrl + ]`)*
