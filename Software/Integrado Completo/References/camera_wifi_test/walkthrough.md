# Walkthrough: Servidor Web Wi-Fi AP con ArduCAM OV2640 en ESP32-C6

Se ha completado la integración nativa en **ESP-IDF** de la cámara **ArduCAM Mini 2MP Plus (OV2640)** en el proyecto [camera_wifi_test](file:///c:/Users/sanie/Documents/GitHub/mini-vehiculo-exploracion/Software/Pruebas_Funcionamiento/test_camera/camera_wifi_test).

El sistema opera en **Modo Punto de Acceso Wi-Fi (AP)** con un **Servidor HTTP plano en el puerto 80**, permitiendo capturar fotos, visualizarlas en el panel de control web y descargarlas a una carpeta de la computadora **sin interferir con el puerto USB-Serial ni el flasheo**.

---

## 📁 Archivos Creados y Modificados

| Archivo | Tipo | Descripción |
| :--- | :--- | :--- |
| [`arducam_ov2640.h`](file:///c:/Users/sanie/Documents/GitHub/mini-vehiculo-exploracion/Software/Pruebas_Funcionamiento/test_camera/camera_wifi_test/main/arducam_ov2640.h) | **NUEVO** | Encabezado del driver nativo ESP-IDF con APIs de inicialización, verificación SPI (`0x55`), detección I2C (VID `0x26`, PID `0x41`/`0x42`), resoluciones y lectura FIFO. |
| [`arducam_ov2640.c`](file:///c:/Users/sanie/Documents/GitHub/mini-vehiculo-exploracion/Software/Pruebas_Funcionamiento/test_camera/camera_wifi_test/main/arducam_ov2640.c) | **NUEVO** | Implementación en C puro con `driver/spi_master.h` y `driver/i2c.h` para control de la cámara y lectura FIFO de alta velocidad con DMA. |
| [`ov2640_regs.h`](file:///c:/Users/sanie/Documents/GitHub/mini-vehiculo-exploracion/Software/Pruebas_Funcionamiento/test_camera/camera_wifi_test/main/ov2640_regs.h) | **NUEVO** | Tablas de inicialización de registros OV2640 portadas desde `Arduino-master` a C estándar. |
| [`main.c`](file:///c:/Users/sanie/Documents/GitHub/mini-vehiculo-exploracion/Software/Pruebas_Funcionamiento/test_camera/camera_wifi_test/main/main.c) | **MODIFICADO** | Inicialización de Wi-Fi en modo AP (`ROVER_EXPLORER_AP`), servidor HTTP en puerto 80, y endpoints `/`, `/capture`, `/stream`, `/resolution`, `/status`. |
| [`index.html`](file:///c:/Users/sanie/Documents/GitHub/mini-vehiculo-exploracion/Software/Pruebas_Funcionamiento/test_camera/camera_wifi_test/main/index.html) | **MODIFICADO** | Interfaz web del Rover con tarjeta de cámara HD, visor en vivo, botón de captura, botón de guardado en PC, selector de resolución y streaming. |
| [`CMakeLists.txt`](file:///c:/Users/sanie/Documents/GitHub/mini-vehiculo-exploracion/Software/Pruebas_Funcionamiento/test_camera/camera_wifi_test/main/CMakeLists.txt) | **MODIFICADO** | Registro de fuentes `arducam_ov2640.c`, componentes `esp_http_server`, `driver` e incrustación de `index.html`. |
| [`download_photos.py`](file:///c:/Users/sanie/Documents/GitHub/mini-vehiculo-exploracion/Software/Pruebas_Funcionamiento/test_camera/download_photos.py) | **NUEVO** | Script Python para solicitar fotos a `http://192.168.4.1/capture` y guardarlas automáticamente en `./images/`. |
| [`context.md`](file:///c:/Users/sanie/Documents/GitHub/mini-vehiculo-exploracion/Software/Pruebas_Funcionamiento/test_camera/context.md) | **ACTUALIZADO** | Documento técnico de contexto completo. |

---

## ⚙️ Conexión de Hardware (Pinout)

| Pin ArduCAM 2MP Plus | Pin ESP32-C6 | Función |
| :--- | :--- | :--- |
| **VCC** | 3.3V / 5V | Alimentación de la cámara |
| **GND** | GND | Tierra común |
| **CS** | GPIO 7 | SPI Chip Select (Control de FIFO/CPLD) |
| **MOSI** | GPIO 23 | SPI Master Out Slave In |
| **MISO** | GPIO 22 | SPI Master In Slave Out |
| **SCK** | GPIO 21 | SPI Clock |
| **SDA** | GPIO 4 | I2C Data (SCCB configuración del sensor) |
| **SCL** | GPIO 5 | I2C Clock (SCCB configuración del sensor) |

---

## 🚀 Cómo Compilar y Flashear el Firmware

En la terminal de ESP-IDF (dentro del directorio `camera_wifi_test`):

```bash
cd camera_wifi_test
idf.py set-target esp32c6
idf.py build
idf.py -p COM_PORT flash monitor
```

---

## 🌐 Cómo Probar el Sistema

### Paso 1: Conexión Wi-Fi
1. En tu computadora o smartphone, busca las redes Wi-Fi disponibles.
2. Conéctate a la red creada por el ESP32:
   * **SSID:** `ROVER_EXPLORER_AP`
   * **Contraseña:** `rover1234`

### Paso 2: Acceso al Panel de Control Web
Abre tu navegador (Chrome, Firefox, Edge) e ingresa a:
👉 `http://192.168.4.1`

Podrás:
* Ver el visor de la cámara en vivo o tomar fotos individuales.
* Cambiar la resolución (QVGA, VGA, SXGA, UXGA 2MP).
* Presionar **"💾 Guardar en PC"** para descargar el archivo `.jpg` directamente al disco.
* Controlar los motores y visualizar la telemetría.

### Paso 3: Almacenamiento Automático de Fotos en Carpeta Local (Python)
Para descargar fotos automáticamente en la carpeta `images/` de tu computadora:

```bash
# Foto única
python download_photos.py

# Ráfaga periódica (ejemplo: cada 3 segundos)
python download_photos.py --loop 3

# Captura en resolución 2MP (1600x1200)
python download_photos.py --res 8
```
Las fotos se guardarán automáticamente como `images/rover_capture_YYYYMMDD_HHMMSS.jpg`.
