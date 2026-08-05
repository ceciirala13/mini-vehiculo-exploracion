# Integración del sensor BME690 en ESP-IDF

Este paquete contiene el HAL del sensor BME690 listo para agregar a un proyecto ESP-IDF existente. **No es necesario modificar la estructura del proyecto**, solo seguir los pasos de abajo.

---

## Contenido del paquete

```
bme690_package/
├── bme690.h                  ← cabecera del HAL (API pública)
├── bme690.c                  ← implementación del HAL (I2C)
├── main.cpp                  ← ejemplo de uso (no reemplaza el tuyo)
└── BME690_SensorAPI/         ← librería oficial de Bosch
    ├── bme69x.c
    ├── bme69x.h
    └── bme69x_defs.h
```

---

## Conexiones físicas (ESP32-S3)

| Pin BME690 | GPIO ESP32-S3 | Nota                              |
|-----------|---------------|-----------------------------------|
| SDA       | GPIO05        |                                   |
| SCL       | GPIO06        |                                   |
| VDD       | 3.3 V         |                                   |
| GND       | GND           |                                   |
| SDO       | GND           | Define dirección I2C = 0x76       |
| CSB       | VDD           | Selecciona modo I2C (no SPI)      |

> Si el SDO de tu módulo está conectado a VDD en lugar de GND, cambia en `bme690.h`:
> ```c
> #define BME690_I2C_ADDR   BME69X_I2C_ADDR_HIGH   // 0x77
> ```

---

## Integración en el proyecto existente

### 1. Copiar los archivos

Copiar `bme690.h`, `bme690.c` y la carpeta `BME690_SensorAPI/` dentro del directorio `main/` de tu proyecto, o en el componente donde vayas a usarlos:

```
tu_proyecto/
└── main/
    ├── tu_main.cpp           ← tu archivo, no lo toques
    ├── bme690.h              ← agregar
    ├── bme690.c              ← agregar
    └── BME690_SensorAPI/     ← agregar carpeta completa
        ├── bme69x.c
        ├── bme69x.h
        └── bme69x_defs.h
```

### 2. Actualizar el CMakeLists.txt de main/

En tu `main/CMakeLists.txt` ya existente, agregar `bme690.c` y `bme69x.c` a la lista de fuentes, e incluir el directorio de la librería:

```cmake
idf_component_register(
    SRCS
        "tu_main.cpp"       # tus archivos existentes
        "bme690.c"          # ← agregar
        "BME690_SensorAPI/bme69x.c"   # ← agregar
    INCLUDE_DIRS
        "."
        "BME690_SensorAPI"  # ← agregar
    REQUIRES
        driver
        freertos
        esp_log
        # ... tus dependencias existentes
)
```

> Solo se modifican las listas `SRCS`, `INCLUDE_DIRS` y `REQUIRES`. El resto del `CMakeLists.txt` queda igual.

---

## Uso en tu código

En el archivo donde necesites leer el sensor, incluir la cabecera e invocar las funciones:

```c
#include "bme690.h"

// Una sola vez al inicio (por ejemplo en app_main o en tu tarea de inicialización):
esp_err_t err = bme690_init();
if (err != ESP_OK) {
    // manejar error
}

// En tu loop o tarea de lectura:
bme690_data_t data;
if (bme690_read_data(&data) == ESP_OK) {
    printf("Temp: %.2f °C | Hum: %.2f %% | Pres: %.2f hPa | Gas: %lu Ohm\n",
           data.temperature,
           data.humidity,
           data.pressure,
           (unsigned long)data.gas_resistance);
}
```

El archivo `main.cpp` incluido en el paquete es solo un ejemplo de referencia con el flujo completo. No es necesario usarlo; toda la lógica relevante ya está en `bme690.c` y `bme690.h`.

---

## Datos que entrega cada lectura

| Campo            | Tipo       | Unidad  | Descripción                              |
|-----------------|------------|---------|------------------------------------------|
| `temperature`   | `float`    | °C      | Temperatura compensada                   |
| `humidity`      | `float`    | %       | Humedad relativa compensada              |
| `pressure`      | `float`    | hPa     | Presión barométrica compensada           |
| `gas_resistance`| `uint32_t` | Ω       | Resistencia del sensor de gas (VOC)      |
| `gas_valid`     | `uint8_t`  | 0/1     | 1 = medición de gas válida               |
| `heater_stable` | `uint8_t`  | 0/1     | 1 = calentador alcanzó temperatura meta  |

> Leer `gas_resistance` solo tiene sentido cuando **ambos** `gas_valid` y `heater_stable` son `1`.

---

## Notas adicionales

- El bus I2C se inicializa dentro de `bme690_init()`. Si tu proyecto ya inicializa el `I2C_NUM_0` en otro lugar, cambia `BME690_I2C_PORT` a `I2C_NUM_1` en `bme690.h` y asegurate de que los pines coincidan.
- El sensor opera en **modo forzado**: cada llamada a `bme690_read_data()` dispara una medición individual. No es necesario ningún timer externo.
- El tiempo de bloqueo por llamada es aproximadamente **200–250 ms** (tiempo de medición + calentador de gas). Tenerlo en cuenta si se llama desde una tarea con restricciones de tiempo.
