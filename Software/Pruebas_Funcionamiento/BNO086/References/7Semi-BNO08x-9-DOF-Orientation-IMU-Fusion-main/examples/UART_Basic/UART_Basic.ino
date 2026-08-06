/*  UART_Basic.ino  —  7Semi BNO08x (SHTP) example over UART
 *
 *  What this does
 *  - Creates a UART transport (default 3,000,000 baud, 8N1)
 *  - Starts the BNO08x
 *  - Enables several sensor reports at ~100 Hz
 *  - In loop(), calls processData() to read+decode packets (if any)
 *  - Prints the most recent values every 250 ms
 *
 *  Wiring (3V3 logic — TX/RX are *crossed*)
 *    BNO08x TX(MISO) -> MCU RX (e.g., GPIO16 on ESP32)
 *    BNO08x RX(SCL)  -> MCU TX (e.g., GPIO17 on ESP32)
 *    VDD             -> 3V3
 *    GND             -> GND
 *    PS0             -> LOW
 *    PS1             -> HIGH
 *
 *  Notes
 *  - Default baud here is 3,000,000 (3M). Lower to 1,000,000 if your board/cable is noisy.
 *  - On ESP32 you can provide explicit RX/TX pins in Serial1.begin(…, RX, TX).
 *  - Your library must be compiled with UART selected via BNO_USE_UART.
 */

#define BNO_USE_UART
#include <7Semi_BNO08x.h>  // must come AFTER the define above

// ---------- UART pins & serial params ----------
static const int UART_RX_PIN = 16;  // MISO
static const int UART_TX_PIN = 17;  // SCL

static const uint32_t UART_BAUD = 3000000UL;  // 3,000,000 baud

struct ImuData {
  float ax, ay, az;      // Accelerometer
  float gx, gy, gz;      // Gyroscope
  float mx, my, mz;      // Magnetometer
  float qi, qj, qk, qr;  // Rotation Vector
  float lax, lay, laz;   // Linear acceleration
};

ImuData d;
static uint32_t t = 0;
// ---------- Transport & device ----------
BnoSelectedBus bnoBus(Serial1, UART_BAUD);  // BnoSelectedBus aliases to BnoUARTBus with BNO_USE_UART
BNO08x_7Semi bno(bnoBus);

// Desired output data rate (ms)
static const uint32_t ODR_MS = 10;  // 100 Hz

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println(F("\n[7Semi BNO08x — UART_Basic]"));

  // Start Serial1 at the desired UART settings.
#if defined(ARDUINO_ARCH_ESP32)
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
#else
  Serial1.begin(UART_BAUD);
#endif


  if (!bno.begin()) {
    Serial.println(" BNO08x initialization failed");
    while (1) delay(1000);
  }

  bno.enableAcc(20);
  bno.enableGyro(20);
  bno.enableMag(50);
  bno.enableRotationVector(20);
  bno.enableLinearAccel(20);
  
}

void loop() {
  bno.processData();  // Always run as fast as possible
  if (millis() - t > 250) {
         t = millis();
    if (bno.getAccelerometer(d.ax, d.ay, d.az)) {
      Serial.print("ACC : ");
      Serial.print(d.ax, 2);
      Serial.print(' ');
      Serial.print(d.ay, 2);
      Serial.print(' ');
      Serial.print(d.az, 2);
      Serial.println(" m/s²");
    }
    if (bno.getGyroscope(d.gx, d.gy, d.gz)) {
      Serial.print("GYRO: ");
      Serial.print(d.gx, 2);
      Serial.print(' ');
      Serial.print(d.gy, 2);
      Serial.print(' ');
      Serial.print(d.gz, 2);
      Serial.println(" rad/s");
    }
    if (bno.getMagnetometer(d.mx, d.my, d.mz)) {
      Serial.print("MAG : ");
      Serial.print(d.mx, 1);
      Serial.print(' ');
      Serial.print(d.my, 1);
      Serial.print(' ');
      Serial.print(d.mz, 1);
      Serial.println(" uT");
    }
    if (bno.getQuaternion(d.qi, d.qj, d.qk, d.qr)) {
      Serial.print("RV  : ");
      Serial.print(d.qi, 3);
      Serial.print(' ');
      Serial.print(d.qj, 3);
      Serial.print(' ');
      Serial.print(d.qk, 3);
      Serial.print(' ');
      Serial.println(d.qr, 3);
    }
    if (bno.getLinearAccel(d.lax, d.lay, d.laz)) {
      Serial.print("LIN : ");
      Serial.print(d.lax, 2);
      Serial.print(' ');
      Serial.print(d.lay, 2);
      Serial.print(' ');
      Serial.print(d.laz, 2);
      Serial.println(" m/s²");
    }
     Serial.print('\n');
  }
}
