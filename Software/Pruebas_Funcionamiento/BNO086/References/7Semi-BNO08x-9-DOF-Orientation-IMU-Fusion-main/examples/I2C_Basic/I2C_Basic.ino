/*  I2C_Basic.ino  —  7Semi BNO08x (SHTP) example over I²C
 *
 *  What this does
 *  - Creates an I²C transport (default 0x4B, 400 kHz)
 *  - Starts the BNO08x
 *  - Enables several sensor reports at ~100 Hz
 *  - In loop(), calls processData() to read+decode packets (if any)
 *  - Prints the most recent values every 250 ms
 *
 *  Wiring (typical 3V3 MCU)
 *    VDD -> 3V3
 *    GND -> GND
 *    SDA -> MCU SDA
 *    SCL -> MCU SCL
 *    PS0 -> LOW
 *    PS1 -> LOW
 *
 *  Notes
 *  - If your board is at 0x4A instead of 0x4B, change the constructor below.
 */
#define BNO_USE_I2C
#include <7Semi_BNO08x.h>  // main driver (BNO08x_7Semi + SH2_* IDs)


// ---------- Transport & device ----------
// Many BNO08x boards use 0x4B (sometimes 0x4A). Start at 400 kHz.
//For Arduino
int8_t BNO_SDA = -1;
int8_t BNO_SCL = -1;
uint32_t BNO_clock = 400000;
// int8_t BNO_SDA = 21;
// int8_t BNO_SCL = 22;
constexpr int PIN_INTN = 9;  // Optional for I2C
constexpr int PIN_RST = 8;   // Optional for I2c
static BnoI2CBus bus(Wire, BNO_SDA, BNO_SCL, 0x4B, BNO_clock, PIN_INTN,
                     PIN_RST);
static BNO08x_7Semi bno(bus);

struct ImuData {
  float ax, ay, az;      // Accelerometer
  float gx, gy, gz;      // Gyroscope
  float mx, my, mz;      // Magnetometer
  float qi, qj, qk, qr;  // Rotation Vector
  float lax, lay, laz;   // Linear acceleration
};
ImuData d;
static uint32_t t = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait for USB */
  }
  Serial.println(F("\n[7Semi BNO08x — I2C_Basic]"));

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
