#ifndef MPU6050_C6_H
#define MPU6050_C6_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Registro y Dirección I2C del MPU6050
#define MPU6050_I2C_ADDR       0x68
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_ACCEL_XOUT 0x3B
#define MPU6050_REG_WHO_AM_I   0x75

// Estructura de telemetría inercial (compatible con protocolo binario PC Server)
#pragma pack(push, 1)
typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
    float roll, pitch;
} imu_pkt_t;
#pragma pack(pop)

// Delimitadores del protocolo binario
#define PKT_MAGIC_0  0xAA
#define PKT_MAGIC_1  0x55
#define PKT_TYPE_IMU 0x05

#ifdef __cplusplus
}
#endif

#endif // MPU6050_C6_H
