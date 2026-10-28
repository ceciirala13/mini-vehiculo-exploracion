/**
 * @file rover_proto.h
 * @brief Binary UART protocol shared by MCU_CONTROL (ESP32-S3) and MCU_COMM (ESP32).
 *
 * !!! KEEP THIS FILE IDENTICAL IN MCU_CONTROL/main AND MCU_COMM/main !!!
 * (bump ROVER_PROTO_VERSION on any change to frames or payload structs)
 *
 * Frame layout (little-endian):
 *
 *   | 0xAA | 0x55 | MSG_ID | SEQ | LEN | PAYLOAD[LEN] | CRC16_L | CRC16_H |
 *
 *   - CRC16-CCITT (poly 0x1021, init 0xFFFF) over MSG_ID, SEQ, LEN, PAYLOAD.
 *   - LEN is 0..RP_MAX_PAYLOAD. Frames are delimited by LEN + CRC (no end byte);
 *     a bad CRC / timeout makes the parser resync on the next 0xAA 0x55.
 *
 * Plain C, no ESP-IDF dependency (unit-testable on a PC).
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROVER_PROTO_VERSION         1

#define RP_SOF0                     0xAA
#define RP_SOF1                     0x55
#define RP_HEADER_LEN               5       /* SOF0 SOF1 ID SEQ LEN */
#define RP_CRC_LEN                  2
#define RP_MAX_PAYLOAD              96
#define RP_MAX_FRAME                (RP_HEADER_LEN + RP_MAX_PAYLOAD + RP_CRC_LEN)
#define RP_INTERBYTE_TIMEOUT_MS     20      /* partial frame is dropped after this gap */

/* =========================================================================
 * Message IDs
 * ========================================================================= */
typedef enum {
    RP_MSG_CMD_DRIVE   = 0x01,  /* COMM -> CTRL, rp_cmd_drive_t, fixed 20 Hz (also heartbeat) */
    RP_MSG_CMD_CONFIG  = 0x02,  /* COMM -> CTRL, reserved for runtime tuning */
    RP_MSG_TLM_STATE   = 0x10,  /* CTRL -> COMM, rp_tlm_state_t, 20 Hz */
} rp_msg_id_t;

/* =========================================================================
 * Payloads
 * ========================================================================= */

/* rp_cmd_drive_t.flags */
#define RP_DRIVE_FLAG_ESTOP         0x01

typedef struct __attribute__((packed)) {
    int16_t speed_x10;          /* -1000..1000  => -100.0..+100.0 % (+ = forward) */
    int16_t steer_x10;          /* -450..450    => -45.0..+45.0 deg (+ = right)   */
    uint8_t flags;              /* RP_DRIVE_FLAG_* */
} rp_cmd_drive_t;

/* rp_tlm_state_t.status bits */
#define RP_ST_MPU_OK                0x01
#define RP_ST_BME_OK                0x02
#define RP_ST_MOTOR_FAULT           0x04
#define RP_ST_FAILSAFE              0x08
#define RP_ST_ARMED                 0x10
#define RP_ST_GAS_VALID             0x20
#define RP_ST_LINK_OK               0x40    /* CTRL is receiving valid commands */

/* rp_tlm_state_t.motion_state */
typedef enum {
    RP_MOTION_DISARMED = 0,
    RP_MOTION_ARMED    = 1,
    RP_MOTION_FAILSAFE = 2,
    RP_MOTION_FAULT    = 3,
} rp_motion_state_t;

typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;

    /* MPU6050 */
    float    ax_g, ay_g, az_g;
    float    gx_dps, gy_dps, gz_dps;
    float    roll_deg, pitch_deg, yaw_deg;

    /* BME690 */
    float    temp_c;
    float    hum_pct;
    float    press_hpa;
    uint32_t gas_ohm;

    /* Actuators (actual, after ramps) */
    int16_t  motor_left_x10;    /* -1000..1000 % x10 */
    int16_t  motor_right_x10;
    int16_t  steer_x10;         /* -450..450 deg x10 */

    uint8_t  status;            /* RP_ST_* */
    uint8_t  motion_state;      /* rp_motion_state_t */

    /* Link diagnostics as seen by CTRL */
    uint16_t rx_ok;
    uint16_t rx_crc_err;

    uint8_t  proto_version;
} rp_tlm_state_t;

_Static_assert(sizeof(rp_cmd_drive_t) <= RP_MAX_PAYLOAD, "drive payload too big");
_Static_assert(sizeof(rp_tlm_state_t) <= RP_MAX_PAYLOAD, "telemetry payload too big");
/* Wire sizes are fixed: tools/rover_client.py and the other MCU depend on them */
_Static_assert(sizeof(rp_cmd_drive_t) == 5, "rp_cmd_drive_t wire size changed");
_Static_assert(sizeof(rp_tlm_state_t) == 69, "rp_tlm_state_t wire size changed");

/* =========================================================================
 * Encoder
 * ========================================================================= */

uint16_t rp_crc16(const uint8_t *data, size_t len, uint16_t crc);

/**
 * @brief Build a complete frame into @p out.
 * @return Frame length in bytes, or 0 if @p len is too big / @p out too small.
 */
size_t rp_encode(uint8_t msg_id, uint8_t seq, const void *payload, uint8_t len,
                 uint8_t *out, size_t out_size);

/* =========================================================================
 * Stream parser (byte-wise state machine)
 * ========================================================================= */

typedef struct {
    uint8_t msg_id;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[RP_MAX_PAYLOAD];
} rp_frame_t;

typedef struct {
    uint8_t    state;
    uint8_t    idx;
    uint16_t   crc_rx;
    uint32_t   last_byte_ms;
    rp_frame_t frame;           /* valid after rp_parser_feed() returns true */

    /* statistics */
    uint32_t   frames_ok;
    uint32_t   crc_errors;
    uint32_t   len_errors;
    uint32_t   timeouts;
} rp_parser_t;

void rp_parser_init(rp_parser_t *p);

/**
 * @brief Feed one received byte.
 * @param now_ms Monotonic time in ms (for the inter-byte timeout).
 * @return true when a complete, CRC-valid frame is available in p->frame.
 */
bool rp_parser_feed(rp_parser_t *p, uint8_t byte, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
