/**
 * @file rover_proto.c
 * @brief Binary UART protocol encoder and stream parser.
 *
 * !!! KEEP THIS FILE IDENTICAL IN MCU_CONTROL/main AND MCU_COMM/main !!!
 */

#include "rover_proto.h"
#include <string.h>

enum {
    ST_SOF0 = 0,
    ST_SOF1,
    ST_ID,
    ST_SEQ,
    ST_LEN,
    ST_PAYLOAD,
    ST_CRC_L,
    ST_CRC_H,
};

uint16_t rp_crc16(const uint8_t *data, size_t len, uint16_t crc)
{
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

size_t rp_encode(uint8_t msg_id, uint8_t seq, const void *payload, uint8_t len,
                 uint8_t *out, size_t out_size)
{
    size_t total = RP_HEADER_LEN + len + RP_CRC_LEN;
    if (len > RP_MAX_PAYLOAD || out == NULL || out_size < total) {
        return 0;
    }
    if (len > 0 && payload == NULL) {
        return 0;
    }

    out[0] = RP_SOF0;
    out[1] = RP_SOF1;
    out[2] = msg_id;
    out[3] = seq;
    out[4] = len;
    if (len > 0) {
        memcpy(&out[RP_HEADER_LEN], payload, len);
    }

    /* CRC over ID, SEQ, LEN, PAYLOAD */
    uint16_t crc = rp_crc16(&out[2], 3 + len, 0xFFFF);
    out[RP_HEADER_LEN + len]     = (uint8_t)(crc & 0xFF);
    out[RP_HEADER_LEN + len + 1] = (uint8_t)(crc >> 8);
    return total;
}

void rp_parser_init(rp_parser_t *p)
{
    memset(p, 0, sizeof(*p));
    p->state = ST_SOF0;
}

static uint16_t frame_crc(const rp_frame_t *f)
{
    uint8_t hdr[3] = { f->msg_id, f->seq, f->len };
    uint16_t crc = rp_crc16(hdr, sizeof(hdr), 0xFFFF);
    return rp_crc16(f->payload, f->len, crc);
}

bool rp_parser_feed(rp_parser_t *p, uint8_t byte, uint32_t now_ms)
{
    /* Drop a stale partial frame */
    if (p->state != ST_SOF0 && (uint32_t)(now_ms - p->last_byte_ms) > RP_INTERBYTE_TIMEOUT_MS) {
        p->timeouts++;
        p->state = ST_SOF0;
    }
    p->last_byte_ms = now_ms;

    switch (p->state) {
    case ST_SOF0:
        if (byte == RP_SOF0) {
            p->state = ST_SOF1;
        }
        break;

    case ST_SOF1:
        if (byte == RP_SOF1) {
            p->state = ST_ID;
        } else if (byte != RP_SOF0) {
            p->state = ST_SOF0;
        }
        break;

    case ST_ID:
        p->frame.msg_id = byte;
        p->state = ST_SEQ;
        break;

    case ST_SEQ:
        p->frame.seq = byte;
        p->state = ST_LEN;
        break;

    case ST_LEN:
        if (byte > RP_MAX_PAYLOAD) {
            p->len_errors++;
            p->state = ST_SOF0;
        } else {
            p->frame.len = byte;
            p->idx = 0;
            p->state = (byte == 0) ? ST_CRC_L : ST_PAYLOAD;
        }
        break;

    case ST_PAYLOAD:
        p->frame.payload[p->idx++] = byte;
        if (p->idx >= p->frame.len) {
            p->state = ST_CRC_L;
        }
        break;

    case ST_CRC_L:
        p->crc_rx = byte;
        p->state = ST_CRC_H;
        break;

    case ST_CRC_H:
        p->crc_rx |= (uint16_t)byte << 8;
        p->state = ST_SOF0;
        if (p->crc_rx == frame_crc(&p->frame)) {
            p->frames_ok++;
            return true;
        }
        p->crc_errors++;
        break;

    default:
        p->state = ST_SOF0;
        break;
    }
    return false;
}
