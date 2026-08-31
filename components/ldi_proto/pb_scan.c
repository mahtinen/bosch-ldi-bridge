#include "pb_scan.h"

void pb_scan_init(pb_scan_t *s, const uint8_t *buf, size_t len)
{
    s->buf = buf;
    s->len = len;
    s->pos = 0;
}

static bool read_varint(pb_scan_t *s, uint64_t *out)
{
    uint64_t v = 0;
    unsigned shift = 0;

    while (s->pos < s->len) {
        uint8_t b = s->buf[s->pos++];
        v |= (uint64_t)(b & 0x7f) << shift;
        if ((b & 0x80) == 0) {
            *out = v;
            return true;
        }
        shift += 7;
        if (shift >= 64) {
            return false; /* overlong: malformed */
        }
    }
    return false; /* ran off the end mid-varint */
}

bool pb_scan_next(pb_scan_t *s, pb_field_t *out)
{
    uint64_t tag;

    if (s->pos >= s->len) {
        return false;
    }
    if (!read_varint(s, &tag)) {
        return false;
    }

    out->field = (uint32_t)(tag >> 3);
    out->wire  = (pb_wire_t)(tag & 0x07);
    out->data  = NULL;
    out->data_len = 0;
    out->varint = 0;

    if (out->field == 0) {
        return false; /* field 0 is invalid; treat as end of usable data */
    }

    switch (out->wire) {
    case PB_WIRE_VARINT:
        return read_varint(s, &out->varint);

    case PB_WIRE_64BIT:
        if (s->len - s->pos < 8) {
            return false;
        }
        for (int i = 0; i < 8; i++) {
            out->varint |= (uint64_t)s->buf[s->pos + i] << (8 * i);
        }
        s->pos += 8;
        return true;

    case PB_WIRE_32BIT:
        if (s->len - s->pos < 4) {
            return false;
        }
        for (int i = 0; i < 4; i++) {
            out->varint |= (uint64_t)s->buf[s->pos + i] << (8 * i);
        }
        s->pos += 4;
        return true;

    case PB_WIRE_LEN: {
        uint64_t n;
        if (!read_varint(s, &n)) {
            return false;
        }
        if (n > s->len - s->pos) {
            return false;
        }
        out->data     = &s->buf[s->pos];
        out->data_len = (size_t)n;
        s->pos += (size_t)n;
        return true;
    }

    default:
        /* Wire types 3 and 4 (start/end group) are proto2-only and removed in
         * proto3. Their length cannot be determined without a schema, so the
         * scan cannot continue past one. */
        return false;
    }
}

int32_t pb_as_int32(uint64_t v)
{
    return (int32_t)(uint32_t)v;
}

int64_t pb_as_int64(uint64_t v)
{
    return (int64_t)v;
}
