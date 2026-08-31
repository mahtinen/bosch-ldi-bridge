/*
 * pb_scan -- minimal protobuf wire-format reader.
 *
 * Knows nothing about any schema: it walks tag/value pairs and lets the caller
 * switch on field number. That is deliberate, and it is what the LDI spec asks
 * for -- section 2.2.4.3 requires clients to ignore unrecognised fields so the
 * server can add new ones without breaking them. A generated parser would be
 * stricter than the protocol wants.
 *
 * Decode-only. LDI is read-only, so there is no encoder here.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PB_WIRE_VARINT = 0,
    PB_WIRE_64BIT  = 1,
    PB_WIRE_LEN    = 2,
    PB_WIRE_32BIT  = 5,
} pb_wire_t;

typedef struct {
    const uint8_t *buf;
    size_t         len;
    size_t         pos;
} pb_scan_t;

typedef struct {
    uint32_t  field;
    pb_wire_t wire;
    uint64_t  varint;   /* valid for VARINT, 64BIT, 32BIT */
    const uint8_t *data; /* valid for LEN */
    size_t    data_len;
} pb_field_t;

void pb_scan_init(pb_scan_t *s, const uint8_t *buf, size_t len);

/*
 * Read the next field. Returns false at the end of the buffer or on malformed
 * input -- the two are deliberately not distinguished, because a truncated
 * notification and a corrupt one call for the same response: use what was
 * parsed and discard the rest.
 */
bool pb_scan_next(pb_scan_t *s, pb_field_t *out);

/* Signed helpers. Protobuf int32/int64 are plain varints (NOT zigzag); only
 * sint32/sint64 use zigzag, and LDI uses neither. */
int32_t pb_as_int32(uint64_t v);
int64_t pb_as_int64(uint64_t v);

#ifdef __cplusplus
}
#endif
