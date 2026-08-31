/*
 * ldi_uuids.h -- what the bike actually advertises.
 *
 * The values below were obtained EMPIRICALLY from the capture log, not from
 * the specification PDF (which is still not in hand). They are facts observed
 * on this bike, a Bosch Performance Line CX Gen5 with a Purion 200 / BRC3800
 * control unit. Confirm against the spec when it arrives.
 *
 * Raw advertisement, 27 bytes, captured at -56 dBm from a PUBLIC address:
 *
 *   00:04:63:xx:xx:xx type=0
 *   02 01 05                            Flags = 0x05
 *   03 02 02 fe                         Incomplete 16-bit svc UUIDs: 0xFE02
 *   13 09 "smart system eBike"          Complete Local Name
 *
 * Decoded:
 *   Flags 0x05 = LE Limited Discoverable Mode | BR/EDR Not Supported.
 *     Limited (not General) discoverable mode is the tell that the bike is in
 *     PAIRING mode -- it does not advertise this way the rest of the time.
 *   0xFE02 sits in the SIG's 16-bit "member service" range, allocated to
 *     member companies. This is the correction that matters: the LDI service
 *     is NOT a 128-bit custom UUID. Assuming it was is what made the
 *     autonomous hunt reject the bike on every scan, so nothing ever connected
 *     and the eBike Flow app reported "pairing failed" while waiting.
 *   AD type 0x02 is the INCOMPLETE list, so the bike has further services it
 *     does not advertise. The full GATT map comes from discovery after
 *     connecting, which is what Phase 2 does.
 */
#pragma once

#include <string.h>

/* ---- Identification, from the captured advertisement ----------------- */

/* Advertised 16-bit service UUID. Bosch smart system, SIG member range. */
#define LDI_ADV_SVC_UUID16      0xFE02

/* Complete Local Name, verbatim. Matched as a substring so a firmware revision
 * that appends or prefixes something still matches. */
#define LDI_ADV_NAME            "smart system eBike"

/* Public-address OUI observed on the control unit. Used only as a weak
 * corroborating signal -- OUIs vary across hardware revisions. */
#define LDI_OUI_0               0x00
#define LDI_OUI_1               0x04
#define LDI_OUI_2               0x63

/* GAP flags seen while the bike is in pairing mode. */
#define LDI_ADV_FLAGS_PAIRING   0x05 /* LE Limited Discoverable | BR/EDR unsup */

/* ---- Discovered GATT map (see docs/LDI-PROTOCOL.md) --------------------- */

/*
 * The advertised 0xFE02 is only the discovery hook. Every real service is
 * 128-bit and unadvertised, sharing this base:
 *
 *     xxxxxxxx-eaa2-11e9-81b4-2a2ae2dbcce4
 *
 * Observed services: 00000010 (control), 0000eb40, 0000eb20 (LIVE DATA),
 * 0000eb10, 0000ebd0, 00000040, 0000eba0, 00000020, 0000eb60.
 *
 * The protobuf live-data stream arrives on the notifiable characteristic of
 * 0000eb20 -- attribute handle 40 on this firmware, though handles are not
 * durable across revisions, which is why discovery stays in the flow rather
 * than hardcoding them.
 */
#define LDI_UUID_BASE_SUFFIX    "-eaa2-11e9-81b4-2a2ae2dbcce4"

/* Short forms of the observed services, for reference and logging. */
#define LDI_SVC_CONTROL_SHORT   0x00000010u
#define LDI_SVC_LIVEDATA_SHORT  0x0000eb20u
#define LDI_CHR_LIVEDATA_SHORT  0x0000eb21u  /* the one LiveData characteristic */

/* TODO(spec): confirm which characteristic UUID inside 0000eb20 carries the
 * stream. The first capture lost the characteristic UUIDs to a static-buffer
 * aliasing bug in our own logging (now fixed); the next session records them. */

/*
 * ANSWERED by the capture: notifications are PARTIAL UPDATES, not full
 * snapshots. The first frame after subscribing carries everything (80 bytes),
 * then subsequent frames carry only what changed (51-57 bytes).
 *
 * So the decoder must MERGE into existing state and never replace it: a field
 * absent from a frame means "unchanged", not "zero". The per-field timestamps
 * and valid_mask already in components/bike_state/ are exactly what this needs.
 *
 * Field 11 is a Unix timestamp (incremented by exactly 1 per second across
 * consecutive frames). Field 12 and 15 look like an odometer in metres.
 *
 * TODO: identify power, cadence and speed. The bike was stationary for this
 * capture, so all three were zero and mutually indistinguishable. Needs a
 * capture taken WHILE PEDALLING -- see docs/LDI-PROTOCOL.md.
 */

/* Bump when the field interpretation changes, so logs record which reading
 * produced them. */
#define LDI_SCHEMA_VERSION 1

/*
 * Does this UUID equal ebike_uuid(short)?
 *
 * Matching by UUID rather than by attribute handle matters: handles are chosen
 * by the server and are not durable across firmware revisions, while the UUIDs
 * are fixed by the specification.
 */
#include <stdbool.h>
#include "host/ble_uuid.h"

static inline bool ldi_uuid_is(const ble_uuid_t *u, uint32_t short_form)
{
    /* 0000xxxx-eaa2-11e9-81b4-2a2ae2dbcce4, little-endian in ble_uuid128_t. */
    static const uint8_t base[16] = {
        0xe4, 0xcc, 0xdb, 0xe2, 0x2a, 0x2a, 0xb4, 0x81,
        0xe9, 0x11, 0xa2, 0xea, 0x00, 0x00, 0x00, 0x00,
    };
    if (u->type != BLE_UUID_TYPE_128) {
        return false;
    }
    const ble_uuid128_t *u128 = (const ble_uuid128_t *)u;
    if (memcmp(u128->value, base, 12) != 0) {
        return false;
    }
    uint32_t got = (uint32_t)u128->value[12] |
                   ((uint32_t)u128->value[13] << 8) |
                   ((uint32_t)u128->value[14] << 16) |
                   ((uint32_t)u128->value[15] << 24);
    return got == short_form;
}
