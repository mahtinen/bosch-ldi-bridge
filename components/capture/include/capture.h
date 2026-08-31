/*
 * capture -- append-only flash log of BLE traffic and link events.
 *
 * WHY THIS EXISTS
 * ---------------
 * The bike is not next to the computer.  Without a log, diagnosing the LDI
 * link would mean carrying a laptop to the bike for every attempt.  With one,
 * the workflow becomes: plug the board into a power bank, take it to the bike,
 * watch the LED, bring it back, and read the whole session out over serial.
 *
 * That is also the shape the finished product takes anyway -- a bike accessory
 * on a power bank -- so this is not scaffolding that gets thrown away.
 *
 * DESIGN
 * ------
 * Strictly append-only into a dedicated 2.4 MB partition, with each record
 * written straight through to flash rather than buffered in RAM.  Losing power
 * mid-ride is the expected case, not an exception, so a RAM buffer would throw
 * away exactly the frames that matter.  Notification rates here are a few per
 * second, so per-record writes cost nothing.
 *
 * No ring buffer: 2.4 MB holds roughly 70,000 twenty-byte frames, about twenty
 * hours at 1 Hz. Wrapping would add power-loss failure modes to save capacity
 * nobody needs. When it fills, it stops and says so.
 *
 * Records are 4-byte aligned, 16-byte header plus payload:
 *
 *   u32 magic  'CAPR'
 *   u8  type   capture_rec_type_t
 *   u8  flags  type-specific
 *   u16 len    payload bytes
 *   u64 t_us   esp_timer_get_time() at capture
 *   u8  payload[len]
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAPTURE_MAGIC 0x52504143u /* 'CAPR' little-endian */

typedef enum {
    CAP_REC_BOOT      = 0, /* session marker; payload = version string   */
    CAP_REC_ADV       = 1, /* raw advertisement; flags = addr type       */
    CAP_REC_EVENT     = 2, /* text event line                            */
    CAP_REC_NOTIFY    = 3, /* notification payload; see capture_notify() */
    CAP_REC_GATT      = 4, /* discovered service/characteristic, as text */
    CAP_REC_LINKQUAL  = 5, /* MTU/DLE verification result, as text       */
} capture_rec_type_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t  type;
    uint8_t  flags;
    uint16_t len;
    uint64_t t_us;
} capture_hdr_t;

/* Mount the partition and find the append point.  Safe to call once. */
esp_err_t capture_init(void);

/* Is the log usable (partition found, space left)? */
bool capture_ready(void);

/* Bytes used / total, and record count seen at init. */
void capture_stats(size_t *used, size_t *total, uint32_t *records);

/* Append a raw record.  Returns ESP_ERR_NO_MEM when the partition is full. */
esp_err_t capture_write(capture_rec_type_t type, uint8_t flags,
                        const void *payload, uint16_t len);

/* printf-style text event. */
esp_err_t capture_event(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/*
 * A notification.  The attribute handle is prepended to the payload so the
 * dump can attribute frames to characteristics without a separate index.
 */
esp_err_t capture_notify(uint16_t attr_handle, const void *data, uint16_t len);

/* An advertisement, verbatim, with the address so it can be identified later. */
esp_err_t capture_adv(const uint8_t addr[6], uint8_t addr_type, int8_t rssi,
                      const uint8_t *data, uint8_t len);

/* Replay the whole log to stdout, hex + ASCII.  Blocks; console use only. */
void capture_dump(bool hex_payloads);

/* Erase the partition. Blocks for a second or two. */
esp_err_t capture_erase(void);

#ifdef __cplusplus
}
#endif
