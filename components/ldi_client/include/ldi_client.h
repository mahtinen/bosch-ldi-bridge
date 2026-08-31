/*
 * ldi_client -- BLE central toward the Bosch eBike.
 *
 * PHASE 2 SCOPE: discover and dump.  This deliberately knows NO UUIDs.
 *
 * The LDI specification is not yet in hand, so instead of hardcoding a
 * service UUID we do what works without one: scan and log everything, connect
 * on request, enumerate every service / characteristic / descriptor, subscribe
 * to every characteristic that supports notify or indicate, and hex-dump
 * whatever arrives.  That is enough to identify the live-data characteristic
 * and capture real frames, which is exactly what Phase 3's decoder needs to be
 * written against.
 *
 * It also runs the MTU/DLE verification before subscribing, because a
 * truncated dump would be worse than no dump -- it would look like a protobuf
 * that needs reverse-engineering rather than a transport bug.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Start passive discovery, logging every advertiser seen. */
esp_err_t ldi_client_scan(int32_t duration_ms);
void      ldi_client_scan_stop(void);

/*
 * Connect to a specific address, e.g. from the scan log.
 * addr_str is "aa:bb:cc:dd:ee:ff"; addr_type is 0 public, 1 random.
 */
esp_err_t ldi_client_connect(const char *addr_str, uint8_t addr_type);

/* Connect to the first advertiser whose name contains this substring. */
esp_err_t ldi_client_connect_by_name(const char *substr);

void ldi_client_disconnect(void);
bool ldi_client_is_connected(void);

/* Toggle the raw notification hex dump. */
void ldi_client_set_dump(bool on);
bool ldi_client_get_dump(void);

/* Print discovered services / characteristics from the last connection. */
void ldi_client_print_gatt(void);

/* Print connection + verification state. */
void ldi_client_print_status(void);

/*
 * Called once from the NimBLE host sync callback, with the identity address
 * type the peripheral already resolved.  Does not start scanning: the bike
 * allows exactly one accessory connection, so taking it unbidden would be
 * rude and could mask a real problem.  Drive it from the console.
 */
void ldi_client_on_sync(uint8_t own_addr_type);

/* ---- autonomous mode -------------------------------------------------
 *
 * The point of this mode is that the bike is not next to the computer. With
 * it on, the board needs no console at all: it scans, picks candidates,
 * connects, verifies the link, subscribes to everything notifiable, and
 * writes it all to the capture partition. The LED reports progress, and the
 * whole session is read back later over serial.
 */

/* Enable/disable the autonomous hunt. Persisted in NVS. */
void ldi_client_set_auto(bool on);
bool ldi_client_get_auto(void);

/*
 * Pin a specific peer, so later sessions connect straight to it instead of
 * hunting. Persisted in NVS. Pass NULL to clear.
 */
esp_err_t ldi_client_set_target(const char *addr_str, uint8_t addr_type);
bool      ldi_client_get_target(char *out, size_t n);

#ifdef __cplusplus
}
#endif
