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

struct ble_gap_event;

#ifdef __cplusplus
extern "C" {
#endif

/* Start passive discovery, logging every advertiser seen. */
esp_err_t ldi_client_scan(int32_t duration_ms);
void      ldi_client_scan_stop(void);

/* ---- inbound links: the eBike connects to US ------------------------
 *
 * The bridge never initiates. It advertises the Live Data Service in the
 * Service Solicitation AD type and the eBike, which is the GAP central in
 * this profile (spec 2.1.3.4), connects to it -- so the bridge occupies an
 * accessory slot instead of competing with the phone for the eBike's single
 * peripheral slot. The three calls below are the arbiter cps_server uses to
 * tell the bike's link apart from the watch's.
 */

/* A new inbound link. Probes the peer for the Live Data Service and, if it
 * is there, takes ownership of the connection. */
void ldi_client_offer_inbound(uint16_t conn_handle);

/* Is this connection the bike's? */
bool ldi_client_owns(uint16_t conn_handle);

/* Handle a forwarded GAP event for the bike link. */
int  ldi_client_gap_event(struct ble_gap_event *event);

/* Drop the bike link. The bike reconnects on its own (spec 2.1.6.1.3). */
void ldi_client_disconnect(void);
bool ldi_client_is_connected(void);

/*
 * Periodic supervision. Detects a link that is up but has gone silent and
 * drops it so the bike re-establishes -- a peer that accepts a connection and
 * then sends nothing is a different failure from one that never connects, and
 * only this tells them apart.
 */
void ldi_client_supervise(void);

/* Notifications received on the current bike link. */
uint32_t ldi_client_notify_total(void);

/* Toggle the raw notification hex dump. */
void ldi_client_set_dump(bool on);
bool ldi_client_get_dump(void);

/* Print discovered services / characteristics from the last connection. */
void ldi_client_print_gatt(void);

/* Print connection + verification state. */
void ldi_client_print_status(void);

/*
 * Called once from the NimBLE host sync callback, with the identity address
 * type the peripheral already resolved.
 */
void ldi_client_on_sync(uint8_t own_addr_type);

#ifdef __cplusplus
}
#endif
