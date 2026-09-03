/*
 * cps_server -- BLE peripheral presenting a standard Cycling Power Service
 * (0x1818) so a Suunto Race S pairs with this device as an ordinary power
 * pod and records rider power and cadence into the workout file.
 *
 * The watch must see nothing unusual.  No vendor characteristics, no
 * required encryption, no extended advertising.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Register the GATT database.  Call before the NimBLE host is started. */
esp_err_t cps_server_init(void);

/* Begin advertising.  Call from the host sync callback. */
void cps_server_start_adv(void);

/* Is the watch connected? */
bool cps_server_is_connected(void);

/* Has the watch subscribed to CP Measurement notifications? */
bool cps_server_is_notifying(void);

/*
 * Verify we are actually advertising when we ought to be, and restart if not.
 * Call periodically.
 *
 * Restarting advertising from the disconnect handler is not sufficient on its
 * own: if that one call fails, nothing is listening for the watch any more and
 * nothing ever tries again. A long ride lost the watch 33 minutes in and never
 * recovered it while the bike streamed on regardless, which is exactly the
 * shape of that failure.
 */
void cps_server_adv_watchdog(void);

/* Terminate the watch link.  Advertising restarts from the disconnect event. */
void cps_server_disconnect(void);

/*
 * Current crank counters and the power we would send right now.  Exposed so
 * the heartbeat log can show what is actually on the wire -- which is the
 * only way to verify the crank arithmetic on target without a BLE central.
 */
void cps_server_get_counters(int16_t *out_power_w, uint16_t *out_cum_rev,
                             uint16_t *out_last_evt_1024);

/*
 * Resolve our own identity address.  Must be called from the NimBLE host
 * sync callback, before cps_server_start_adv().
 */
esp_err_t cps_server_resolve_addr(void);

/* The identity address type resolved above; the central reuses it. */
uint8_t cps_server_own_addr_type(void);

/*
 * NOTE on the dual-role design: the peripheral GAP callback is private to
 * cps_server.c and is registered by cps_server_start_adv() itself.  The
 * central registers its own separate callback when it calls
 * ble_gap_connect().  NimBLE routes each connection's events to whichever
 * callback created that link, so role demultiplexing is free -- no
 * "if (role == MASTER)" anywhere.
 */

#ifdef __cplusplus
}
#endif
