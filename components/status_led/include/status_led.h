/*
 * status_led -- the only on-device indication of what is going on.
 *
 * The Suunto does not display the connected sensor's name, so there is no
 * on-watch confirmation of what it attached to.  This LED plus the serial
 * log are the entire user interface.
 *
 * States are encoded as PULSE COUNTS on a 2 s cycle rather than as blink
 * rates, so they can be counted unambiguously even when caught mid-cycle.
 * Solid-on is reserved for the fully-working state because that is the one
 * a rider glances at mid-ride: steady light = good, needs no counting.
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    STATUS_BOOTING = 0,   /* solid on, until the BLE host syncs           */
    STATUS_ADVERTISING,   /* 1 pulse  -- waiting for the watch            */
    STATUS_WATCH_ONLY,    /* 2 pulses -- CPS link up, no data source      */
    STATUS_BIKE_ONLY,     /* 3 pulses -- data flowing, watch not attached */
    STATUS_BOTH_OK,       /* solid with a heartbeat blink -- the good state */
    STATUS_SIMULATING,    /* solid with a longer dropout -- data is FAKE  */
    STATUS_DEGRADED,      /* 10 Hz flutter -- verification failed         */
} status_state_t;

void status_led_init(void);
void status_led_set(status_state_t state);
status_state_t status_led_get(void);
const char *status_led_state_name(status_state_t s);

#ifdef __cplusplus
}
#endif
