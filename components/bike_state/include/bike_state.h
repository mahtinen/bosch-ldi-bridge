/*
 * bike_state -- the single shared contract between the bike side and the
 * watch side of the bridge.
 *
 * Deliberately has ZERO Bluetooth dependencies so it can be compiled and
 * unit-tested on the host.
 *
 * THREADING INVARIANT
 * -------------------
 *   Writers  MUST run on the NimBLE host task.
 *   Readers  may run anywhere.
 *
 * Every producer in this firmware is scheduled onto the NimBLE host task
 * (the LDI notify callback is a GAP callback; the simulator and the CPS
 * notify tick are ble_npl_callouts on nimble_port_get_dflt_eventq()).  A
 * NimBLE event queue runs one event to completion before the next, so the
 * bike -> watch handoff is same-task, in-order and needs no
 * synchronisation on the hot path.
 *
 * The accessors are still spinlock-guarded because status_led is a genuine
 * second reader on a different task and must never observe a torn mixture
 * of the 13 fields.  See bike_state.c for why a spinlock and not a mutex,
 * atomics or a queue.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The 13 fields the Bosch LDI exposes.  Used as bit positions in
 * valid_mask and as indices into field_us[]. */
typedef enum {
    BIKE_F_SPEED = 0,
    BIKE_F_BATTERY_SOC,
    BIKE_F_RIDER_POWER,
    BIKE_F_CADENCE,
    BIKE_F_ODOMETER,
    BIKE_F_TIME,
    BIKE_F_LIGHT_STATUS,
    BIKE_F_AMBIENT_BRIGHTNESS,
    BIKE_F_LIGHT_RESERVE,
    BIKE_F_SYSTEM_LOCK,
    BIKE_F_STANDSTILL,
    BIKE_F_CHARGER_CONNECTED,
    BIKE_F_DIAGNOSIS_CONNECTED,
    BIKE_FIELD_COUNT /* == 13 */
} bike_field_t;

/*
 * Units here are the bridge's canonical units, NOT whatever the LDI wire
 * format uses.  All conversion happens in exactly one place --
 * components/ldi_proto/ldi_decode.c -- so that when the spec turns out to
 * say power is in centiwatts, one line changes and nothing downstream cares.
 */
typedef struct {
    /* --- bookkeeping ------------------------------------------------ */
    uint32_t seq;                        /* bumped on every commit      */
    uint32_t valid_mask;                 /* 1u << bike_field_t          */
    int64_t  field_us[BIKE_FIELD_COUNT]; /* esp_timer_get_time() per field */

    /* --- the payload ------------------------------------------------ */
    float    speed_mps;
    uint8_t  battery_soc_pct;
    uint16_t rider_power_w;   /* -> CPS Instantaneous Power              */
    uint16_t cadence_rpm;     /* -> CPS crank revolution data            */
    uint32_t odometer_m;
    uint32_t bike_time_s;
    uint8_t  light_status;
    uint16_t ambient_brightness;
    uint8_t  light_reserve_pct;
    bool     system_locked;
    bool     standstill;
    bool     charger_connected;
    bool     diagnosis_connected;
} bike_state_t;

void bike_state_init(void);

/*
 * Replace the whole state.  Caller builds a bike_state_t, sets the fields
 * it knows and calls bike_state_mark() for each; commit stamps seq and
 * copies it in under the lock.
 *
 * MUST be called from the NimBLE host task (asserted in debug builds).
 */
void bike_state_commit(const bike_state_t *src);

/* Coherent copy of all 13 fields.  Safe from any task. */
void bike_state_snapshot(bike_state_t *dst);

/* Stamp a field as seen-now.  Call before commit(). */
void bike_state_mark(bike_state_t *s, bike_field_t f, int64_t now_us);

/* Has this field ever arrived? */
bool bike_state_valid(const bike_state_t *s, bike_field_t f);

/*
 * Has this field arrived AND been updated within max_age_ms?
 *
 * The CPS layer gates on this: a field that has gone stale is reported as
 * zero rather than held at its last value, because holding would make the
 * watch record a flat plateau that never happened -- and the rider cannot
 * notice, since the watch never displays the sensor name.
 */
bool bike_state_fresh(const bike_state_t *s, bike_field_t f, uint32_t max_age_ms);

/* Human-readable field name, for logs. */
const char *bike_state_field_name(bike_field_t f);

#ifdef __cplusplus
}
#endif
