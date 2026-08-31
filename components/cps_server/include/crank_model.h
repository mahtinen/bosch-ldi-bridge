/*
 * crank_model -- converts a cadence in rpm into the two counters the BLE
 * Cycling Power Measurement characteristic actually carries:
 *
 *   Cumulative Crank Revolutions  (uint16, free-running)
 *   Last Crank Event Time         (uint16, units of 1/1024 s, free-running)
 *
 * This lives on the CONSUMER side of the data-source seam on purpose.  The
 * Bosch LDI reports cadence in rpm, not crank events, so this exact code
 * serves both the simulator and the real bike -- wiring up the bike in
 * Phase 4 replaces the source, not this file.
 *
 * Both counters wrap modulo 65536 and are NEVER reset -- not on wrap, not on
 * disconnect, not on reconnect, not when pedalling stops.  Resetting the
 * event time is the classic cause of "cadence spikes to 1200 rpm" in the
 * recorded workout.
 *
 * The event time wraps every exactly 65536/1024 = 64.000 s, so a receiver
 * must use modular UNSIGNED subtraction:
 *
 *   d_rev = (uint16_t)(rev_n - rev_prev)
 *   d_t   = (uint16_t)(evt_n - evt_prev)
 *   rpm   = d_rev * 61440 / d_t          (61440 == 1024 * 60)
 *
 * Signed subtraction breaks across the wrap.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int64_t  t0_us;          /* time origin for the 1/1024 s stamp        */
    int64_t  prev_us;        /* last tick                                 */
    double   phase_rev;      /* accumulated fractional revolution, [0,1)  */
    uint16_t cum_crank_rev;  /* wraps at 65536                            */
    uint16_t last_evt_1024;  /* wraps at 65536 == 64.000 s                */
    bool     started;
} crank_model_t;

void crank_model_init(crank_model_t *m, int64_t now_us);

/*
 * Advance the model.  Call at a steady rate well above the crank rate --
 * 50 Hz (20 ms) is what this firmware uses.
 *
 * cadence_rpm <= 0.5 means "not pedalling": BOTH counters are left exactly
 * as they are and nothing advances.  That is the whole point of this
 * function and the single most important line in it.
 */
void crank_model_tick(crank_model_t *m, float cadence_rpm, int64_t now_us);

/* Coherent read of both counters. */
void crank_model_get(const crank_model_t *m, uint16_t *out_cum_rev,
                     uint16_t *out_last_evt_1024);

#ifdef __cplusplus
}
#endif
