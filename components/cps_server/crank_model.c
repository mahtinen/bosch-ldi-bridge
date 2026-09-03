#include "crank_model.h"

/*
 * Real-time driven, deliberately.
 *
 * The ESP-IDF blecsc example drives its equivalent from the TIMER TICK
 * COUNT instead: it increments the revolution counter once per 100 ms tick
 * and adds a fixed (60*1024)/rpm to the event time.  At a claimed 80 rpm
 * that emits one revolution per 100 ms while stamping them 768 ticks
 * (750 ms) apart, so its event-time clock runs 7.5x faster than reality.
 * A receiver computing d_rev/d_t still reads "80 rpm", which is why the bug
 * is invisible in a naive test -- but the time base is fiction, cannot be
 * reconciled with a real 1 s recording, and will not survive being fed real
 * Bosch cadence arriving at unpredictable intervals.
 *
 * It also recomputes (60*1024)/rpm per revolution and truncates every time,
 * so the error compounds: at 79 rpm, 61440/79 = 777.72 -> 777, an implied
 * 79.07 rpm that drifts.  Here the phase is accumulated in double and
 * truncation happens only at the moment of stamping, against a real
 * timestamp, so it never compounds.
 */

/*
 * NOTE: the counters deliberately restart at zero after a reboot, and are NOT
 * persisted. That was tried and removed, because it cannot work:
 *
 *  - Saving to NVS on a timer means restoring a STALE value. Measured at
 *    90 rpm with a 60 s save interval, the count came back 75 revolutions
 *    behind, so the first delta after the reconnect was a huge modular value
 *    -- the very cadence spike persistence was meant to prevent, now happening
 *    on every reboot instead of only on some.
 *  - Saving often enough to be current means an NVS write every few
 *    revolutions, which is real flash wear for no benefit.
 *  - RTC memory does not survive the case that matters, a full power cut.
 *
 * And it is unnecessary: last_crank_event_time is a uint16 of 1/1024 s, so it
 * wraps every 64.000 s. Any reconnect gap longer than a minute makes the time
 * delta ambiguous no matter what the revolution count says, so a conforming
 * client has to establish a fresh baseline from the first notification of a
 * new connection. Given that, the value it restarts from does not matter.
 */
void crank_model_init(crank_model_t *m, int64_t now_us)
{
    m->t0_us         = now_us;
    m->prev_us       = now_us;
    m->phase_rev     = 0.0;
    m->cum_crank_rev = 0;
    m->last_evt_1024 = 0;
    m->started       = true;
}

void crank_model_tick(crank_model_t *m, float cadence_rpm, int64_t now_us)
{
    if (!m->started) {
        crank_model_init(m, now_us);
        return;
    }

    int64_t dt_us = now_us - m->prev_us;
    if (dt_us <= 0) {
        return; /* no time passed, or the clock moved backwards */
    }
    m->prev_us = now_us;

    /*
     * THE CRITICAL LINE.
     *
     * Not pedalling: cum_crank_rev and last_evt_1024 are left EXACTLY as
     * they are.  Nothing advances.  On the wire this makes bytes 4..7 of the
     * measurement repeat verbatim while notifications continue at full rate,
     * so the receiver sees d_rev == 0 and d_t == 0, holds the last cadence
     * briefly and then reports zero -- which is precisely what a real
     * crank-based power meter does when you stop pedalling.
     *
     * Advancing the event time here instead would look correct (cadence
     * would read 0) but would produce a large under-read spike on the first
     * revolution after restarting, and no real sensor behaves that way.
     */
    if (cadence_rpm <= 0.5f) {
        return;
    }

    const double dt_s         = (double)dt_us / 1e6;
    const double rev_period_s = 60.0 / (double)cadence_rpm;

    m->phase_rev += ((double)cadence_rpm / 60.0) * dt_s;

    while (m->phase_rev >= 1.0) {
        m->phase_rev -= 1.0;

        /*
         * Interpolate back to when this revolution actually completed.  The
         * leftover phase tells us how far past the completion we already
         * are, so the stamp is a real instant rather than "whenever the
         * timer happened to fire".
         */
        const int64_t evt_us =
            now_us - (int64_t)(m->phase_rev * rev_period_s * 1e6);

        m->cum_crank_rev++; /* uint16, wraps naturally at 65536 */

        /* 1/1024 s ticks since the time origin, wrapped to 16 bits. */
        const int64_t ticks = ((evt_us - m->t0_us) * 1024) / 1000000;
        m->last_evt_1024 = (uint16_t)(uint64_t)ticks;
    }
}

void crank_model_get(const crank_model_t *m, uint16_t *out_cum_rev,
                     uint16_t *out_last_evt_1024)
{
    if (out_cum_rev) {
        *out_cum_rev = m->cum_crank_rev;
    }
    if (out_last_evt_1024) {
        *out_last_evt_1024 = m->last_evt_1024;
    }
}
