/*
 * cps_source -- the seam between "where power and cadence come from" and
 * "how they are put on the wire".
 *
 * Implemented by sim_source today; the Bosch LDI client feeds the same
 * struct in Phase 4.  The CPS server never learns which one it is talking
 * to, which is what makes the watch-first build order work.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t power_w;      /* instantaneous rider power, watts             */
    float   cadence_rpm;  /* >= 0.0f;  0 means "not pedalling"            */
    bool    valid;        /* false => data is stale/absent, report zero   */
    int64_t timestamp_us; /* esp_timer_get_time() when produced           */
} cps_source_sample_t;

/*
 * Fetch the latest sample.  Called from the CPS notify tick on the NimBLE
 * host task.  Must not block.
 */
void cps_source_get(cps_source_sample_t *out);

/*
 * Is there a data source at all right now?
 *
 * Deliberately NOT the same question as sample.valid, which asks whether the
 * latest reading is fresh.  A rider coasting produces a source that is present
 * and reporting zero; a bike that is switched off, out of range, or holding
 * its one accessory slot open for something else produces no source at all.
 *
 * Those two were indistinguishable on the wire until now, and the cost was
 * concrete: on 2026-09-12 the bridge notified 0 W at 1 Hz for five hours with
 * no bike attached, and the watch wrote every one of those zeros into the
 * activity as though they had been measured.
 */
bool cps_source_present(void);

#ifdef __cplusplus
}
#endif
