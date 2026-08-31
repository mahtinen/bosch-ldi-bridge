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

#ifdef __cplusplus
}
#endif
