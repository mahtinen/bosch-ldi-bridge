/*
 * sim_source -- a scripted synthetic ride, so the whole watch side can be
 * proven with no bike and no Bosch spec.
 *
 * It implements cps_source_get(), which is the seam the real LDI client
 * takes over in Phase 4.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cps_source.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The simulator's current sample. Called by data_source, not by cps_server. */
void sim_source_get(cps_source_sample_t *out);

void sim_source_start(void);
void sim_source_stop(void);

/* Manual override, for pairing and for hand-checking packets in nRF Connect
 * where a moving target makes the tick arithmetic impossible to verify. */
void sim_source_set_script(bool enabled);
void sim_source_set_manual(int16_t power_w, float cadence_rpm);

/* Current phase name, for the status log. */
const char *sim_source_phase_name(void);

#ifdef __cplusplus
}
#endif
