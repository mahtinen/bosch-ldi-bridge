/*
 * data_source -- chooses what feeds the watch.
 *
 * The CPS server calls cps_source_get() and never learns where the numbers came
 * from. This module implements that function and dispatches to either the
 * scripted simulator or the real bike.
 *
 * Keeping the choice here rather than in cps_server is what let the watch side
 * be built and proven months before the bike side existed, and it is what makes
 * the simulator a permanent test harness rather than scaffolding.
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DATA_SOURCE_SIM = 0, /* scripted synthetic ride                        */
    DATA_SOURCE_BIKE,    /* decoded Bosch LDI                              */
    DATA_SOURCE_AUTO,    /* bike when its data is fresh, otherwise nothing */
} data_source_mode_t;

void data_source_set_mode(data_source_mode_t mode);
data_source_mode_t data_source_get_mode(void);
const char *data_source_mode_name(data_source_mode_t m);

/* Which source actually supplied the last sample. */
const char *data_source_active_name(void);

#ifdef __cplusplus
}
#endif
