#include "data_source.h"
#include "bike_state.h"
#include "cps_source.h"
#include "sim_source.h"

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "datasrc";

#ifndef CONFIG_BRIDGE_STALE_MS
#define CONFIG_BRIDGE_STALE_MS 3000
#endif

/*
 * AUTO is the default rather than BIKE, so that plugging the board in without
 * the bike present does not silently stream zeros to the watch as though the
 * rider were coasting.
 */
static data_source_mode_t s_mode = DATA_SOURCE_AUTO;
static const char *s_active = "none";

void data_source_set_mode(data_source_mode_t mode)
{
    s_mode = mode;
    ESP_LOGI(TAG, "source mode -> %s", data_source_mode_name(mode));
}

data_source_mode_t data_source_get_mode(void) { return s_mode; }

const char *data_source_mode_name(data_source_mode_t m)
{
    switch (m) {
    case DATA_SOURCE_SIM:  return "sim";
    case DATA_SOURCE_BIKE: return "bike";
    case DATA_SOURCE_AUTO: return "auto";
    }
    return "?";
}

const char *data_source_active_name(void) { return s_active; }

/* Read the decoded bike state, honouring per-field staleness. */
static bool bike_sample(cps_source_sample_t *out)
{
    bike_state_t s;
    bike_state_snapshot(&s);

    bool have_power   = bike_state_fresh(&s, BIKE_F_RIDER_POWER,
                                         CONFIG_BRIDGE_STALE_MS);
    bool have_cadence = bike_state_fresh(&s, BIKE_F_CADENCE,
                                         CONFIG_BRIDGE_STALE_MS);

    if (!have_power && !have_cadence) {
        return false;
    }

    /*
     * Stale means zero, not "hold the last value".
     *
     * This is not merely defensive. The LDI spec (2.2.3.2) warns that when a
     * data source disappears the server sends no notification and no
     * indication -- the last value simply stands, indistinguishable from
     * fresh. Holding it would write a plateau into the workout that never
     * happened, and the rider cannot notice because the watch never displays
     * the sensor name.
     */
    out->power_w      = have_power   ? (int16_t)s.rider_power_w : 0;
    out->cadence_rpm  = have_cadence ? (float)s.cadence_rpm     : 0.0f;
    out->valid        = true;
    out->timestamp_us = esp_timer_get_time();
    return true;
}

/*
 * cps_source_get -- the seam the CPS server calls.
 *
 * Defined here rather than in sim_source so that adding the bike did not
 * require touching either the simulator or the CPS server.
 */
void cps_source_get(cps_source_sample_t *out)
{
    switch (s_mode) {

    case DATA_SOURCE_SIM:
        sim_source_get(out);
        s_active = "sim";
        return;

    case DATA_SOURCE_BIKE:
        if (bike_sample(out)) {
            s_active = "bike";
        } else {
            /* Explicitly no data, rather than falling back to synthetic
             * numbers that would be recorded as if they were real. */
            out->power_w = 0;
            out->cadence_rpm = 0.0f;
            out->valid = false;
            out->timestamp_us = esp_timer_get_time();
            s_active = "bike(stale)";
        }
        return;

    case DATA_SOURCE_AUTO:
    default:
        if (bike_sample(out)) {
            s_active = "bike";
            return;
        }
        /*
         * Deliberately NOT falling through to the simulator. Auto-substituting
         * synthetic power when the bike goes quiet would write fabricated data
         * into a real workout with no indication anywhere -- a data-integrity
         * failure, not a convenience. Use `source sim` explicitly to test.
         */
        out->power_w = 0;
        out->cadence_rpm = 0.0f;
        out->valid = false;
        out->timestamp_us = esp_timer_get_time();
        s_active = "none";
        return;
    }
}
