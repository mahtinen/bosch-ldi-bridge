#include "data_source.h"
#include "bike_state.h"
#include "cps_source.h"
#include "sim_source.h"

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "datasrc";

#ifndef CONFIG_BRIDGE_VALUE_HOLD_MS
#define CONFIG_BRIDGE_VALUE_HOLD_MS 15000
#endif
#ifndef CONFIG_BRIDGE_BIKE_SILENT_MS
#define CONFIG_BRIDGE_BIKE_SILENT_MS 8000
#endif

/*
 * Three different questions, and conflating them is what cost the 2026-09-12
 * ride most of its data:
 *
 *   "is the bike there?"       -> BIKE_SILENT_MS against a field in EVERY
 *                                 frame (odometer / standstill)
 *   "is the rider stopped?"    -> the standstill flag, which the bike states
 *                                 outright rather than leaving to inference
 *   "is this value usable?"    -> VALUE_HOLD_MS, sized ABOVE the bike's own
 *                                 update spacing for power and cadence
 *
 * The old code asked only the third, at 3 s, against fields the bike sends
 * every 3.0-4.8 s. See the Kconfig help for the measurements.
 */
#define BIKE_LINK_SILENT_MS CONFIG_BRIDGE_BIKE_SILENT_MS
#define VALUE_HOLD_MS       CONFIG_BRIDGE_VALUE_HOLD_MS

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

/*
 * "Is the bike talking to us at all?"
 *
 * Answered from a field the bike puts in EVERY frame, not from the GAP
 * connection state, because a link that is up but silent is the same thing to
 * the rider as no link -- and the LDI spec (2.2.3.2) warns that a vanished
 * data source is signalled by nothing at all.
 *
 * Odometer is the right field for it. Measured across two logged rides it
 * appears in 100% of frames, while rider power appears in 5% and cadence in
 * 4%, so its age tracks "frames are arriving" exactly, with none of the
 * on-change sparsity that makes the power fields useless for this question.
 */
static bool bike_link_alive(void)
{
    bike_state_t s;
    bike_state_snapshot(&s);
    return bike_state_fresh(&s, BIKE_F_ODOMETER, BIKE_LINK_SILENT_MS) ||
           bike_state_fresh(&s, BIKE_F_STANDSTILL, BIKE_LINK_SILENT_MS);
}

bool cps_source_present(void)
{
    switch (s_mode) {
    case DATA_SOURCE_SIM:
        return true;
    case DATA_SOURCE_BIKE:
    case DATA_SOURCE_AUTO:
    default:
        return bike_link_alive();
    }
}

/* Read the decoded bike state, honouring standstill and per-field age. */
static bool bike_sample(cps_source_sample_t *out)
{
    bike_state_t s;
    bike_state_snapshot(&s);

    /*
     * No frames arriving at all: there is no sample here, only the memory of
     * one. Said here as well as in cps_source_present() so that the reported
     * active source stays honest -- otherwise `status` reads "source: bike"
     * while the notify tick is suspended for want of a bike.
     */
    if (!bike_link_alive()) {
        return false;
    }

    /* Link is up but neither field has ever appeared in a frame. */
    if (!bike_state_valid(&s, BIKE_F_RIDER_POWER) &&
        !bike_state_valid(&s, BIKE_F_CADENCE)) {
        return false;
    }

    /*
     * Standstill first, because it is the one statement about this the bike
     * makes in every single frame rather than leaving to inference. When it is
     * asserted, zero is a measurement, not a guess.
     */
    bool stopped = bike_state_fresh(&s, BIKE_F_STANDSTILL, BIKE_LINK_SILENT_MS)
                   && s.standstill;

    /*
     * Otherwise hold the last value for VALUE_HOLD_MS, because an absent field
     * means "unchanged" (spec 2.2.4.3), not "zero".
     *
     * The old behaviour -- zero the moment a field aged out -- was the right
     * answer to spec 2.2.3.2, where a data source vanishes and its last value
     * stands indistinguishable from fresh. It was the wrong answer to ordinary
     * steady riding, which is most of a ride, and it wrote 16,499 zeros into a
     * 5 h 53 activity. Holding is safe precisely because CHANGE is what
     * triggers a send: easing off, stopping pedalling, or any new value
     * arrives on its own. Only a silent disappearance is held, and
     * cps_source_present() cuts that off after BIKE_SILENT_MS anyway.
     */
    bool have_power   = !stopped &&
                        bike_state_fresh(&s, BIKE_F_RIDER_POWER, VALUE_HOLD_MS);
    bool have_cadence = !stopped &&
                        bike_state_fresh(&s, BIKE_F_CADENCE, VALUE_HOLD_MS);

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
            s_active = "bike(no data)";
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
