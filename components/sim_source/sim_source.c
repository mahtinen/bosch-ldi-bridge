#include "sim_source.h"
#include "cps_source.h"

#include <stddef.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sim";

/*
 * The "staircase and gap" profile.
 *
 * Every phase falsifies a specific claim and is visually unmistakable in
 * the recorded workout.  The 150 W / 60 rpm and 250 W / 90 rpm blocks are
 * deliberately not values the watch could plausibly synthesise, so seeing
 * them in the exported file is proof of provenance rather than coincidence.
 *
 * Total 8:30, then loops.
 */
typedef struct {
    const char *name;
    float    duration_s;
    int16_t  power_start, power_end;
    float    cadence_start, cadence_end;
    bool     square_wave; /* alternate power_start/power_end every 5 s */
} sim_phase_t;

static const sim_phase_t s_phases[] = {
    /* name          dur   pwr0 pwr1   cad0   cad1  square */
    { "idle",         60,    0,    0,   0.f,   0.f, false },
    { "ramp-up",     120,    0,  300,  60.f, 100.f, false },
    { "block-A",      60,  150,  150,  60.f,  60.f, false },
    { "block-B",      60,  250,  250,  90.f,  90.f, false },
    { "hard-stop",    30,    0,    0,   0.f,   0.f, false },
    { "restart",      60,  200,  200,  80.f,  80.f, false },
    { "square-wave",  60,  100,  300,  85.f,  85.f, true  },
    { "ramp-down",    60,  300,    0,  85.f,   0.f, false },
    { "idle-tail",    30,    0,    0,   0.f,   0.f, false },
};
#define SIM_PHASE_COUNT (sizeof(s_phases) / sizeof(s_phases[0]))

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static bool    s_running;
static bool    s_script_enabled = true;
static int64_t s_t0_us;

static int16_t s_manual_power;
static float   s_manual_cadence;

static size_t  s_phase_idx;

static float lerp(float a, float b, float t)
{
    return a + (b - a) * t;
}

static void sim_eval(int64_t now_us, int16_t *out_power, float *out_cadence,
                     size_t *out_phase)
{
    float elapsed = (float)(now_us - s_t0_us) / 1e6f;

    float total = 0.f;
    for (size_t i = 0; i < SIM_PHASE_COUNT; i++) {
        total += s_phases[i].duration_s;
    }
    if (total <= 0.f) {
        *out_power = 0; *out_cadence = 0.f; *out_phase = 0;
        return;
    }

    /* Loop the script. */
    float t = elapsed - total * (float)((int)(elapsed / total));
    if (t < 0.f) {
        t = 0.f;
    }

    for (size_t i = 0; i < SIM_PHASE_COUNT; i++) {
        const sim_phase_t *p = &s_phases[i];
        if (t < p->duration_s) {
            const float frac = (p->duration_s > 0.f) ? (t / p->duration_s) : 0.f;

            if (p->square_wave) {
                /* Alternate every 5 s, so the notification rate and any
                 * watch-side smoothing become directly visible. */
                int slot = (int)(t / 5.0f);
                *out_power = (slot % 2 == 0) ? p->power_start : p->power_end;
            } else {
                *out_power = (int16_t)lerp((float)p->power_start,
                                           (float)p->power_end, frac);
            }
            *out_cadence = lerp(p->cadence_start, p->cadence_end, frac);
            *out_phase   = i;
            return;
        }
        t -= p->duration_s;
    }

    *out_power = 0; *out_cadence = 0.f; *out_phase = SIM_PHASE_COUNT - 1;
}

/*
 * sim_source_get() -- the simulator's sample.
 *
 * This used to BE cps_source_get(), i.e. the simulator was wired straight to
 * the CPS server. Now data_source owns that seam and dispatches between the
 * simulator and the decoded bike, so neither this file nor cps_server needed
 * changing when the bike arrived.
 */
void sim_source_get(cps_source_sample_t *out)
{
    int64_t now = esp_timer_get_time();

    portENTER_CRITICAL(&s_lock);
    bool    running = s_running;
    bool    script  = s_script_enabled;
    int16_t mpower  = s_manual_power;
    float   mcad    = s_manual_cadence;
    portEXIT_CRITICAL(&s_lock);

    if (!running) {
        out->power_w = 0;
        out->cadence_rpm = 0.f;
        out->valid = false;
        out->timestamp_us = now;
        return;
    }

    if (script) {
        size_t phase;
        sim_eval(now, &out->power_w, &out->cadence_rpm, &phase);
        portENTER_CRITICAL(&s_lock);
        s_phase_idx = phase;
        portEXIT_CRITICAL(&s_lock);
    } else {
        out->power_w     = mpower;
        out->cadence_rpm = mcad;
    }

    out->valid        = true;
    out->timestamp_us = now;
}

void sim_source_start(void)
{
    portENTER_CRITICAL(&s_lock);
    s_running = true;
    s_t0_us   = esp_timer_get_time();
    s_phase_idx = 0;
    portEXIT_CRITICAL(&s_lock);

    ESP_LOGW(TAG, "SIMULATED DATA -- power and cadence are synthetic");
    ESP_LOGI(TAG, "script: 8:30 loop, %d phases, blocks at 150W/60rpm and 250W/90rpm",
             (int)SIM_PHASE_COUNT);
}

void sim_source_stop(void)
{
    portENTER_CRITICAL(&s_lock);
    s_running = false;
    portEXIT_CRITICAL(&s_lock);
}

void sim_source_set_script(bool enabled)
{
    portENTER_CRITICAL(&s_lock);
    s_script_enabled = enabled;
    if (enabled) {
        s_t0_us = esp_timer_get_time();
    }
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "script %s", enabled ? "ON" : "OFF (manual)");
}

void sim_source_set_manual(int16_t power_w, float cadence_rpm)
{
    portENTER_CRITICAL(&s_lock);
    s_script_enabled = false;
    s_manual_power   = power_w;
    s_manual_cadence = cadence_rpm;
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "manual: %d W, %.1f rpm", (int)power_w, cadence_rpm);
}

const char *sim_source_phase_name(void)
{
    portENTER_CRITICAL(&s_lock);
    bool   script = s_script_enabled;
    size_t idx    = s_phase_idx;
    portEXIT_CRITICAL(&s_lock);

    if (!script) {
        return "manual";
    }
    return (idx < SIM_PHASE_COUNT) ? s_phases[idx].name : "?";
}
