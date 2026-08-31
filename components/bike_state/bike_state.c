#include "bike_state.h"

#include <assert.h>
#include <string.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * WHY A SPINLOCK
 * --------------
 * The struct is ~150 bytes and the copy takes a few hundred nanoseconds.
 *
 *  - Not a FreeRTOS mutex: take/give costs more than the memcpy it guards,
 *    and it is a *blocking* call.  bike_state_snapshot() is called from
 *    inside NimBLE host callbacks; if the low-priority LED task held a
 *    mutex and got preempted, the BLE host would block on a cosmetic LED.
 *    Priority inheritance would prevent deadlock but not the stall.
 *
 *  - Not C11 atomics: the ESP32-C3 is -march=rv32imc_zicsr_zifencei, i.e.
 *    NO 'A' extension.  ESP-IDF emulates C11 atomics in
 *    components/newlib/stdatomic.c by masking interrupts -- so atomics cost
 *    exactly what this spinlock costs, while being unable to make 13 fields
 *    mutually consistent as a group.  Strictly worse.
 *
 *  - Not a queue: this is latest-value-wins state, not a stream.  A queue
 *    would need drain-to-empty logic on every read and could back up, which
 *    would let the watch be notified from a stale backlog tail.  (A queue IS
 *    the right tool for connection *events* -- see bridge_status.c.)
 *
 * Worst case here is ~1 us of interrupts masked.
 */
static portMUX_TYPE  s_lock = portMUX_INITIALIZER_UNLOCKED;
static bike_state_t  s_state;

/* Set once at init so debug builds can catch a writer on the wrong task. */
static TaskHandle_t  s_writer_task;

static const char *const s_field_names[BIKE_FIELD_COUNT] = {
    "speed",        "battery_soc",       "rider_power",   "cadence",
    "odometer",     "time",              "light_status",  "ambient_brightness",
    "light_reserve","system_lock",       "standstill",    "charger_connected",
    "diagnosis_connected",
};

void bike_state_init(void)
{
    portENTER_CRITICAL(&s_lock);
    memset(&s_state, 0, sizeof(s_state));
    portEXIT_CRITICAL(&s_lock);
    s_writer_task = NULL;
}

void bike_state_commit(const bike_state_t *src)
{
    /* Latch the first writer, then assert every later write comes from the
     * same task.  Converts an accidental future cross-task writer into an
     * immediate loud failure instead of a rare, baffling corruption. */
    if (s_writer_task == NULL) {
        s_writer_task = xTaskGetCurrentTaskHandle();
    } else {
        assert(s_writer_task == xTaskGetCurrentTaskHandle() &&
               "bike_state_commit() must run on the NimBLE host task");
    }

    portENTER_CRITICAL(&s_lock);
    uint32_t seq = s_state.seq;
    memcpy(&s_state, src, sizeof(s_state));
    s_state.seq = seq + 1;
    portEXIT_CRITICAL(&s_lock);
}

void bike_state_snapshot(bike_state_t *dst)
{
    portENTER_CRITICAL(&s_lock);
    memcpy(dst, &s_state, sizeof(*dst));
    portEXIT_CRITICAL(&s_lock);
}

void bike_state_mark(bike_state_t *s, bike_field_t f, int64_t now_us)
{
    if (f < 0 || f >= BIKE_FIELD_COUNT) {
        return;
    }
    s->valid_mask |= (1u << (unsigned)f);
    s->field_us[f] = now_us;
}

bool bike_state_valid(const bike_state_t *s, bike_field_t f)
{
    if (f < 0 || f >= BIKE_FIELD_COUNT) {
        return false;
    }
    return (s->valid_mask & (1u << (unsigned)f)) != 0;
}

bool bike_state_fresh(const bike_state_t *s, bike_field_t f, uint32_t max_age_ms)
{
    if (!bike_state_valid(s, f)) {
        return false;
    }
    int64_t age_us = esp_timer_get_time() - s->field_us[f];
    if (age_us < 0) {
        age_us = 0; /* clock went backwards; treat as fresh rather than stale */
    }
    return age_us <= (int64_t)max_age_ms * 1000;
}

const char *bike_state_field_name(bike_field_t f)
{
    if (f < 0 || f >= BIKE_FIELD_COUNT) {
        return "?";
    }
    return s_field_names[f];
}
