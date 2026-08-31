#include "status_led.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "status";

/*
 * ESP32-C3 SuperMini: onboard LED on GPIO8, ACTIVE LOW (cathode to the pin).
 *
 * Polarity CONFIRMED active-low on this board: the blue LED pulses once per
 * 2 s while advertising, rather than being mostly-on with a brief gap.
 * (The solid red LED on the board is its power indicator on the 3V3 rail,
 * not driven by firmware.)
 *
 * Two hardware notes:
 *  - If a different board ever shows an inverted pattern, flip
 *    CONFIG_BRIDGE_LED_ACTIVE_LOW; it is a one-line fix.
 *  - GPIO8 is a strapping pin on the C3 (with GPIO2 and GPIO9).  Driving it
 *    from firmware AFTER boot is fine and is what the ESP-IDF blink example
 *    does; just never add an external strong pull-down, and expect the LED
 *    to flicker during reset.
 *
 * Do NOT copy CONFIG_BLINK_LED_STRIP=y from the blink example's C3 defaults
 * -- that targets the DevKitM's addressable RGB LED, not this plain one.
 */
#ifndef CONFIG_BRIDGE_LED_GPIO
#define CONFIG_BRIDGE_LED_GPIO 8
#endif
#ifndef CONFIG_BRIDGE_LED_ACTIVE_LOW
#define CONFIG_BRIDGE_LED_ACTIVE_LOW 1
#endif

#define LED_GPIO      ((gpio_num_t)CONFIG_BRIDGE_LED_GPIO)
#define LED_TICK_MS   50
#define CYCLE_MS      2000
#define CYCLE_SLOTS   (CYCLE_MS / LED_TICK_MS) /* 40 slots of 50 ms */

static volatile status_state_t s_state = STATUS_BOOTING;

static void led_write(bool on)
{
#if CONFIG_BRIDGE_LED_ACTIVE_LOW
    gpio_set_level(LED_GPIO, on ? 0 : 1);
#else
    gpio_set_level(LED_GPIO, on ? 1 : 0);
#endif
}

/*
 * Returns whether the LED should be lit at slot `slot` of the 2 s cycle.
 * A "pulse" is 100 ms on (2 slots) followed by 200 ms off (4 slots).
 */
static bool pattern_for(status_state_t st, int slot)
{
    switch (st) {
    case STATUS_BOOTING:
        return true; /* solid */

    case STATUS_ADVERTISING:
    case STATUS_WATCH_ONLY:
    case STATUS_BIKE_ONLY: {
        int pulses = (st == STATUS_ADVERTISING) ? 1
                   : (st == STATUS_WATCH_ONLY)  ? 2
                                                : 3;
        /* Each pulse occupies 6 slots (100 ms on, 200 ms off). */
        int window = pulses * 6;
        if (slot >= window) {
            return false; /* quiet remainder of the cycle */
        }
        return (slot % 6) < 2;
    }

    case STATUS_BOTH_OK:
        /* Solid, with a brief blink-off each second as a heartbeat. */
        return !(slot == 0 || slot == 20);

    case STATUS_SIMULATING:
        /* Solid with a longer, clearly visible dropout: "working, but the
         * data is fake".  Distinguishable from BOTH_OK at a glance, which
         * matters given a synthetic ride must never be mistaken for real. */
        return !(slot < 2);

    case STATUS_DEGRADED:
        /* 10 Hz flutter: the visually alarming one. */
        return (slot % 2) == 0;
    }
    return false;
}

static void status_led_task(void *arg)
{
    (void)arg;
    int slot = 0;
    status_state_t last = (status_state_t)-1;

    for (;;) {
        status_state_t st = s_state;
        if (st != last) {
            ESP_LOGI(TAG, "STATE -> %s", status_led_state_name(st));
            last = st;
            slot = 0;
        }
        led_write(pattern_for(st, slot));
        slot = (slot + 1) % CYCLE_SLOTS;
        vTaskDelay(pdMS_TO_TICKS(LED_TICK_MS));
    }
}

void status_led_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << LED_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    led_write(true);

    xTaskCreate(status_led_task, "status_led", 2048, NULL, 2, NULL);
    ESP_LOGI(TAG, "LED on GPIO%d (%s)", (int)LED_GPIO,
             CONFIG_BRIDGE_LED_ACTIVE_LOW ? "active low" : "active high");
}

void status_led_set(status_state_t state)
{
    s_state = state;
}

status_state_t status_led_get(void)
{
    return s_state;
}

const char *status_led_state_name(status_state_t s)
{
    switch (s) {
    case STATUS_BOOTING:     return "booting";
    case STATUS_ADVERTISING: return "advertising";
    case STATUS_WATCH_ONLY:  return "watch connected";
    case STATUS_BIKE_ONLY:   return "bike connected";
    case STATUS_BOTH_OK:     return "both connected";
    case STATUS_SIMULATING:  return "SIMULATED DATA";
    case STATUS_DEGRADED:    return "DEGRADED";
    }
    return "?";
}
