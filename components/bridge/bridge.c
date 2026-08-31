#include "bridge.h"
#include "bike_state.h"
#include "bridge_console.h"
#include "data_source.h"
#include "ble_link_verify.h"
#include "capture.h"
#include "cps_server.h"
#include "ldi_client.h"
#include "sim_source.h"
#include "status_led.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

static const char *TAG = "bridge";

/* Declared by NimBLE's config store but not exposed in a public header;
 * both the bleprph and blecent examples declare it locally like this. */
void ble_store_config_init(void);

/* ------------------------------------------------------------------ */
/*  Host callbacks                                                     */
/* ------------------------------------------------------------------ */

static void bridge_on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE host reset, reason=%d", reason);
    status_led_set(STATUS_DEGRADED);
}

static void bridge_on_sync(void)
{
    if (cps_server_resolve_addr() != ESP_OK) {
        status_led_set(STATUS_DEGRADED);
        return;
    }

    sim_source_start();
    cps_server_start_adv();

    /*
     * The central half.  It registers its OWN gap callback when it scans or
     * connects, entirely separate from the peripheral's -- NimBLE routes each
     * connection's events to whichever callback created that link, so the two
     * roles never need demultiplexing and there is no "if (role == MASTER)"
     * anywhere in this firmware.
     *
     * Scanning is not started automatically: the bike allows exactly one
     * accessory connection, so grabbing it unbidden would be rude and could
     * mask a real problem. Drive it from the console with `scan`.
     */
    ldi_client_on_sync(cps_server_own_addr_type());

    status_led_set(STATUS_ADVERTISING);
}

static void bridge_host_task(void *param)
{
    (void)param;
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run(); /* returns only on nimble_port_stop() */
    nimble_port_freertos_deinit();
}

/* ------------------------------------------------------------------ */
/*  Status aggregation                                                 */
/* ------------------------------------------------------------------ */

static void bridge_status_timer_cb(void *arg)
{
    (void)arg;

    /*
     * Phase 1: there is no bike link yet, so "watch connected and
     * notifying" is as good as it gets -- and because the data is
     * synthetic, that state must be shown as SIMULATING, never as
     * BOTH_OK.  A simulated ride must never look like a real one.
     */
    /*
     * With the bike side live, the LED is the only feedback available when the
     * board is on a power bank next to the bike and the laptop is elsewhere.
     * Pulse counts, so a state can be read by counting rather than by judging
     * a blink rate.
     */
    bool watch = cps_server_is_connected();
    bool bike  = ldi_client_is_connected();

    status_state_t want;
    if (bike && watch) {
        want = ble_link_verify_is_ok() ? STATUS_BOTH_OK : STATUS_DEGRADED;
    } else if (bike) {
        /* Bike present: degraded if the link could not be verified, since a
         * truncated protobuf stream is worse than an obvious failure. */
        want = ble_link_verify_is_ok() ? STATUS_BIKE_ONLY : STATUS_DEGRADED;
    } else if (cps_server_is_notifying()) {
        want = STATUS_SIMULATING; /* streaming synthetic data to the watch */
    } else if (watch) {
        want = STATUS_WATCH_ONLY;
    } else {
        want = STATUS_ADVERTISING;
    }
    status_led_set(want);
}

/*
 * A once-a-minute record whose only job is to mark that the board was still
 * alive at that moment.
 *
 * Without it, how long a session lasted can only be inferred from whatever else
 * happened to be logged, which is sparse and irregular when nothing is
 * connected. With it, the last record before the next boot marker gives uptime
 * to within a minute -- which is exactly what is needed to answer "did the
 * power source hold overnight, or cut out after twenty minutes?".
 *
 * Free heap goes in too: over a multi-hour run a slow leak would show as a
 * downward trend, and there is no cheaper place to watch for one.
 */
static void bridge_uptime_cb(void *arg)
{
    (void)arg;

    int64_t up_s = esp_timer_get_time() / 1000000;
    capture_event("alive up=%llds heap=%u watch=%d bike=%d",
                  (long long)up_s, (unsigned)esp_get_free_heap_size(),
                  cps_server_is_connected() ? 1 : 0,
                  ldi_client_is_connected() ? 1 : 0);
}

static void bridge_heartbeat_cb(void *arg)
{
    (void)arg;

    /*
     * One structured line every 10 s.  With no display on the watch and no
     * sensor name shown, a captured serial log is what answers "did it
     * actually work?" without guessing.
     */
    int16_t  power = 0;
    uint16_t rev = 0, evt = 0;
    cps_server_get_counters(&power, &rev, &evt);

    size_t cap_used = 0, cap_total = 0;
    uint32_t cap_recs = 0;
    capture_stats(&cap_used, &cap_total, &cap_recs);

    ESP_LOGI(TAG, "STATE watch=%d notify=%d bike=%d phase=%-11s "
                  "power=%4dW rev=%5u evt=%5u cap=%lurec",
             cps_server_is_connected() ? 1 : 0,
             cps_server_is_notifying() ? 1 : 0,
             ldi_client_is_connected() ? 1 : 0,
             sim_source_phase_name(), (int)power, rev, evt,
             (unsigned long)cap_recs);

    ESP_LOGI(TAG, "      src=%s(%s) mode=%s",
             data_source_active_name(),
             ldi_client_is_connected() ? "bike-linked" : "no-bike",
             data_source_mode_name(data_source_get_mode()));
}

/* ------------------------------------------------------------------ */
/*  Public                                                             */
/* ------------------------------------------------------------------ */

void bridge_clear_bonds(void)
{
    int rc = ble_store_clear();
    ESP_LOGI(TAG, "bonds cleared, rc=%d", rc);
}

void bridge_disconnect_watch(void)
{
    cps_server_disconnect();
}

esp_err_t bridge_start(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    bike_state_init();

    /* Before BLE: the log wants to record the whole session including setup. */
    if (capture_init() != ESP_OK) {
        ESP_LOGW(TAG, "capture log unavailable -- running without it");
    }

    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.reset_cb        = bridge_on_reset;
    ble_hs_cfg.sync_cb         = bridge_on_sync;
    /* Round-robin eviction when the bond table fills, so a device that has
     * been paired to several things never wedges. */
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /*
     * Security: permissive on both sides.  Never require encryption, always
     * accept pairing if it is offered.
     *
     * This is correct whether or not the Suunto's "Pair sensor" flow
     * actually performs an SMP pairing -- which is not documented anywhere
     * and cannot be known before testing.  Essentially every commercial BLE
     * power meter runs its measurement notifications unencrypted and
     * unbonded precisely to maximise head-unit compatibility, so a
     * require-encryption design would turn an unknown into a hard failure
     * with a confusing symptom.
     */
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT; /* truthful: an LED and
                                                       * a serial port are not
                                                       * a display or keyboard.
                                                       * Yields Just Works. */
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm    = 0; /* mandatory given NO_INPUT_NO_OUTPUT: asking
                                * for MITM with no I/O is incoherent and
                                * would demand a passkey we cannot produce */
    ble_hs_cfg.sm_sc      = 1; /* LESC when the peer supports it; NimBLE
                                * falls back to legacy pairing on its own */
    /*
     * LTK only.  Deliberately NOT distributing the IRK: it exists to resolve
     * private addresses, and we use a stable public address with no privacy,
     * so exchanging IRKs buys nothing and costs a stored object per bond.
     */
    ble_hs_cfg.sm_our_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC;

    ESP_ERROR_CHECK(cps_server_init());

    /* Must come after the GATT registration and before the host task. */
    ble_store_config_init();

    nimble_port_freertos_init(bridge_host_task);

    /* Status aggregation and heartbeat. */
    const esp_timer_create_args_t st_args = {
        .callback = bridge_status_timer_cb, .name = "bridge_st",
    };
    const esp_timer_create_args_t hb_args = {
        .callback = bridge_heartbeat_cb, .name = "bridge_hb",
    };
    const esp_timer_create_args_t up_args = {
        .callback = bridge_uptime_cb, .name = "bridge_up",
    };
    esp_timer_handle_t st_timer, hb_timer, up_timer;
    ESP_ERROR_CHECK(esp_timer_create(&st_args, &st_timer));
    ESP_ERROR_CHECK(esp_timer_create(&hb_args, &hb_timer));
    ESP_ERROR_CHECK(esp_timer_create(&up_args, &up_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(st_timer, 250 * 1000));
    ESP_ERROR_CHECK(esp_timer_start_periodic(hb_timer, 10 * 1000 * 1000));
    ESP_ERROR_CHECK(esp_timer_start_periodic(up_timer, 60 * 1000 * 1000));

    /* One immediately, so a session that dies inside the first minute still
     * leaves a record of having started. */
    bridge_uptime_cb(NULL);

    bridge_console_start();

    return ESP_OK;
}
