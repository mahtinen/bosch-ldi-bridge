#include "bridge.h"
#include "bike_state.h"
#include "bridge_console.h"
#include "data_source.h"
#include "ble_link_verify.h"
#include "capture.h"
#include "cps_server.h"
#include "cps_source.h"
#include "ldi_client.h"
#include "ldi_uuids.h"
#include "sim_source.h"
#include "status_led.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_bt.h"

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
     * The bike half.  Both peers now arrive through the one advertisement --
     * the watch because it carries the Cycling Power service UUID, the eBike
     * because it carries the Live Data Service as a solicitation -- so the
     * free demultiplexing NimBLE used to give us (each connection routed to
     * whichever callback created it) is gone.  Ownership is arbitrated
     * instead, and the wiring lives here so neither half has to know about
     * the other.
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
    /* Cheap, and the one thing that recovers a peer that wandered off. */
    cps_server_adv_watchdog();
    ldi_client_supervise();

    bool watch = cps_server_is_connected();
    bool bike  = ldi_client_is_connected();

    status_state_t want;
    if (bike && watch) {
        want = ble_link_verify_is_ok() ? STATUS_BOTH_OK : STATUS_DEGRADED;
    } else if (bike) {
        /* Bike present: degraded if the link could not be verified, since a
         * truncated protobuf stream is worse than an obvious failure. */
        want = ble_link_verify_is_ok() ? STATUS_BIKE_ONLY : STATUS_DEGRADED;
    } else if (data_source_get_mode() == DATA_SOURCE_SIM &&
               cps_server_is_notifying()) {
        want = STATUS_SIMULATING; /* streaming synthetic data to the watch */
    } else if (watch) {
        /*
         * Watch attached, nothing feeding it.  This used to read SIMULATING,
         * because the test was "is the watch subscribed" rather than "is the
         * simulator the source" -- so the 2026-09-12 ride spent five hours
         * claiming to stream fake data while auto mode was in fact streaming
         * zeros from an absent bike.  Two pulses is the state that matters
         * here, and it already means exactly this.
         */
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
    /*
     * adv= answers "did the peer stop finding us, or did we stop advertising".
     * Those two are indistinguishable in a log read back after the fact, and
     * they need completely different fixes.
     */
    capture_event("alive up=%llds heap=%u watch=%d bike=%d src=%d adv=%d",
                  (long long)up_s, (unsigned)esp_get_free_heap_size(),
                  cps_server_is_connected() ? 1 : 0,
                  ldi_client_is_connected() ? 1 : 0,
                  cps_source_present() ? 1 : 0,
                  cps_server_is_advertising() ? 1 : 0);
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
                  "power=%4dW rev=%5u evt=%5u cap=%lurec%s",
             cps_server_is_connected() ? 1 : 0,
             cps_server_is_notifying() ? 1 : 0,
             ldi_client_is_connected() ? 1 : 0,
             sim_source_phase_name(), (int)power, rev, evt,
             (unsigned long)cap_recs, capture_full() ? " LOG-FULL" : "");

    bool sub_power = false, sub_cadence = false;
    (void)cps_server_subscriptions(&sub_power, &sub_cadence);

    ESP_LOGI(TAG, "      src=%s(%s) mode=%s notifying=%s sub=[pwr=%d cad=%d]",
             data_source_active_name(),
             ldi_client_is_connected() ? "bike-linked" : "no-bike",
             data_source_mode_name(data_source_get_mode()),
             cps_source_present() ? "yes" : "SUSPENDED (no source)",
             sub_power ? 1 : 0, sub_cadence ? 1 : 0);
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

    /*
     * Turn the transmitter up to maximum.
     *
     * This was never set, so the controller ran at its +9 dBm default while
     * the ESP32-C3 supports +20 -- eleven decibels left unused on a board
     * whose PCB antenna is the weakest part of it. Discovery by the eBike has
     * been unreliable throughout (found immediately sometimes, not at all
     * others, across power cycles of both devices), which is what marginal RF
     * looks like rather than a protocol fault.
     *
     * Costs current, which is the helpful direction here: the documented risk
     * on a USB power bank is the bank cutting out BELOW ~50-100 mA.
     */
    esp_err_t pwr = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P20);
    if (pwr == ESP_OK) {
        pwr = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P20);
    }
    if (pwr != ESP_OK) {
        ESP_LOGW(TAG, "could not raise TX power: %s", esp_err_to_name(pwr));
    } else {
        ESP_LOGI(TAG, "BLE TX power set to +20 dBm");
    }

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
     * LTK and IRK.
     *
     * The IRK looks pointless here -- it exists to resolve private addresses
     * and we advertise a stable public one -- and this distributed ENC alone
     * on exactly that reasoning. But the eBike is the central in this profile
     * and decides what a bond looks like, and a working third-party LDI
     * accessory (Xunil99/ha-bosch-ebike) distributes ENC | ID. Matching it
     * costs one stored object per bond and removes a variable from a pairing
     * that currently fails.
     */
    ble_hs_cfg.sm_our_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC |
                                   BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC |
                                   BLE_SM_PAIR_KEY_DIST_ID;

    ESP_ERROR_CHECK(cps_server_init());

    /*
     * Advertise that we want the eBike Live Data Service, and tell the
     * peripheral half who to hand a connection to once it turns out to be the
     * bike.  cps_server stays ignorant of the bike, ldi_client stays ignorant
     * of advertising, and the knowledge that they share one radio lives here.
     */
    static const ble_uuid128_t ldi_uuid = LDI_SVC_LIVEDATA_UUID128;
    cps_server_set_solicit_uuid(&ldi_uuid);

    static const cps_link_arbiter_t arbiter = {
        .offer  = ldi_client_offer_inbound,
        .owns   = ldi_client_owns,
        .handle = ldi_client_gap_event,
    };
    cps_server_set_link_arbiter(&arbiter);

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
