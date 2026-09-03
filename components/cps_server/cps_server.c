#include "cps_server.h"
#include "capture.h"
#include "cps_ctrl_point.h"
#include "cps_gatt.h"
#include "cps_source.h"
#include "crank_model.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "services/gap/ble_svc_gap.h"

static const char *TAG = "cps_srv";

/* Notification rate.  See the note on 1 Hz below. */
#define CPS_NOTIFY_PERIOD_US  (1000 * 1000)
/* Crank model tick: well above any plausible crank rate. */
#define CPS_CRANK_TICK_US     (20 * 1000)

/* Delay after the watch subscribes before we ask for connection params. */
#define CPS_CONN_PARAM_DELAY_US (1000 * 1000)

static uint8_t  s_own_addr_type;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool     s_notify_enabled;
static bool     s_conn_params_requested;

static crank_model_t s_crank;

static esp_timer_handle_t s_crank_timer;   /* 50 Hz  */
static esp_timer_handle_t s_notify_timer;  /* 1 Hz   */
static esp_timer_handle_t s_connparam_timer;

static struct ble_npl_event s_notify_ev;
static struct ble_npl_event s_connparam_ev;

static int cps_server_gap_event(struct ble_gap_event *event, void *arg);

/* ------------------------------------------------------------------ */
/*  Advertising                                                        */
/* ------------------------------------------------------------------ */

void cps_server_start_adv(void)
{
    struct ble_hs_adv_fields fields;
    struct ble_gap_adv_params adv_params;
    int rc;

    memset(&fields, 0, sizeof(fields));

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    /*
     * The service UUID is the load-bearing part of this advertisement: a
     * watch filtering its power-pod scan on 0x1818 would never see us
     * without it.  The blecsc example omits its own service UUID entirely,
     * which is why advertising is modelled on bleprph instead.
     *
     * "Complete" (AD type 0x03) rather than "incomplete" (0x02): we
     * advertise exactly the one primary service a scanner cares about, so
     * the complete list is both honest and correctly filterable.
     */
    fields.uuids16 = (ble_uuid16_t[]){
        BLE_UUID16_INIT(CPS_SVC_UUID16),
    };
    fields.num_uuids16        = 1;
    fields.uuids16_is_complete = 1;

    /* Set at init via ble_svc_gap_device_appearance_set(); read back here. */
    fields.appearance            = ble_svc_gap_device_appearance();
    fields.appearance_is_present = 1;

    fields.tx_pwr_lvl            = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.tx_pwr_lvl_is_present = 1;

    const char *name = ble_svc_gap_device_name();
    fields.name             = (uint8_t *)name;
    fields.name_len         = strlen(name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        /* Most likely cause: the payload exceeded 31 bytes. */
        ESP_LOGE(TAG, "adv_set_fields rc=%d (payload too large?)", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; /* undirected connectable */
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; /* general discoverable   */
    /*
     * Fast (30-60 ms) permanently, restarted immediately on disconnect, with
     * no slow-advertising fallback phase.  The watch opens a short scan
     * window at sport-mode start and expects to find its remembered sensor
     * within a couple of seconds; this guarantees several advertising events
     * land inside any plausible window.  The usual reason to back off is
     * battery, which does not apply -- we are on a USB power bank where the
     * documented risk is the bank cutting out BELOW ~50-100 mA, so fast
     * advertising nudges current the helpful way.
     */
    adv_params.itvl_min = BLE_GAP_ADV_ITVL_MS(30);
    adv_params.itvl_max = BLE_GAP_ADV_ITVL_MS(60);

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                           cps_server_gap_event, NULL);
    if (rc != 0) {
        /* Silently failing here is how a watch never comes back: the
         * disconnect handler restarts advertising, and if that restart fails
         * nothing is listening for the watch any more. */
        ESP_LOGE(TAG, "adv_start rc=%d -- NOT ADVERTISING", rc);
        capture_event("adv_start FAILED rc=%d", rc);
        return;
    }

    ESP_LOGI(TAG, "advertising as \"%s\" svc=0x%04X appearance=0x%04X",
             name, CPS_SVC_UUID16, fields.appearance);
    capture_event("advertising started");
}

/* ------------------------------------------------------------------ */
/*  Connection parameters                                              */
/* ------------------------------------------------------------------ */

static void cps_request_conn_params(struct ble_npl_event *ev)
{
    (void)ev;

    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || s_conn_params_requested) {
        return;
    }
    s_conn_params_requested = true;

    /*
     * We are the peripheral; the watch is the central and it decides.  This
     * is a request, issued ONCE.  If it is refused we log it and live with
     * whatever the watch imposed -- some centrals disconnect on repeated
     * parameter-update spam, so this must never be retried in a loop.
     *
     * latency = 0 is non-negotiable: slave latency lets the peripheral skip
     * connection events, which directly delays our notifications by up to
     * latency * interval.  There is no power reason to want it here.
     */
    struct ble_gap_upd_params want = {
        .itvl_min            = BLE_GAP_CONN_ITVL_MS(30),
        .itvl_max            = BLE_GAP_CONN_ITVL_MS(50),
        .latency             = 0,
        .supervision_timeout = BLE_GAP_SUPERVISION_TIMEOUT_MS(4000),
        .min_ce_len          = 0,
        .max_ce_len          = 0,
    };

    int rc = ble_gap_update_params(s_conn_handle, &want);
    if (rc != 0) {
        ESP_LOGW(TAG, "update_params rc=%d (accepting what the watch chose)", rc);
    }
    capture_event("watch conn param request rc=%d (want 30-50ms, latency 0)", rc);
}

static void cps_connparam_timer_cb(void *arg)
{
    (void)arg;
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &s_connparam_ev);
}

/* ------------------------------------------------------------------ */
/*  The measurement tick                                               */
/* ------------------------------------------------------------------ */

/* esp_timer task: advance the crank model against real time. */
static void cps_crank_timer_cb(void *arg)
{
    (void)arg;

    cps_source_sample_t s;
    cps_source_get(&s);

    /*
     * A stale or absent sample is treated as "not pedalling, no power"
     * rather than held at its last value.  Holding would make the watch
     * record a flat plateau that never happened, and the rider cannot
     * notice because the watch never displays the sensor name.
     */
    float cadence = s.valid ? s.cadence_rpm : 0.0f;
    crank_model_tick(&s_crank, cadence, esp_timer_get_time());
}

/* Host task: pack and notify. */
static void cps_notify_ev_cb(struct ble_npl_event *ev)
{
    (void)ev;

    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_notify_enabled) {
        return;
    }

    cps_source_sample_t s;
    cps_source_get(&s);

    int32_t power = s.valid ? s.power_w : 0;
    if (power < 0) {
        power = 0; /* LDI rider power is never negative */
    }
    if (power > 2000) {
        power = 2000;
    }

    uint16_t rev, evt;
    crank_model_get(&s_crank, &rev, &evt);

    cps_gatt_notify(s_conn_handle, (int16_t)power, rev, evt);
}

static void cps_notify_timer_cb(void *arg)
{
    (void)arg;
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &s_notify_ev);
}

/* ------------------------------------------------------------------ */
/*  GAP events                                                         */
/* ------------------------------------------------------------------ */

static void cps_log_conn_desc(const char *what, uint16_t conn_handle)
{
    struct ble_gap_conn_desc desc;
    if (ble_gap_conn_find(conn_handle, &desc) != 0) {
        return;
    }
    /*
     * This log line is the only confirmation available anywhere: the watch
     * does not display the connected sensor name, so serial output plus the
     * status LED are the entire user interface.
     */
    ESP_LOGI(TAG,
             "%s handle=%u itvl=%u(%.2fms) latency=%u timeout=%u(%ums) "
             "encrypted=%d authenticated=%d bonded=%d",
             what, conn_handle, desc.conn_itvl, desc.conn_itvl * 1.25,
             desc.conn_latency, desc.supervision_timeout,
             desc.supervision_timeout * 10,
             desc.sec_state.encrypted, desc.sec_state.authenticated,
             desc.sec_state.bonded);
}

static int cps_server_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle           = event->connect.conn_handle;
            s_notify_enabled        = false;
            s_conn_params_requested = false;
            cps_log_conn_desc("WATCH CONNECTED", s_conn_handle);
            {
                struct ble_gap_conn_desc d;
                if (ble_gap_conn_find(s_conn_handle, &d) == 0) {
                    capture_event("watch connect itvl=%u latency=%u timeout=%u "
                                  "enc=%d bonded=%d",
                                  d.conn_itvl, d.conn_latency,
                                  d.supervision_timeout,
                                  d.sec_state.encrypted, d.sec_state.bonded);
                }
            }
        } else {
            ESP_LOGW(TAG, "connect failed status=%d, re-advertising",
                     event->connect.status);
            cps_server_start_adv();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        /*
         * The reason matters more than the fact. A long ride once lost the
         * watch 33 minutes in and never got it back, and the reason had not
         * been recorded -- leaving no way to tell a supervision timeout (0x08,
         * RF or a stalled host) from the watch deliberately hanging up (0x13),
         * which need completely different fixes.
         */
        ESP_LOGI(TAG, "WATCH DISCONNECTED reason=0x%02x", event->disconnect.reason);
        capture_event("watch disconnect reason=0x%02x notifying_was=%d",
                      event->disconnect.reason, s_notify_enabled ? 1 : 0);
        cps_ctrl_point_on_disconnect(event->disconnect.conn.conn_handle);
        s_conn_handle           = BLE_HS_CONN_HANDLE_NONE;
        s_notify_enabled        = false;
        s_conn_params_requested = false;
        esp_timer_stop(s_connparam_timer);
        cps_server_start_adv();
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE: {
        cps_log_conn_desc("conn params updated", event->conn_update.conn_handle);
        /*
         * Worth logging: the watch is the central and picks the interval. One
         * was observed imposing itvl=8, i.e. 10 ms -- 100 connection events a
         * second, against the 30-50 ms requested here. Sharing a radio with the
         * bike link at that rate is a plausible source of the supervision
         * timeouts (disconnect reason 0x208) seen mid-ride.
         */
        struct ble_gap_conn_desc d;
        if (ble_gap_conn_find(event->conn_update.conn_handle, &d) == 0) {
            capture_event("watch conn_update status=%d itvl=%u(%.1fms) "
                          "latency=%u timeout=%u",
                          event->conn_update.status, d.conn_itvl,
                          d.conn_itvl * 1.25, d.conn_latency,
                          d.supervision_timeout);
        }
        return 0;
    }

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == g_cps_measurement_handle) {
            s_notify_enabled = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "CP Measurement notifications %s (reason=%d)",
                     s_notify_enabled ? "ENABLED" : "disabled",
                     event->subscribe.reason);
            /*
             * reason 3 is BLE_GAP_SUBSCRIBE_REASON_RESTORE: a bonded peer
             * reconnected and NimBLE reinstated its stored CCCD without the
             * peer writing it again. Worth recording, because after a watch
             * pause/resume it distinguishes "the watch came back and
             * resubscribed" from "the watch never came back at all".
             */
            capture_event("watch subscribe notify=%d reason=%d",
                          s_notify_enabled ? 1 : 0, event->subscribe.reason);
            if (s_notify_enabled && !s_conn_params_requested) {
                /* Let the watch finish discovery before asking. */
                (void)esp_timer_start_once(s_connparam_timer,
                                           CPS_CONN_PARAM_DELAY_US);
            }
        } else if (event->subscribe.attr_handle == g_cps_control_point_handle) {
            cps_ctrl_point_set_indicate(event->subscribe.conn_handle,
                                        event->subscribe.cur_indicate);
        }
        return 0;

    case BLE_GAP_EVENT_MTU:
        /*
         * Informational on this link.  The CP Measurement is 8 bytes and the
         * default 23-byte ATT MTU carries 20, so no MTU or Data Length
         * Extension work is needed here.  That problem belongs entirely to
         * the bike link, where the opposite is true.
         */
        ESP_LOGI(TAG, "watch MTU = %d", event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        cps_log_conn_desc("encryption changed", event->enc_change.conn_handle);
        if (event->enc_change.status != 0) {
            /*
             * The peer holds an LTK we do not, or vice versa.  Drop our side
             * of the bond so the next attempt is a clean fresh pairing
             * rather than a permanent silent failure.
             */
            struct ble_gap_conn_desc desc;
            if (ble_gap_conn_find(event->enc_change.conn_handle, &desc) == 0) {
                ESP_LOGW(TAG, "encryption failed status=%d, deleting bond",
                         event->enc_change.status);
                ble_store_util_delete_peer(&desc.peer_id_addr);
            }
        }
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        /*
         * We hold a bond but the peer is pairing again, which means it
         * forgot us -- the user removed the sensor on the watch and is
         * re-adding it.  Throw the stale bond away and let the new pairing
         * proceed.  Without this, "remove sensor, then re-pair" fails and
         * looks like a broken device.
         */
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
            ESP_LOGI(TAG, "repeat pairing, deleting stale bond");
            ble_store_util_delete_peer(&desc.peer_id_addr);
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_PASSKEY_ACTION:
        /*
         * Must never happen: sm_io_cap is BLE_HS_IO_NO_INPUT_OUTPUT, which
         * yields Just Works.  Log loudly if it does, because it means the
         * security configuration is not what this file assumes.
         */
        ESP_LOGE(TAG, "UNEXPECTED passkey action %d -- check sm_io_cap",
                 event->passkey.params.action);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "advertising complete reason=%d, restarting",
                 event->adv_complete.reason);
        cps_server_start_adv();
        return 0;

    default:
        return 0;
    }
}

/* ------------------------------------------------------------------ */
/*  Init                                                               */
/* ------------------------------------------------------------------ */

bool cps_server_is_connected(void)
{
    return s_conn_handle != BLE_HS_CONN_HANDLE_NONE;
}

bool cps_server_is_notifying(void)
{
    return s_conn_handle != BLE_HS_CONN_HANDLE_NONE && s_notify_enabled;
}

void cps_server_get_counters(int16_t *out_power_w, uint16_t *out_cum_rev,
                             uint16_t *out_last_evt_1024)
{
    cps_source_sample_t s;
    cps_source_get(&s);
    if (out_power_w) {
        *out_power_w = s.valid ? s.power_w : 0;
    }
    crank_model_get(&s_crank, out_cum_rev, out_last_evt_1024);
}

void cps_server_adv_watchdog(void)
{
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        return; /* connected: not supposed to be advertising */
    }
    if (ble_gap_adv_active()) {
        return; /* all is well */
    }

    /*
     * Not connected and not advertising: the watch cannot possibly find us.
     * Rate-limited so a persistently failing restart does not fill the log.
     */
    static int64_t last_try_us;
    int64_t now = esp_timer_get_time();
    if (last_try_us != 0 && (now - last_try_us) < 5 * 1000 * 1000) {
        return;
    }
    last_try_us = now;

    ESP_LOGW(TAG, "not connected and not advertising -- restarting");
    capture_event("adv watchdog: advertising was OFF, restarting");
    cps_server_start_adv();
}

void cps_server_disconnect(void)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    int rc = ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    ESP_LOGI(TAG, "terminate rc=%d", rc);
}

esp_err_t cps_server_init(void)
{
    esp_err_t err = cps_gatt_init();
    if (err != ESP_OK) {
        return err;
    }

    crank_model_init(&s_crank, esp_timer_get_time());

    ble_npl_event_init(&s_notify_ev,    cps_notify_ev_cb,       NULL);
    ble_npl_event_init(&s_connparam_ev, cps_request_conn_params, NULL);

    const esp_timer_create_args_t crank_args = {
        .callback = cps_crank_timer_cb, .name = "cps_crank",
    };
    const esp_timer_create_args_t notify_args = {
        .callback = cps_notify_timer_cb, .name = "cps_notify",
    };
    const esp_timer_create_args_t cp_args = {
        .callback = cps_connparam_timer_cb, .name = "cps_cparm",
    };

    ESP_ERROR_CHECK(esp_timer_create(&crank_args,  &s_crank_timer));
    ESP_ERROR_CHECK(esp_timer_create(&notify_args, &s_notify_timer));
    ESP_ERROR_CHECK(esp_timer_create(&cp_args,     &s_connparam_timer));

    ESP_ERROR_CHECK(esp_timer_start_periodic(s_crank_timer,  CPS_CRANK_TICK_US));
    /*
     * 1 Hz, matching essentially every commercial BLE power meter.  The
     * watch records at 1 s so nothing faster reaches the file, and at any
     * cadence >= 60 rpm every packet carries at least one new crank event,
     * so the receiver never sees a zero time delta in normal operation.
     */
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_notify_timer, CPS_NOTIFY_PERIOD_US));

    return ESP_OK;
}

uint8_t cps_server_own_addr_type(void)
{
    return s_own_addr_type;
}

esp_err_t cps_server_resolve_addr(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ensure_addr rc=%d", rc);
        return ESP_FAIL;
    }

    /*
     * privacy = 0 yields the public identity address, which is derived from
     * the factory eFuse MAC and is therefore identical on every boot.
     *
     * This matters for re-pairing.  A resolvable private address requires
     * the peer to hold our IRK, which only exists after a successful bond,
     * so it is unresolvable before pairing and after the watch forgets us.
     * A non-resolvable private address regenerates on every boot, so after
     * a reflash the watch sees a different device and the bond is orphaned
     * -- exactly the "watch forgot the sensor" failure this design must
     * avoid.  Sport watches routinely key their remembered-sensor list on
     * the address.
     */
    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "id_infer_auto rc=%d", rc);
        return ESP_FAIL;
    }

    uint8_t addr[6] = {0};
    ble_hs_id_copy_addr(s_own_addr_type, addr, NULL);
    ESP_LOGI(TAG, "own address type=%d %02X:%02X:%02X:%02X:%02X:%02X",
             s_own_addr_type, addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
    return ESP_OK;
}
