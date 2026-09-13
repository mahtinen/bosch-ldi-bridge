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
/* The link whose connection-parameter request is pending on the timer. */
static uint16_t s_connparam_handle = BLE_HS_CONN_HANDLE_NONE;

/* Whether a data source was present at the last notify tick, and when that
 * last changed -- so a suspension can be reported with its duration. */
static bool     s_source_present;
static int64_t  s_source_change_us;

/*
 * The advertisement carries a Service Solicitation UUID as well as the CPS
 * service UUID, so the eBike and the watch are both answered by one set of
 * advertising data.  cps_server does not know what the solicited service is;
 * the supervisor supplies it (see cps_server_set_solicit_uuid).
 */
static const ble_uuid128_t *s_solicit_uuid;

/*
 * Every live inbound link, and who owns each event.
 *
 * A list rather than a single handle, because the watch is no longer the only
 * thing that can connect to us. A new link is not classified until the
 * arbiter's probe finishes, so at any instant one of these may turn out to
 * belong to the bike -- and with two connections in flight there is no safe
 * moment to guess. watch_handle() derives the answer instead of caching it.
 */
/*
 * Every live inbound link, with the subscriptions it holds.
 *
 * Per-connection, not global, because the two identities are two separate
 * pairings and the watch may connect to both at once -- one taking power, the
 * other taking cadence. A single pair of flags would either notify a
 * characteristic nobody asked for or silence one somebody did, and with two
 * links up it would do both at the same time.
 */
typedef struct {
    uint16_t handle;
    bool     cps_sub;   /* subscribed to CP Measurement  */
    bool     csc_sub;   /* subscribed to CSC Measurement */
    bool     params_requested;
} link_t;

static link_t s_conns[MYNEWT_VAL(BLE_MAX_CONNECTIONS)];
static int    s_conn_count;
static cps_link_arbiter_t s_arbiter;

static link_t *link_find(uint16_t h)
{
    for (int i = 0; i < s_conn_count; i++) {
        if (s_conns[i].handle == h) {
            return &s_conns[i];
        }
    }
    return NULL;
}

static void conn_add(uint16_t h)
{
    if (s_conn_count < (int)(sizeof(s_conns) / sizeof(s_conns[0]))) {
        s_conns[s_conn_count] = (link_t){ .handle = h };
        s_conn_count++;
    }
}

static void conn_remove(uint16_t h)
{
    for (int i = 0; i < s_conn_count; i++) {
        if (s_conns[i].handle == h) {
            s_conns[i] = s_conns[--s_conn_count];
            return;
        }
    }
}

/* Is this link one of ours, rather than the bike's? */
static bool is_watch_link(const link_t *l)
{
    return !s_arbiter.owns || !s_arbiter.owns(l->handle);
}

/* Any watch link at all -- used only for coarse "is a watch there" answers. */
static uint16_t watch_handle(void)
{
    for (int i = 0; i < s_conn_count; i++) {
        if (is_watch_link(&s_conns[i])) {
            return s_conns[i].handle;
        }
    }
    return BLE_HS_CONN_HANDLE_NONE;
}

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
/*  Advertising: two identities                                        */
/* ------------------------------------------------------------------ */

/*
 * The bridge advertises as TWO devices.
 *
 * A watch types a sensor once, at pairing, from what it advertises, and then
 * each sport mode uses only the types it wants. Measured on a Suunto Race S:
 *
 *   0x1818 alone   -> typed "Power"    -> eMTB uses it, Cycling does not
 *   0x1818+0x1816  -> typed "Bike pod" -> Cycling uses it, eMTB does not
 *
 * One entry gets one type, and the two modes want different ones. Presenting
 * two addresses with two names, each advertising exactly one service, lets the
 * watch pair both, and every mode then finds what it is looking for. The
 * numbers behind them are the same crank model and the same power, so the two
 * can never disagree.
 *
 * Extended advertising is only the mechanism for a second advertising set.
 * Both instances use LEGACY PDUs, because a watch scanning legacy 1M PHY --
 * which is all of them -- cannot see extended ones at all.
 */

/*
 * ONE identity, one advertising set, legacy advertising.
 *
 * There was briefly a second identity -- a separate address and name
 * advertising Cycling Speed and Cadence -- built because a Suunto Race S
 * attaches the pod in eMTB but not in normal Cycling, and sensor typing looked
 * like the cause. It was not. The watch connects a sensor only when the active
 * sport mode has a SCREEN FIELD displaying that sensor's data; adding a power
 * field to a Cycling-based custom mode fixes it, and one power pod then covers
 * every mode.
 *
 * Two identities needed extended advertising, which needed a third connection
 * slot and a derived static-random address, and which changes how the
 * controller establishes incoming connections. All of that has been removed:
 * it solved a problem that did not exist, and it was carrying risk on the one
 * link that matters. The Cycling Speed and Cadence Service itself remains in
 * the GATT database -- see cps_gatt.c -- unadvertised and free.
 */
void cps_server_start_adv(void)
{
    struct ble_hs_adv_fields fields;
    struct ble_hs_adv_fields rsp;
    struct ble_gap_adv_params adv_params;
    int rc;

    if (s_conn_count >= MYNEWT_VAL(BLE_MAX_CONNECTIONS)) {
        return;
    }
    if (ble_gap_adv_active()) {
        return;
    }

    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    fields.uuids16 = (ble_uuid16_t[]){ BLE_UUID16_INIT(CPS_SVC_UUID16) };
    fields.num_uuids16 = 1;
    /*
     * INCOMPLETE (AD type 0x02): this GATT database also hosts 0x1816 and
     * 0x180A, so a "complete" list of one would be false under CSS Part A 1.1.
     * The bike makes the same choice in its own advertisement.
     */
    fields.uuids16_is_complete = 0;


    if (s_solicit_uuid) {
        fields.sol_uuids128     = s_solicit_uuid;
        fields.sol_num_uuids128 = 1;
    }

    /*
     * Appearance goes in the ADVERTISEMENT, not the scan response.
     *
     * The v19 release notes (appendix 2, "Supported Accessory Appearance
     * Categories") say the eBike UI matches an accessory on
     * dec_category(appearance). A passive scanner never asks for a scan
     * response, so an appearance kept there is invisible to it -- and the
     * bridge then solicits the Live Data Service while giving the bike no way
     * to categorise what is soliciting it.
     *
     * That matched the symptom exactly: eBike Flow, scanning actively from the
     * phone, listed the bridge by name, while every connection attempt failed
     * without one record reaching our host. A working third-party accessory
     * (Xunil99/ha-bosch-ebike) puts flags, appearance, solicitation and name
     * all in the advertising data and sets no scan response at all.
     *
     * Budget, legacy 31 bytes:
     *     flags                    3
     *     appearance               4
     *     16-bit svc UUID 0x1818   4
     *     128-bit solicitation    18
     *                             --
     *                             29 of 31
     *
     * Which leaves no room for the name; it stays in the scan response, where
     * spec 2.1.5.3.2 explicitly permits it and where the watch -- which does
     * scan actively -- has always found it.
     */
    fields.appearance            = ble_svc_gap_device_appearance();
    fields.appearance_is_present = 1;


    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields rc=%d (over 31 bytes?)", rc);
        return;
    }

    memset(&rsp, 0, sizeof(rsp));
    rsp.name                  = (uint8_t *)ble_svc_gap_device_name();
    rsp.name_len              = (uint8_t)strlen((const char *)rsp.name);
    rsp.name_is_complete      = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_rsp_set_fields rc=%d (over 31 bytes?)", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min  = BLE_GAP_ADV_ITVL_MS(30);
    adv_params.itvl_max  = BLE_GAP_ADV_ITVL_MS(60);

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                           cps_server_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_start rc=%d -- NOT ADVERTISING", rc);
        capture_event("adv_start FAILED rc=%d", rc);
        return;
    }

    ESP_LOGI(TAG, "advertising as \"%s\" svc=0x%04X appearance=0x%04X "
                  "solicit=%s conns=%d (legacy, appearance in adv)",
             ble_svc_gap_device_name(), CPS_SVC_UUID16, fields.appearance,
             s_solicit_uuid ? "yes" : "no", s_conn_count);
    capture_event("advertising started legacy conns=%d solicit=%d",
                  s_conn_count, s_solicit_uuid ? 1 : 0);
}


void cps_server_set_solicit_uuid(const ble_uuid128_t *uuid)
{
    s_solicit_uuid = uuid;
}

void cps_server_set_link_arbiter(const cps_link_arbiter_t *arb)
{
    if (arb) {
        s_arbiter = *arb;
    } else {
        memset(&s_arbiter, 0, sizeof(s_arbiter));
    }
}

int cps_server_conn_count(void)
{
    return s_conn_count;
}

/*
 * Is the radio actually advertising right now?
 *
 * The log records when advertising STARTS and never confirms it is still
 * running, so "the bike stopped finding us" and "we stopped advertising" were
 * indistinguishable after the fact -- which is exactly the question that
 * matters when a peer reports a device disappearing mid-session.
 */
bool cps_server_is_advertising(void)
{
    return ble_gap_adv_active() != 0;
}

/* ------------------------------------------------------------------ */
/*  Connection parameters                                              */
/* ------------------------------------------------------------------ */

static void cps_request_conn_params(struct ble_npl_event *ev)
{
    (void)ev;

    /*
     * Per link, because there may be two of them. The handle is latched when
     * the timer is armed rather than looked up now: with two identities,
     * "the watch link" is ambiguous by the time this fires.
     */
    link_t *l = link_find(s_connparam_handle);
    if (l == NULL || l->params_requested) {
        return;
    }
    l->params_requested = true;

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

    int rc = ble_gap_update_params(l->handle, &want);
    if (rc != 0) {
        ESP_LOGW(TAG, "update_params rc=%d (accepting what the watch chose)", rc);
    }
    capture_event("watch conn param request handle=%u rc=%d "
                  "(want 30-50ms, latency 0)", l->handle, rc);
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

    if (s_conn_count == 0) {
        return;
    }

    /*
     * No data source: say nothing rather than say zero.
     *
     * Every CP Measurement carries Instantaneous Power, and because the flags
     * field is a compile-time constant it carries the crank fields too -- so
     * there is no way to encode "I have no data" inside a notification.
     * Silence is the only honest alternative, and it is cheap: an idle ATT
     * link does not disconnect, so the watch keeps the sensor paired and
     * simply records a gap.
     *
     * The 2026-09-12 ride is what this exists for. The bike was absent for
     * 5 h 20 of a 5 h 53 ride and the bridge notified 0 W / 0 rpm throughout;
     * 16,499 of 17,075 recorded samples are those fabricated zeros, and
     * nothing in the file tells them apart from a rider freewheeling.
     */
    bool present = cps_source_present();
    if (present != s_source_present) {
        int64_t now    = esp_timer_get_time();
        int64_t held_s = (now - s_source_change_us) / 1000000;
        s_source_present   = present;
        s_source_change_us = now;
        ESP_LOGW(TAG, "data source %s -- notifications %s (previous state "
                      "held %llds)",
                 present ? "PRESENT" : "GONE",
                 present ? "resumed" : "suspended", (long long)held_s);
        capture_event("cps notify %s after %llds",
                      present ? "RESUMED: source returned"
                              : "SUSPENDED: no data source",
                      (long long)held_s);
    }
    if (!present) {
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

    /*
     * Every subscribed link, on whichever characteristic it asked for.
     *
     * The same crank values and the same power go to all of them, which is
     * what makes two identities honest rather than a trick: the power pod and
     * the cadence pod are one sensor, and a watch reading cadence from CSC on
     * one link and from CPS on the other sees identical numbers.
     */
    for (int i = 0; i < s_conn_count; i++) {
        link_t *l = &s_conns[i];
        if (!is_watch_link(l)) {
            continue;
        }
        if (l->cps_sub) {
            cps_gatt_notify(l->handle, (int16_t)power, rev, evt);
        }
        if (l->csc_sub) {
            csc_gatt_notify(l->handle, rev, evt);
        }
    }
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

/*
 * Which connection is this event about?  Returns false for events that are
 * not tied to one (a failed connect, an advertising completion).
 */
static bool event_conn_handle(const struct ble_gap_event *ev, uint16_t *out)
{
    switch (ev->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (ev->connect.status != 0) {
            return false;
        }
        *out = ev->connect.conn_handle;
        return true;
    case BLE_GAP_EVENT_DISCONNECT:   *out = ev->disconnect.conn.conn_handle; return true;
    case BLE_GAP_EVENT_CONN_UPDATE:  *out = ev->conn_update.conn_handle;     return true;
    case BLE_GAP_EVENT_ENC_CHANGE:   *out = ev->enc_change.conn_handle;      return true;
    case BLE_GAP_EVENT_SUBSCRIBE:    *out = ev->subscribe.conn_handle;       return true;
    case BLE_GAP_EVENT_MTU:          *out = ev->mtu.conn_handle;             return true;
    case BLE_GAP_EVENT_NOTIFY_RX:    *out = ev->notify_rx.conn_handle;       return true;
    case BLE_GAP_EVENT_NOTIFY_TX:    *out = ev->notify_tx.conn_handle;       return true;
    case BLE_GAP_EVENT_DATA_LEN_CHG: *out = ev->data_len_chg.conn_handle;    return true;
    case BLE_GAP_EVENT_REPEAT_PAIRING:
        *out = ev->repeat_pairing.conn_handle;
        return true;
#ifdef BLE_GAP_EVENT_LINK_ESTAB
    case BLE_GAP_EVENT_LINK_ESTAB:
        if (ev->link_estab.status != 0) {
            return false;
        }
        *out = ev->link_estab.conn_handle;
        return true;
#endif
    default:
        return false;
    }
}

static int cps_server_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    /*
     * One advertisement, two peers, one callback.
     *
     * NimBLE routes a connection's events to whichever callback created the
     * link, and until the role flip that demultiplexed the two halves for
     * free: the watch arrived through advertising, the bike through our own
     * connect.  Now both arrive here, so ownership has to be asked about
     * explicitly.  The supervisor installs the arbiter; this module still
     * knows nothing about the bike.
     */
    uint16_t ch = BLE_HS_CONN_HANDLE_NONE;
    bool has_ch = event_conn_handle(event, &ch);

    /*
     * Ownership is read BEFORE dispatching, because the arbiter's own
     * disconnect handling clears the handle it owns -- ask afterwards and a
     * bike disconnect looks like a watch disconnect.
     */
    bool foreign = has_ch && s_arbiter.owns && s_arbiter.owns(ch);

    /*
     * Connection accounting is done here for BOTH peers, not inside the
     * switch below. A bike disconnect never reaches that switch, so counting
     * there would leak a slot on every bike drop until the count looked full
     * and advertising stopped for good -- with nothing in the log to say why.
     */
    if (event->type == BLE_GAP_EVENT_DISCONNECT && has_ch) {
        conn_remove(ch);
    }

    if (foreign) {
        int rc = s_arbiter.handle(event);
        if (event->type == BLE_GAP_EVENT_DISCONNECT) {
            /* A slot just came free; let the bike find us again. */
            cps_server_start_adv();
        }
        return rc;
    }

    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            /*
             * Asked BEFORE the new link joins the list, so a second peer
             * arriving does not count as the watch and does not reset the
             * first one's state.  Without that ordering, a bike connecting
             * behind an already-subscribed watch would clear the watch's
             * subscription flag and silently stop its notifications.
             */
            /*
             * Provisionally ours, with its own clean subscription state. The
             * arbiter probes the peer for the solicited service and takes the
             * link over if it finds it, at which point is_watch_link() stops
             * returning it and every later event for it is routed above.
             */
            conn_add(event->connect.conn_handle);
            cps_log_conn_desc("PEER CONNECTED", event->connect.conn_handle);
            {
                /*
                 * Recorded unconditionally. This used to sit inside the
                 * conn_find() guard, so a connection whose descriptor could
                 * not be read left no trace in the one log a session away from
                 * the computer can produce.
                 */
                struct ble_gap_conn_desc d;
                if (ble_gap_conn_find(event->connect.conn_handle, &d) == 0) {
                    capture_event("inbound connect handle=%u itvl=%u latency=%u "
                                  "timeout=%u enc=%d bonded=%d",
                                  event->connect.conn_handle,
                                  d.conn_itvl, d.conn_latency,
                                  d.supervision_timeout,
                                  d.sec_state.encrypted, d.sec_state.bonded);
                } else {
                    capture_event("inbound connect handle=%u (no conn desc)",
                                  event->connect.conn_handle);
                }
            }
            if (s_arbiter.offer) {
                s_arbiter.offer(event->connect.conn_handle);
            }
            /* A slot may remain -- keep advertising for the other peer. */
            cps_server_start_adv();
        } else {
            /*
             * A FAILED INBOUND CONNECTION, and this branch is reachable as a
             * peripheral -- ble_gap_rx_conn_comp_failed() delivers CONNECT
             * with a non-zero status to the slave callback whenever the
             * controller reports LE Connection Complete with an error. That is
             * what happens when a CONNECT_IND IS received and the link then
             * fails to establish (HCI 0x3E).
             *
             * It was ESP_LOGW only. On a power bank with no serial attached
             * that made "the bike never sent a connection request" and "the
             * bike connected and the link died" indistinguishable in the flash
             * log -- and an entire afternoon was spent concluding the former
             * from an absence that proved nothing.
             */
            ESP_LOGW(TAG, "connect failed status=%d, re-advertising",
                     event->connect.status);
            capture_event("inbound connect FAILED status=%d",
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
        ESP_LOGI(TAG, "WATCH DISCONNECTED handle=%u reason=0x%02x",
                 event->disconnect.conn.conn_handle, event->disconnect.reason);
        capture_event("watch disconnect handle=%u reason=0x%02x",
                      event->disconnect.conn.conn_handle,
                      event->disconnect.reason);
        cps_ctrl_point_on_disconnect(event->disconnect.conn.conn_handle);
        /* The link (and its subscriptions) was already removed above, before
         * ownership was consulted. Nothing global left to clear. */
        if (s_connparam_handle == event->disconnect.conn.conn_handle) {
            s_connparam_handle = BLE_HS_CONN_HANDLE_NONE;
            esp_timer_stop(s_connparam_timer);
        }
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

    case BLE_GAP_EVENT_SUBSCRIBE: {
        link_t *l = link_find(event->subscribe.conn_handle);
        if (l == NULL) {
            return 0;
        }
        if (event->subscribe.attr_handle == g_cps_measurement_handle) {
            l->cps_sub = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "handle=%u CP Measurement notifications %s (reason=%d)",
                     l->handle, l->cps_sub ? "ENABLED" : "disabled",
                     event->subscribe.reason);
            /*
             * reason 3 is BLE_GAP_SUBSCRIBE_REASON_RESTORE: a bonded peer
             * reconnected and NimBLE reinstated its stored CCCD without the
             * peer writing it again. Worth recording, because after a watch
             * pause/resume it distinguishes "the watch came back and
             * resubscribed" from "the watch never came back at all".
             */
            capture_event("watch subscribe handle=%u power notify=%d reason=%d",
                          l->handle, l->cps_sub ? 1 : 0,
                          event->subscribe.reason);
            if (l->cps_sub && !l->params_requested) {
                /* Let the watch finish discovery before asking. */
                s_connparam_handle = l->handle;
                (void)esp_timer_start_once(s_connparam_timer,
                                           CPS_CONN_PARAM_DELAY_US);
            }
        } else if (event->subscribe.attr_handle == g_csc_measurement_handle) {
            /*
             * Tracked separately from CPS on purpose. A watch is free to take
             * one service and ignore the other -- that is the entire point of
             * offering both -- so one flag for two subscriptions would either
             * notify a characteristic nobody asked for or silence one somebody
             * did.
             */
            l->csc_sub = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "handle=%u CSC Measurement notifications %s (reason=%d)",
                     l->handle, l->csc_sub ? "ENABLED" : "disabled",
                     event->subscribe.reason);
            capture_event("watch subscribe handle=%u cadence notify=%d reason=%d",
                          l->handle, l->csc_sub ? 1 : 0,
                          event->subscribe.reason);
            if (l->csc_sub && !l->params_requested) {
                s_connparam_handle = l->handle;
                (void)esp_timer_start_once(s_connparam_timer,
                                           CPS_CONN_PARAM_DELAY_US);
            }
        } else if (event->subscribe.attr_handle == g_cps_control_point_handle) {
            cps_ctrl_point_set_indicate(event->subscribe.conn_handle,
                                        event->subscribe.cur_indicate);
        }
        return 0;
    }

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
        /*
         * Captured, not just logged, and this is the only trace a FAILED
         * inbound connection leaves.
         *
         * A peripheral gets no event when a central's connection attempt does
         * not complete -- there is no "connect failed" on this side of the
         * link. But the attempt stops the advertising instance, so an
         * ADV_COMPLETE with no CONNECT after it is the signature of exactly
         * that: something tried to connect to us and did not arrive.
         *
         * Without this record, "the eBike Flow app says connection failed" and
         * "the bike never transmitted anything" are indistinguishable in the
         * one log available after an unattended session.
         */
        ESP_LOGI(TAG, "advertising complete reason=%d, restarting",
                 event->adv_complete.reason);
        capture_event("adv complete reason=%d conns=%d",
                      event->adv_complete.reason, s_conn_count);
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
    return watch_handle() != BLE_HS_CONN_HANDLE_NONE;
}

bool cps_server_is_notifying(void)
{
    /*
     * Either subscription counts. A watch that takes only CSC is still being
     * fed -- cadence but not power -- and reporting that as "not notifying"
     * would put the LED and the status line at odds with what is on the wire.
     */
    bool p = false, c = false;
    return cps_server_subscriptions(&p, &c) && (p || c);
}

bool cps_server_subscriptions(bool *power, bool *cadence)
{
    bool up = false, p = false, c = false;

    /* Across BOTH identities: the power pod and the cadence pod are separate
     * pairings, and either, neither or both may be subscribed. */
    for (int i = 0; i < s_conn_count; i++) {
        if (!is_watch_link(&s_conns[i])) {
            continue;
        }
        up = true;
        p = p || s_conns[i].cps_sub;
        c = c || s_conns[i].csc_sub;
    }
    if (power)   *power   = p;
    if (cadence) *cadence = c;
    return up;
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
    /*
     * Connection COUNT, not "is the watch connected".  With the bike arriving
     * through the same advertisement, one peer attached is no longer a reason
     * to stop: the other one still has to be able to find us.
     */
    if (s_conn_count >= MYNEWT_VAL(BLE_MAX_CONNECTIONS)) {
        return; /* nothing left to offer */
    }

    /*
     * No ble_gap_adv_active() guard here, deliberately.
     *
     * cps_server_start_adv() already makes that check and returns if
     * advertising is running, so duplicating it buys nothing and is one more
     * place to get wrong -- as it was when there were two advertising sets and
     * this guard could only see the first of them.
     */
    static int64_t last_try_us;
    int64_t now = esp_timer_get_time();
    if (last_try_us != 0 && (now - last_try_us) < 5 * 1000 * 1000) {
        return;
    }
    last_try_us = now;

    cps_server_start_adv();
}

void cps_server_disconnect(void)
{
    uint16_t h = watch_handle();
    if (h == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    int rc = ble_gap_terminate(h, BLE_ERR_REM_USER_CONN_TERM);
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
#if CONFIG_BRIDGE_RANDOM_STATIC_ADDR
    /*
     * A STATIC RANDOM address, derived from the public one so it is identical
     * on every boot (see the re-pairing note above -- a regenerating address
     * would orphan the watch's bond on every reflash).
     *
     * Why at all: the eBike transmits a connection request that never reaches
     * our host, and afterwards believes itself connected -- it stops listing
     * us until the BIKE is power-cycled, while power-cycling the BRIDGE
     * changes nothing. A CONNECT_IND whose address type does not match the
     * advertiser is discarded in the link layer, below HCI, invisible to us
     * and leaving the initiator thinking it succeeded. That is this symptom
     * exactly.
     *
     * Our public address begins 0x9C = 1001 1100. The top two bits are 10,
     * which is a valid PUBLIC address but NOT a valid random one -- static
     * requires 11, RPA 01, NRPA 00. Anything treating our public address as
     * random produces precisely the failure above. Setting the top two bits
     * makes it a conforming static random address and removes the ambiguity.
     *
     * COSTS THE WATCH ITS PAIRING: sport watches key the remembered-sensor
     * list on the address, so the watch must forget and re-pair the pod.
     */
    ble_addr_t rnd;
    memset(&rnd, 0, sizeof(rnd));
    if (ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, rnd.val, NULL) != 0) {
        ESP_LOGE(TAG, "cannot read the public address to derive from");
        return ESP_FAIL;
    }
    rnd.type    = BLE_ADDR_RANDOM;
    rnd.val[5] |= 0xC0;   /* top two bits 11 -> static random */

    rc = ble_hs_id_set_rnd(rnd.val);
    if (rc != 0) {
        ESP_LOGE(TAG, "id_set_rnd rc=%d", rc);
        return ESP_FAIL;
    }
    s_own_addr_type = BLE_OWN_ADDR_RANDOM;
    ESP_LOGW(TAG, "using a STATIC RANDOM address -- the watch must re-pair");
#else
    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "id_infer_auto rc=%d", rc);
        return ESP_FAIL;
    }
#endif

    uint8_t addr[6] = {0};
    ble_hs_id_copy_addr(s_own_addr_type, addr, NULL);
    ESP_LOGI(TAG, "own address type=%d %02X:%02X:%02X:%02X:%02X:%02X",
             s_own_addr_type, addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
    return ESP_OK;
}
