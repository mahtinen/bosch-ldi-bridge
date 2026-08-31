#include "ble_link_verify.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"

static const char *TAG = "linkver";

#ifndef CONFIG_BRIDGE_MIN_ATT_MTU
#define CONFIG_BRIDGE_MIN_ATT_MTU 100
#endif
#ifndef CONFIG_BRIDGE_MIN_LL_RX_OCTETS
#define CONFIG_BRIDGE_MIN_LL_RX_OCTETS 100
#endif
#ifndef CONFIG_BRIDGE_LINK_VERIFY_TIMEOUT_MS
#define CONFIG_BRIDGE_LINK_VERIFY_TIMEOUT_MS 3000
#endif

static ble_link_quality_t  s_q;
static esp_timer_handle_t  s_timeout;
static struct ble_npl_event s_timeout_ev;
static bool                s_inited;

/* ------------------------------------------------------------------ */

static void verify_timeout_ev_cb(struct ble_npl_event *ev)
{
    (void)ev;

    if (s_q.conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }

    /*
     * No BLE_GAP_EVENT_DATA_LEN_CHG arrived.  Because there is no API to read
     * the negotiated value, we cannot distinguish "the peer refused" from
     * "the peer never answered" -- so report UNVERIFIED rather than claiming
     * failure, and say why.
     */
    if (!s_q.dle_event_seen) {
        ESP_LOGE(TAG, "DLE UNVERIFIED: no DATA_LEN_CHG in %d ms. "
                      "Payloads may truncate at 20 bytes.",
                 CONFIG_BRIDGE_LINK_VERIFY_TIMEOUT_MS);
        ESP_LOGE(TAG, "  Empirical cross-check: if notifications consistently "
                      "arrive at exactly 20 bytes, DLE definitively did not take.");
    }
    ble_link_verify_log();
}

static void verify_timeout_timer_cb(void *arg)
{
    (void)arg;
    /* esp_timer task -> hand off to the host task. */
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &s_timeout_ev);
}

static int on_mtu_exchanged(uint16_t conn_handle,
                            const struct ble_gatt_error *error,
                            uint16_t mtu, void *arg)
{
    (void)arg;

    if (error != NULL && error->status != 0) {
        ESP_LOGW(TAG, "MTU exchange failed status=%d", error->status);
        return 0;
    }

    /*
     * Cross-check the callback's value against the authoritative read-back.
     * A disagreement would mean the host and the procedure result diverged,
     * which is worth knowing about loudly.
     */
    uint16_t readback = ble_att_mtu(conn_handle);
    if (readback != mtu) {
        ESP_LOGW(TAG, "MTU disagreement: callback=%u ble_att_mtu()=%u",
                 mtu, readback);
    }

    s_q.att_mtu = readback;
    s_q.mtu_ok  = (readback >= CONFIG_BRIDGE_MIN_ATT_MTU);
    s_q.t_mtu_us = esp_timer_get_time();

    ESP_LOGI(TAG, "ATT MTU = %u (%s, minimum %d)", s_q.att_mtu,
             s_q.mtu_ok ? "ok" : "TOO SMALL", CONFIG_BRIDGE_MIN_ATT_MTU);
    return 0;
}

/* ------------------------------------------------------------------ */

void ble_link_verify_reset(void)
{
    memset(&s_q, 0, sizeof(s_q));
    s_q.conn_handle = BLE_HS_CONN_HANDLE_NONE;
    if (s_timeout) {
        esp_timer_stop(s_timeout);
    }
}

void ble_link_verify_begin(uint16_t conn_handle)
{
    if (!s_inited) {
        ble_npl_event_init(&s_timeout_ev, verify_timeout_ev_cb, NULL);
        const esp_timer_create_args_t args = {
            .callback = verify_timeout_timer_cb, .name = "linkver",
        };
        ESP_ERROR_CHECK(esp_timer_create(&args, &s_timeout));
        s_inited = true;
    }

    ble_link_verify_reset();
    s_q.conn_handle = conn_handle;
    s_q.t_conn_us   = esp_timer_get_time();
    s_q.t_estab_us  = s_q.t_conn_us;

    int rc;

#if CONFIG_BRIDGE_DLE_BEFORE_MTU
    /*
     * Data length FIRST, by default.  Negotiating an ATT MTU of 247 while the
     * link-layer payload is still 27 forces L2CAP fragmentation -- which
     * should work, but is exactly the configuration under which truncation
     * was reported against early bike firmware.  Doing DLE first removes the
     * ambiguity.  Flip CONFIG_BRIDGE_DLE_BEFORE_MTU if this bike turns out to
     * dislike the order; that is why it is a config option and not a
     * hardcoded sequence.
     */
    rc = ble_gap_set_data_len(conn_handle, BLE_LINK_TX_OCTETS, BLE_LINK_TX_TIME);
    ESP_LOGI(TAG, "set_data_len(%d, %d) rc=%d%s",
             BLE_LINK_TX_OCTETS, BLE_LINK_TX_TIME, rc,
             rc == 0 ? " (command accepted -- NOT yet verified)" : "");

    rc = ble_gattc_exchange_mtu(conn_handle, on_mtu_exchanged, NULL);
    ESP_LOGI(TAG, "exchange_mtu rc=%d", rc);
#else
    rc = ble_gattc_exchange_mtu(conn_handle, on_mtu_exchanged, NULL);
    ESP_LOGI(TAG, "exchange_mtu rc=%d", rc);

    rc = ble_gap_set_data_len(conn_handle, BLE_LINK_TX_OCTETS, BLE_LINK_TX_TIME);
    ESP_LOGI(TAG, "set_data_len(%d, %d) rc=%d%s",
             BLE_LINK_TX_OCTETS, BLE_LINK_TX_TIME, rc,
             rc == 0 ? " (command accepted -- NOT yet verified)" : "");
#endif

    /*
     * rc == 0 above means only that the local controller accepted the HCI
     * command.  It says nothing about what the peer agreed to.  That
     * distinction is the "connection established != success" trap this whole
     * module exists to avoid.
     */
    (void)esp_timer_start_once(s_timeout,
                               (uint64_t)CONFIG_BRIDGE_LINK_VERIFY_TIMEOUT_MS * 1000);
}

bool ble_link_verify_on_gap_event(struct ble_gap_event *event)
{
    switch (event->type) {

    case BLE_GAP_EVENT_DATA_LEN_CHG:
        s_q.max_tx_octets = event->data_len_chg.max_tx_octets;
        s_q.max_rx_octets = event->data_len_chg.max_rx_octets;
        s_q.max_tx_time   = event->data_len_chg.max_tx_time;
        s_q.max_rx_time   = event->data_len_chg.max_rx_time;
        s_q.dle_event_seen = true;
        s_q.t_dle_us       = esp_timer_get_time();

        /*
         * RX, not TX.  The bike's notifications arrive on our receive path, so
         * a wide TX with a narrow RX is precisely the failure described in the
         * project brief and would still truncate the protobuf stream.
         */
        s_q.dle_ok = (s_q.max_rx_octets >= CONFIG_BRIDGE_MIN_LL_RX_OCTETS);

        ESP_LOGI(TAG, "DATA_LEN_CHG rx=%u/%uus tx=%u/%uus (%s, minimum rx %d)",
                 s_q.max_rx_octets, s_q.max_rx_time,
                 s_q.max_tx_octets, s_q.max_tx_time,
                 s_q.dle_ok ? "ok" : "TOO SMALL",
                 CONFIG_BRIDGE_MIN_LL_RX_OCTETS);
        return true;

    case BLE_GAP_EVENT_MTU:
        if (s_q.att_mtu == 0) {
            s_q.att_mtu  = event->mtu.value;
            s_q.mtu_ok   = (s_q.att_mtu >= CONFIG_BRIDGE_MIN_ATT_MTU);
            s_q.t_mtu_us = esp_timer_get_time();
        }
        ESP_LOGI(TAG, "GAP MTU event = %u", event->mtu.value);
        return true;

    default:
        return false;
    }
}

void ble_link_verify_set_encrypted(bool encrypted)
{
    s_q.encrypted = encrypted;
    if (encrypted && s_q.t_enc_us == 0) {
        s_q.t_enc_us = esp_timer_get_time();
    }
}

void ble_link_verify_get(ble_link_quality_t *out)
{
    if (out) {
        *out = s_q;
    }
}

bool ble_link_verify_is_ok(void)
{
    return s_q.mtu_ok && s_q.dle_ok && s_q.dle_event_seen;
}

void ble_link_verify_log(void)
{
    bool ok = ble_link_verify_is_ok();

    if (ok) {
        ESP_LOGI(TAG,
                 "BIKE LINK VERIFIED: mtu=%u ll_rx=%u ll_tx=%u enc=%d",
                 s_q.att_mtu, s_q.max_rx_octets, s_q.max_tx_octets,
                 s_q.encrypted ? 1 : 0);
    } else {
        ESP_LOGE(TAG,
                 "BIKE LINK DEGRADED: mtu=%u(%s) ll_rx=%u ll_tx=%u "
                 "dle_seen=%d enc=%d",
                 s_q.att_mtu, s_q.mtu_ok ? "ok" : "small",
                 s_q.max_rx_octets, s_q.max_tx_octets,
                 s_q.dle_event_seen ? 1 : 0, s_q.encrypted ? 1 : 0);
    }
}
