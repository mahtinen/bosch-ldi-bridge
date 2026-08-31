#include "cps_ctrl_point.h"
#include "cps_gatt.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "host/ble_hs.h"
#include "nimble/nimble_port.h"

static const char *TAG = "cps_cp";

/* How long a procedure may stay outstanding before we force-complete it. */
#define CPS_CP_PROCEDURE_TIMEOUT_US (30 * 1000 * 1000)

static bool     s_indicate_enabled;
static uint16_t s_indicate_conn = BLE_HS_CONN_HANDLE_NONE;

/* The pending request, captured in the access callback and answered later. */
static bool     s_procedure_active;
static uint16_t s_pending_conn = BLE_HS_CONN_HANDLE_NONE;
static uint8_t  s_pending_opcode;

static struct ble_npl_event s_response_ev;
static struct ble_npl_event s_timeout_ev;

/*
 * An esp_timer one-shot, deliberately NOT a ble_npl_callout.
 *
 * ble_npl_callout_reset()'s second argument is documented as OS ticks, but
 * npl_os_freertos.c does:
 *
 *   #if CONFIG_BT_NIMBLE_USE_ESP_TIMER
 *       esp_timer_start_once(callout->handle, ticks * 1000);
 *   #else
 *       xTimerChangePeriod(callout->handle, ticks, ...);   // real ticks
 *
 * and CONFIG_BT_NIMBLE_USE_ESP_TIMER defaults to y.  So the units are
 * milliseconds in the default configuration and ticks otherwise -- a 10x
 * discrepancy at the default 100 Hz tick rate that flips silently if anyone
 * touches that Kconfig option.  esp_timer with explicit microseconds has no
 * such ambiguity.
 */
static esp_timer_handle_t s_timeout_timer;

static void cps_cp_send_response(uint16_t conn_handle, uint8_t opcode,
                                 uint8_t response_value)
{
    /*
     * {Response Code, Request Op Code, Response Value}.
     *
     * The Response Code is 0x20 for CPS.  CSCS uses 0x10, and the blecsc
     * example both uses that value and appends only the single response
     * byte, omitting the first two entirely.  Neither may be copied.
     */
    uint8_t rsp[3] = { CPS_CP_RESPONSE_CODE, opcode, response_value };

    struct os_mbuf *om = ble_hs_mbuf_from_flat(rsp, sizeof(rsp));
    if (om == NULL) {
        ESP_LOGW(TAG, "response: out of mbufs");
        return;
    }

    int rc = ble_gatts_indicate_custom(conn_handle, g_cps_control_point_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "indicate rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "op 0x%02X -> response %02X %02X %02X",
                 opcode, rsp[0], rsp[1], rsp[2]);
    }
}

static void cps_cp_finish(void)
{
    esp_timer_stop(s_timeout_timer);
    s_procedure_active = false;
    s_pending_conn     = BLE_HS_CONN_HANDLE_NONE;
}

/*
 * Runs on the NimBLE host task, after the access callback has already
 * returned, so the ATT Write Response goes out first as the spec requires.
 * The blecsc example indicates synchronously from inside its access
 * callback, which inverts that ordering.
 */
static void cps_cp_response_ev_cb(struct ble_npl_event *ev)
{
    (void)ev;

    if (!s_procedure_active) {
        return;
    }

    /*
     * We support no op codes at all, so every request is answered "Op Code
     * Not Supported" with its op code echoed back verbatim -- the watch
     * matches its request to our response by that byte.
     *
     * The one that matters in practice is 0x0C Start Offset Compensation,
     * written by the watch's "Calibrate power POD" flow.  Answering it
     * cleanly is the difference between an on-screen failure and a spinner
     * that never resolves.  CP Feature bit 9 (Offset Compensation
     * Supported) is clear, which is the declarative half of the same
     * statement.
     */
    cps_cp_send_response(s_pending_conn, s_pending_opcode,
                         CPS_CP_RSP_OP_NOT_SUPPORTED);
    cps_cp_finish();
}

/* Host task; posted to from the esp_timer callback. */
static void cps_cp_timeout_ev_cb(struct ble_npl_event *ev)
{
    (void)ev;

    if (!s_procedure_active) {
        return;
    }
    ESP_LOGW(TAG, "procedure timeout, op 0x%02X", s_pending_opcode);
    cps_cp_send_response(s_pending_conn, s_pending_opcode,
                         CPS_CP_RSP_OPERATION_FAILED);
    cps_cp_finish();
}

/*
 * Runs on the esp_timer task, so it must not touch the BLE host directly --
 * it only hands the work to the NimBLE event queue.
 *
 * Dormant with a reject-all handler, since nothing here is long-running.
 * It exists so we can never leave the watch waiting on an indication that
 * never arrives, and so that adding real offset compensation later is safe
 * by construction.
 */
static void cps_cp_timeout_timer_cb(void *arg)
{
    (void)arg;
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &s_timeout_ev);
}

void cps_ctrl_point_init(void)
{
    s_indicate_enabled = false;
    s_indicate_conn    = BLE_HS_CONN_HANDLE_NONE;
    s_procedure_active = false;
    s_pending_conn     = BLE_HS_CONN_HANDLE_NONE;
    s_pending_opcode   = 0;

    ble_npl_event_init(&s_response_ev, cps_cp_response_ev_cb, NULL);
    ble_npl_event_init(&s_timeout_ev,  cps_cp_timeout_ev_cb,  NULL);

    const esp_timer_create_args_t args = {
        .callback = cps_cp_timeout_timer_cb,
        .arg      = NULL,
        .name     = "cps_cp_to",
    };
    ESP_ERROR_CHECK(esp_timer_create(&args, &s_timeout_timer));
}

void cps_ctrl_point_set_indicate(uint16_t conn_handle, bool enabled)
{
    s_indicate_enabled = enabled;
    s_indicate_conn    = enabled ? conn_handle : BLE_HS_CONN_HANDLE_NONE;
    ESP_LOGI(TAG, "control point indications %s",
             enabled ? "ENABLED" : "disabled");
}

void cps_ctrl_point_on_disconnect(uint16_t conn_handle)
{
    if (s_indicate_conn == conn_handle) {
        s_indicate_enabled = false;
        s_indicate_conn    = BLE_HS_CONN_HANDLE_NONE;
    }
    if (s_pending_conn == conn_handle) {
        cps_cp_finish();
    }
}

int cps_ctrl_point_handle_write(uint16_t conn_handle,
                               struct ble_gatt_access_ctxt *ctxt)
{
    /*
     * Per spec, a write to the control point while its CCCD is not
     * configured for indications must be rejected with the
     * Bluetooth-defined "CCCD Improperly Configured" error, 0xFD.  NimBLE
     * has no constant for it (its BLE_ATT_ERR_* list stops at 0x13), so it
     * is defined in cps_gatt.h.  The blecsc example uses 0x81, which lies
     * in the application-specific range and means nothing.
     */
    if (!s_indicate_enabled || s_indicate_conn != conn_handle) {
        ESP_LOGW(TAG, "write with indications disabled -> ATT 0x%02X",
                 CPS_ATT_ERR_CCCD_IMPROPERLY_CONFIGURED);
        return CPS_ATT_ERR_CCCD_IMPROPERLY_CONFIGURED;
    }

    if (s_procedure_active) {
        return BLE_ATT_ERR_UNLIKELY; /* Procedure Already In Progress */
    }

    uint8_t opcode = 0;
    if (os_mbuf_copydata(ctxt->om, 0, 1, &opcode) != 0) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    s_pending_conn     = conn_handle;
    s_pending_opcode   = opcode;
    s_procedure_active = true;

    /*
     * Defer the indication so the ATT Write Response is sent first.  Also
     * keeps the access callback short, which matters because it runs on the
     * NimBLE host task.
     */
    (void)esp_timer_start_once(s_timeout_timer, CPS_CP_PROCEDURE_TIMEOUT_US);
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &s_response_ev);

    return 0;
}
