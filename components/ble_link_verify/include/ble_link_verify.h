/*
 * ble_link_verify -- request and then VERIFY the ATT MTU and the link-layer
 * data length on a connection.
 *
 * This exists as its own module because it is the single hardest functional
 * requirement in the project.  The Bosch eBike is documented not to initiate
 * the Data Length Update procedure, so the accessory must; and if the
 * negotiated ATT MTU stays at the 23-byte default the stack silently
 * truncates notification payloads to 20 bytes and the protobuf stream
 * arrives corrupted.  "Connection established" is NOT success.
 *
 * THE READ-BACK PROBLEM
 * ---------------------
 * There is no API in NimBLE that returns a connection's NEGOTIATED data
 * length.  In particular ble_gap_read_sugg_def_data_len() returns the host's
 * suggested default for NEW connections and will happily report 251 while
 * this link is actually stuck at 27.  Anyone "simplifying" the timeout logic
 * below by calling it would silently break the project's central
 * requirement.
 *
 * The BLE_GAP_EVENT_DATA_LEN_CHG event is the only read-back that exists, so
 * verification is: request, latch the event, and time out if it never comes.
 * A timeout means UNVERIFIED -- which is not the same as "failed", and is
 * reported as such.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Max LL payload with DLE, and its 1M-PHY air time: (251 + 14) * 8 us. */
#define BLE_LINK_TX_OCTETS 251
#define BLE_LINK_TX_TIME   2120

typedef struct {
    uint16_t conn_handle;

    uint16_t att_mtu;        /* from ble_att_mtu() after the exchange       */
    uint16_t max_tx_octets;  /* all four from BLE_GAP_EVENT_DATA_LEN_CHG   */
    uint16_t max_rx_octets;  /* RX is the one that matters: the bike's     */
    uint16_t max_tx_time;    /* notifications land on OUR receive path      */
    uint16_t max_rx_time;

    bool mtu_ok;
    bool dle_ok;
    bool dle_event_seen;     /* false after timeout => UNVERIFIED           */
    bool encrypted;

    int64_t t_conn_us, t_estab_us, t_enc_us, t_mtu_us, t_dle_us;
} ble_link_quality_t;

/*
 * Begin verification on a freshly established link.  Call from the central's
 * GAP callback on BLE_GAP_EVENT_LINK_ESTAB where available -- issuing a
 * data-length request on a link that then fails synchronisation looks
 * exactly like the peer refusing DLE, which is the one failure mode this
 * module exists to distinguish.
 */
void ble_link_verify_begin(uint16_t conn_handle);

/*
 * Feed every GAP event for the link through here.  Returns true if the event
 * was one this module consumes (DATA_LEN_CHG / MTU), purely so the caller can
 * avoid double-logging.
 */
struct ble_gap_event;
bool ble_link_verify_on_gap_event(struct ble_gap_event *event);

/* Snapshot of what has been observed so far. */
void ble_link_verify_get(ble_link_quality_t *out);

/* Verified, in the sense that matters: both negotiated AND wide enough. */
bool ble_link_verify_is_ok(void);

/*
 * Record the link's encryption state.  Must be called from the owning GAP
 * callback -- this module never inspects security itself.
 *
 * Previously the struct carried an `encrypted` field that was logged but never
 * assigned, so every log line read "enc=0" regardless of the truth. That is
 * worse than omitting it: it looked like evidence and was read as evidence.
 */
void ble_link_verify_set_encrypted(bool encrypted);

/* One consolidated log line -- the diagnostic the whole project hinges on. */
void ble_link_verify_log(void);

void ble_link_verify_reset(void);

#ifdef __cplusplus
}
#endif
