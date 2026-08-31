/*
 * ldi_auto -- the autonomous hunt.
 *
 * The bike is not next to the computer, so the board has to be able to do a
 * whole diagnostic session on its own: scan, choose a candidate, connect,
 * verify the link, subscribe to everything notifiable, and record it all to
 * flash. The rider watches the LED; the log gets read back afterwards.
 *
 * State machine, driven from a single periodic timer:
 *
 *   IDLE ──► SCANNING ──► CONNECTING ──► LISTENING ──┐
 *              ▲               │             │        │
 *              └───────────────┴─────────────┴────────┘
 *                    (on failure or silence, blacklist and retry)
 *
 * "Silence" is a real outcome worth recording, not just a timeout: a peer that
 * accepts a connection and then sends nothing is a different problem from one
 * that refuses to connect, and the log distinguishes them.
 */
#include "ldi_client.h"
#include "ldi_auto.h"
#include "ble_link_verify.h"
#include "capture.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "ldi_auto";

#define NVS_NS       "bridge"
#define NVS_AUTO     "ldi_auto"
#define NVS_TGT_ADDR "ldi_addr"
#define NVS_TGT_TYPE "ldi_type"

/* How long to sit in each state before giving up and moving on. */
#define SCAN_MS        10000
#define CONNECT_MS      12000
#define SILENCE_MS      45000   /* connected but no notifications */
#define RETRY_MS         5000
#define TICK_MS          1000

#define MAX_BLACKLIST 8

typedef enum { ST_IDLE, ST_SCANNING, ST_CONNECTING, ST_LISTENING } auto_state_t;

static bool          s_auto;
static auto_state_t  s_state;
static int64_t       s_state_since_us;
static esp_timer_handle_t s_tick;

static ble_addr_t s_blacklist[MAX_BLACKLIST];
static int        s_blacklist_n;

static bool       s_have_target;
static ble_addr_t s_target;

static uint32_t   s_attempts;
static uint32_t   s_sessions;   /* connections that produced notifications */

/* ------------------------------------------------------------------ */

static const char *state_name(auto_state_t s)
{
    switch (s) {
    case ST_IDLE:       return "idle";
    case ST_SCANNING:   return "scanning";
    case ST_CONNECTING: return "connecting";
    case ST_LISTENING:  return "listening";
    }
    return "?";
}

static void go(auto_state_t st)
{
    if (s_state != st) {
        ESP_LOGI(TAG, "%s -> %s", state_name(s_state), state_name(st));
        capture_event("auto %s -> %s", state_name(s_state), state_name(st));
    }
    s_state = st;
    s_state_since_us = esp_timer_get_time();
}

static uint32_t elapsed_ms(void)
{
    return (uint32_t)((esp_timer_get_time() - s_state_since_us) / 1000);
}

bool ldi_auto_is_blacklisted(const ble_addr_t *a)
{
    for (int i = 0; i < s_blacklist_n; i++) {
        if (memcmp(s_blacklist[i].val, a->val, 6) == 0) {
            return true;
        }
    }
    return false;
}

static void blacklist(const ble_addr_t *a, const char *why)
{
    if (ldi_auto_is_blacklisted(a) || s_blacklist_n >= MAX_BLACKLIST) {
        return;
    }
    s_blacklist[s_blacklist_n++] = *a;
    ESP_LOGW(TAG, "blacklisting %02x:%02x:%02x:%02x:%02x:%02x (%s)",
             a->val[5], a->val[4], a->val[3], a->val[2], a->val[1], a->val[0], why);
    capture_event("blacklist %02x:%02x:%02x:%02x:%02x:%02x %s",
                  a->val[5], a->val[4], a->val[3], a->val[2], a->val[1],
                  a->val[0], why);
}

/* ------------------------------------------------------------------ */
/*  NVS-backed settings                                                */
/* ------------------------------------------------------------------ */

static void save_u8(const char *key, uint8_t v)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_u8(h, key, v);
    nvs_commit(h);
    nvs_close(h);
}

void ldi_client_set_auto(bool on)
{
    s_auto = on;
    save_u8(NVS_AUTO, on ? 1 : 0);
    ESP_LOGI(TAG, "autonomous mode %s", on ? "ON" : "off");
    capture_event("auto mode %s", on ? "on" : "off");

    if (!on) {
        go(ST_IDLE);
    }
}

bool ldi_client_get_auto(void) { return s_auto; }

esp_err_t ldi_client_set_target(const char *addr_str, uint8_t addr_type)
{
    nvs_handle_t h;

    if (addr_str == NULL) {
        s_have_target = false;
        if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
            nvs_erase_key(h, NVS_TGT_ADDR);
            nvs_commit(h);
            nvs_close(h);
        }
        ESP_LOGI(TAG, "target cleared");
        return ESP_OK;
    }

    unsigned v[6];
    if (sscanf(addr_str, "%x:%x:%x:%x:%x:%x",
               &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) {
        return ESP_ERR_INVALID_ARG;
    }
    for (int i = 0; i < 6; i++) {
        s_target.val[5 - i] = (uint8_t)v[i];
    }
    s_target.type = addr_type;
    s_have_target = true;

    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, NVS_TGT_ADDR, s_target.val, 6);
        nvs_set_u8(h, NVS_TGT_TYPE, addr_type);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGI(TAG, "target pinned: %s type=%u", addr_str, addr_type);
    capture_event("target pinned %s type=%u", addr_str, addr_type);
    return ESP_OK;
}

bool ldi_client_get_target(char *out, size_t n)
{
    if (!s_have_target) {
        return false;
    }
    snprintf(out, n, "%02x:%02x:%02x:%02x:%02x:%02x type=%u",
             s_target.val[5], s_target.val[4], s_target.val[3],
             s_target.val[2], s_target.val[1], s_target.val[0], s_target.type);
    return true;
}

static void load_settings(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    uint8_t v = 0;
    if (nvs_get_u8(h, NVS_AUTO, &v) == ESP_OK) {
        s_auto = (v != 0);
    }
    size_t len = 6;
    if (nvs_get_blob(h, NVS_TGT_ADDR, s_target.val, &len) == ESP_OK && len == 6) {
        s_have_target = true;
        uint8_t t = 0;
        if (nvs_get_u8(h, NVS_TGT_TYPE, &t) == ESP_OK) {
            s_target.type = t;
        }
    }
    nvs_close(h);
}

/* ------------------------------------------------------------------ */
/*  The tick                                                           */
/* ------------------------------------------------------------------ */

static void auto_tick(void *arg)
{
    (void)arg;

    if (!s_auto) {
        return;
    }

    /* A live connection that is producing data: nothing to do. */
    if (ldi_client_is_connected()) {
        if (s_state != ST_LISTENING) {
            go(ST_LISTENING);
        }
        if (ldi_client_notify_total() > 0) {
            return; /* working; leave it alone */
        }
        if (elapsed_ms() > SILENCE_MS) {
            /*
             * Connected, enumerated, subscribed -- and silent. Worth
             * distinguishing from a refusal: it usually means either the wrong
             * peer, or the bike has not had its accessory pairing completed in
             * the eBike Flow app.
             */
            ble_addr_t peer;
            if (ldi_client_peer_addr(&peer)) {
                blacklist(&peer, "connected but silent");
            }
            capture_event("silent for %u ms -- disconnecting",
                          (unsigned)SILENCE_MS);
            ldi_client_disconnect();
            go(ST_IDLE);
        }
        return;
    }

    switch (s_state) {

    case ST_IDLE:
        if (elapsed_ms() < RETRY_MS) {
            return;
        }
        if (s_have_target) {
            char buf[24];
            snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x",
                     s_target.val[5], s_target.val[4], s_target.val[3],
                     s_target.val[2], s_target.val[1], s_target.val[0]);
            s_attempts++;
            capture_event("auto connect to pinned target %s", buf);
            if (ldi_client_connect(buf, s_target.type) == ESP_OK) {
                go(ST_CONNECTING);
            }
        } else {
            s_attempts++;
            if (ldi_client_scan(SCAN_MS) == ESP_OK) {
                go(ST_SCANNING);
            }
        }
        return;

    case ST_SCANNING:
        if (elapsed_ms() < SCAN_MS + 500) {
            return;
        }
        {
            /*
             * Pick the strongest candidate that advertises a 128-bit service
             * UUID. The LDI service is custom, so it cannot be a 16-bit
             * assigned number; that single test rejects almost every unrelated
             * device nearby without needing to know Bosch's UUID.
             */
            ble_addr_t cand;
            if (ldi_client_best_candidate(&cand)) {
                char buf[24];
                snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x",
                         cand.val[5], cand.val[4], cand.val[3],
                         cand.val[2], cand.val[1], cand.val[0]);
                capture_event("auto candidate %s type=%u", buf, cand.type);
                if (ldi_client_connect(buf, cand.type) == ESP_OK) {
                    go(ST_CONNECTING);
                    return;
                }
            } else {
                capture_event("auto: no candidate with a 128-bit service uuid");
            }
        }
        go(ST_IDLE);
        return;

    case ST_CONNECTING:
        if (elapsed_ms() > CONNECT_MS) {
            capture_event("auto connect timed out");
            ldi_client_disconnect();
            go(ST_IDLE);
        }
        return;

    case ST_LISTENING:
        /* Reached only when the link dropped; is_connected() handled above. */
        go(ST_IDLE);
        return;
    }
}

void ldi_auto_note_session(void)
{
    s_sessions++;
}

void ldi_auto_init(void)
{
    load_settings();

    const esp_timer_create_args_t args = {
        .callback = auto_tick, .name = "ldi_auto",
    };
    ESP_ERROR_CHECK(esp_timer_create(&args, &s_tick));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_tick, TICK_MS * 1000));

    s_state_since_us = esp_timer_get_time();

    char tgt[40];
    if (ldi_client_get_target(tgt, sizeof(tgt))) {
        ESP_LOGI(TAG, "autonomous=%s target=%s", s_auto ? "on" : "off", tgt);
    } else {
        ESP_LOGI(TAG, "autonomous=%s, no pinned target (will hunt)",
                 s_auto ? "on" : "off");
    }
}

void ldi_auto_print_status(void)
{
    char tgt[40];
    printf("auto mode       : %s\n", s_auto ? "ON" : "off");
    printf("auto state      : %s (%lu ms)\n", state_name(s_state),
           (unsigned long)elapsed_ms());
    printf("pinned target   : %s\n",
           ldi_client_get_target(tgt, sizeof(tgt)) ? tgt : "(none)");
    printf("attempts        : %lu\n", (unsigned long)s_attempts);
    printf("sessions w/ data: %lu\n", (unsigned long)s_sessions);
    printf("blacklisted     : %d\n", s_blacklist_n);
    for (int i = 0; i < s_blacklist_n; i++) {
        const uint8_t *v = s_blacklist[i].val;
        printf("   %02x:%02x:%02x:%02x:%02x:%02x\n",
               v[5], v[4], v[3], v[2], v[1], v[0]);
    }
}

void ldi_auto_clear_blacklist(void)
{
    s_blacklist_n = 0;
    printf("blacklist cleared\n");
}
