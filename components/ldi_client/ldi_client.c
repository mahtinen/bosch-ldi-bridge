#include "ldi_client.h"
#include "ldi_auto.h"
#include "ble_link_verify.h"
#include "bike_state.h"
#include "capture.h"
#include "ldi_decode.h"
#include "ldi_uuids.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_store.h"
#include "host/ble_uuid.h"

static const char *TAG = "ldi";

#ifndef CONFIG_BRIDGE_CAPTURE_PAYLOAD_MS
#define CONFIG_BRIDGE_CAPTURE_PAYLOAD_MS 1000
#endif
#ifndef CONFIG_BRIDGE_MAX_DISCOVERED_CHRS
#define CONFIG_BRIDGE_MAX_DISCOVERED_CHRS 32
#endif
#define MAX_SEEN 16

/*
 * Either the 0xFE02 service UUID or the name match alone (100) is enough. The
 * weak signals -- OUI, pairing flags, a 128-bit UUID -- cannot reach this on
 * their own, so an unrelated neighbour never gets connected to.
 */
#define LDI_CANDIDATE_MIN_SCORE 100

/* ---- discovered GATT, kept for the console to print ------------------ */
typedef struct {
    uint16_t   def_handle;
    uint16_t   val_handle;
    uint8_t    properties;
    ble_uuid_any_t uuid;
    ble_uuid_any_t svc_uuid;
    bool       subscribed;
    uint32_t   notify_count;
    uint32_t   notify_bytes;
    uint16_t   min_len, max_len;
} chr_info_t;

static chr_info_t s_chrs[CONFIG_BRIDGE_MAX_DISCOVERED_CHRS];
static int        s_chr_count;

/* ---- scan results, so "connect by name" can work -------------------- */
typedef struct {
    ble_addr_t addr;
    char       name[32];
    int8_t     rssi;
    uint8_t    score;    /* see ldi_score_adv() */
    bool       ldi_svc;  /* advertised 0xFE02 */
    bool       ldi_name; /* name matched LDI_ADV_NAME */
} seen_t;

static seen_t s_seen[MAX_SEEN];
static int    s_seen_count;

static uint16_t   s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static ble_addr_t s_peer;
static uint32_t   s_notify_total;

/*
 * The attribute handle of ebike_uuid(eb21), the ONE characteristic that
 * carries live data (spec 2.2.3, Table 2-10). Everything else the bike exposes
 * belongs to other Bosch protocols and must not be fed to the decoder.
 */
static uint16_t   s_ldi_handle = BLE_HS_CONN_HANDLE_NONE;

/* Merged bike state, accumulated across partial notifications. */
static bike_state_t s_bike;
static uint32_t     s_decode_ok;
static uint32_t     s_decode_err;

/* Payloads deliberately not written to flash; see the rate limit in the
 * notification handler. Counted so the log cannot misrepresent the real rate. */
static uint32_t     s_notify_skipped;

static uint8_t  s_own_addr_type;
static bool     s_dump = true;
static bool     s_scanning;

/* Discovery walks services, then characteristics per service. */
static int      s_disc_svc_idx;
static struct { uint16_t start, end; ble_uuid_any_t uuid; } s_svcs[12];
static int      s_svc_count;

static int ldi_gap_event(struct ble_gap_event *event, void *arg);
static void discover_next_service_chrs(void);

/* ------------------------------------------------------------------ */
/*  Helpers                                                            */
/* ------------------------------------------------------------------ */

static const char *addr_str(const uint8_t *v)
{
    static char buf[18];
    snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x",
             v[5], v[4], v[3], v[2], v[1], v[0]);
    return buf;
}

/*
 * Rotating buffers, NOT one static buffer.
 *
 * With a single static buffer, two uuid_str() calls in the same printf both
 * return the same pointer, so both arguments render as whichever call the
 * compiler evaluated last. That silently turned every
 *   "chr %s ... svc=%s"
 * line in the first bike capture into "chr <svc> ... svc=<svc>", losing all the
 * characteristic UUIDs -- the single most valuable thing in that log.
 */
static const char *uuid_str(const ble_uuid_t *u)
{
    static char bufs[4][BLE_UUID_STR_LEN];
    static unsigned next;
    char *b = bufs[next++ % 4];
    return ble_uuid_to_str(u, b);
}

static void props_str(uint8_t p, char *out, size_t n)
{
    snprintf(out, n, "%s%s%s%s%s",
             (p & BLE_GATT_CHR_PROP_READ)      ? "R" : "-",
             (p & BLE_GATT_CHR_PROP_WRITE)     ? "W" : "-",
             (p & BLE_GATT_CHR_PROP_NOTIFY)    ? "N" : "-",
             (p & BLE_GATT_CHR_PROP_INDICATE)  ? "I" : "-",
             (p & BLE_GATT_CHR_PROP_WRITE_NO_RSP) ? "w" : "-");
}

/*
 * Hex + ASCII, 16 bytes per line.  The ASCII column matters more than it looks:
 * protobuf length-delimited string fields show up as readable runs, which is
 * often the fastest way to recognise field boundaries by eye before any
 * decoder exists.
 */
static void hexdump(const uint8_t *data, uint16_t len)
{
    char hex[16 * 3 + 1];
    char asc[17];

    for (uint16_t off = 0; off < len; off += 16) {
        int n = (len - off) < 16 ? (len - off) : 16;
        int h = 0;
        for (int i = 0; i < n; i++) {
            h += snprintf(&hex[h], sizeof(hex) - h, "%02x ", data[off + i]);
            asc[i] = (data[off + i] >= 0x20 && data[off + i] < 0x7f)
                     ? (char)data[off + i] : '.';
        }
        asc[n] = '\0';
        ESP_LOGI(TAG, "    %04x  %-48s |%s|", off, hex, asc);
    }
}

static chr_info_t *find_chr_by_val_handle(uint16_t h)
{
    for (int i = 0; i < s_chr_count; i++) {
        if (s_chrs[i].val_handle == h) {
            return &s_chrs[i];
        }
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
/*  Scanning                                                           */
/* ------------------------------------------------------------------ */

/*
 * How strongly does this advertisement look like the bike?
 *
 * Learned from a real capture rather than assumed. The bike advertises:
 *
 *   02 01 05                      Flags 0x05 -- LE Limited Discoverable
 *   03 02 02 fe                   16-bit service UUID 0xFE02
 *   13 09 "smart system eBike"    Complete Local Name
 *
 * The earlier version of this function required a 128-BIT service UUID, on the
 * reasoning that a custom service cannot be a 16-bit assigned number. That was
 * wrong: Bosch uses a SIG-allocated 16-bit member UUID (0xFE02). The bike was
 * therefore rejected on every scan, nothing ever connected, and the eBike Flow
 * app reported "pairing failed" while waiting for an accessory.
 *
 * Scored rather than boolean so the strongest match wins when several devices
 * are in range, and so a firmware revision that changes one of these signals
 * still leaves the others working.
 */
static uint8_t ldi_score_adv(const struct ble_hs_adv_fields *f,
                             const char *name, const ble_addr_t *addr,
                             bool *out_svc, bool *out_name)
{
    uint8_t score = 0;
    bool svc = false, nm = false;

    /* Advertised 0xFE02 -- the single most reliable signal. */
    for (int i = 0; i < f->num_uuids16; i++) {
        if (ble_uuid_u16(&f->uuids16[i].u) == LDI_ADV_SVC_UUID16) {
            svc = true;
            score += 100;
            break;
        }
    }

    /* Name match, substring so a revision may prefix or append. */
    if (name && name[0] && strstr(name, LDI_ADV_NAME) != NULL) {
        nm = true;
        score += 100;
    }

    /* Public address with the observed OUI: weak corroboration only, since
     * OUIs vary across hardware revisions. */
    if (addr->type == BLE_ADDR_PUBLIC &&
        addr->val[5] == LDI_OUI_0 && addr->val[4] == LDI_OUI_1 &&
        addr->val[3] == LDI_OUI_2) {
        score += 20;
    }

    /* Limited-discoverable means something is actively in pairing mode. */
    if (f->flags == LDI_ADV_FLAGS_PAIRING) {
        score += 10;
    }

    /*
     * Retained as a low-confidence fallback: if a future bike really does use a
     * 128-bit custom service, this still surfaces it as a candidate rather than
     * making the hunt blind again.
     */
    if (f->num_uuids128 > 0) {
        score += 5;
    }

    if (out_svc)  *out_svc = svc;
    if (out_name) *out_name = nm;
    return score;
}

static void remember(const ble_addr_t *addr, const char *name, int8_t rssi,
                     uint8_t score, bool ldi_svc, bool ldi_name)
{
    for (int i = 0; i < s_seen_count; i++) {
        if (memcmp(s_seen[i].addr.val, addr->val, 6) == 0) {
            if (name && name[0] && !s_seen[i].name[0]) {
                strlcpy(s_seen[i].name, name, sizeof(s_seen[i].name));
            }
            s_seen[i].rssi = rssi;
            if (score > s_seen[i].score) {
                s_seen[i].score = score;
            }
            s_seen[i].ldi_svc  |= ldi_svc;
            s_seen[i].ldi_name |= ldi_name;
            return;
        }
    }
    if (s_seen_count >= MAX_SEEN) {
        return;
    }
    s_seen[s_seen_count].addr = *addr;
    s_seen[s_seen_count].rssi = rssi;
    s_seen[s_seen_count].name[0] = '\0';
    if (name) {
        strlcpy(s_seen[s_seen_count].name, name, sizeof(s_seen[s_seen_count].name));
    }
    s_seen_count++;
}

static void log_adv(const struct ble_gap_disc_desc *d)
{
    struct ble_hs_adv_fields f;
    char name[32] = {0};

    if (ble_hs_adv_parse_fields(&f, d->data, d->length_data) != 0) {
        return;
    }
    if (f.name != NULL && f.name_len > 0) {
        size_t n = f.name_len < sizeof(name) - 1 ? f.name_len : sizeof(name) - 1;
        memcpy(name, f.name, n);
    }

    bool is_new = true;
    for (int i = 0; i < s_seen_count; i++) {
        if (memcmp(s_seen[i].addr.val, d->addr.val, 6) == 0) {
            is_new = false;
            break;
        }
    }
    bool ldi_svc = false, ldi_name = false;
    uint8_t score = ldi_score_adv(&f, name, &d->addr, &ldi_svc, &ldi_name);
    remember(&d->addr, name, d->rssi, score, ldi_svc, ldi_name);

    /*
     * One record per advertiser per scan, not per packet.
     *
     * Recording every packet fills the 2.4 MB partition in roughly two hours
     * of hunting -- long before an unattended session would even reach the
     * bike. A device's repeat advertisements carry nothing new beyond RSSI
     * jitter, so the first sighting in each scan is the whole signal.
     */
    if (!is_new) {
        return; /* also keeps the console readable */
    }
    capture_adv(d->addr.val, d->addr.type, d->rssi, d->data, d->length_data);

    ESP_LOGI(TAG, "adv %s type=%d rssi=%d score=%u%s%s name=\"%s\"",
             addr_str(d->addr.val), d->addr.type, d->rssi, score,
             ldi_svc ? " [0xFE02]" : "", ldi_name ? " [name]" : "", name);

    for (int i = 0; i < f.num_uuids16; i++) {
        ESP_LOGI(TAG, "     svc16  %s", uuid_str(&f.uuids16[i].u));
    }
    for (int i = 0; i < f.num_uuids128; i++) {
        ESP_LOGI(TAG, "     svc128 %s", uuid_str(&f.uuids128[i].u));
    }
    if (f.mfg_data != NULL && f.mfg_data_len >= 2) {
        /* Company ID is the first two octets, little-endian.  Bosch is 0x0064
         * in the SIG list; worth checking against whatever appears here. */
        ESP_LOGI(TAG, "     mfg    company=0x%04x len=%u",
                 (unsigned)(f.mfg_data[0] | (f.mfg_data[1] << 8)),
                 f.mfg_data_len);
        hexdump(f.mfg_data, f.mfg_data_len);
    }
}

esp_err_t ldi_client_scan(int32_t duration_ms)
{
    struct ble_gap_disc_params p = {0};

    s_seen_count = 0;

    /* Passive: we only want to see what is broadcast, not provoke scan
     * responses, and passive scanning is less likely to disturb a bike that
     * is already talking to something else. */
    p.passive        = 1;
    p.filter_duplicates = 0; /* we de-duplicate ourselves, keeping RSSI fresh */
    p.itvl           = 0;
    p.window         = 0;
    p.filter_policy  = 0;
    p.limited        = 0;

    int rc = ble_gap_disc(s_own_addr_type, duration_ms, &p, ldi_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_disc rc=%d", rc);
        return ESP_FAIL;
    }
    s_scanning = true;
    ESP_LOGI(TAG, "scanning for %ld ms ...", (long)duration_ms);
    return ESP_OK;
}

void ldi_client_scan_stop(void)
{
    if (s_scanning) {
        ble_gap_disc_cancel();
        s_scanning = false;
        ESP_LOGI(TAG, "scan stopped; %d advertiser(s) seen", s_seen_count);
    }
}

/* ------------------------------------------------------------------ */
/*  Connecting                                                         */
/* ------------------------------------------------------------------ */

static esp_err_t connect_to(const ble_addr_t *addr)
{
    if (s_scanning) {
        ldi_client_scan_stop();
    }
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGW(TAG, "already connected; disconnect first");
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Note the bike allows exactly ONE accessory connection. If this times out
     * repeatedly, suspect another accessory (a Kiox, a head unit, a phone
     * holding the LDI slot) rather than our own code.
     */
    int rc = ble_gap_connect(s_own_addr_type, addr, 10000, NULL,
                             ldi_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_connect rc=%d", rc);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "connecting to %s ...", addr_str(addr->val));
    return ESP_OK;
}

esp_err_t ldi_client_connect(const char *addr_str_in, uint8_t addr_type)
{
    ble_addr_t a = {0};
    unsigned v[6];

    if (sscanf(addr_str_in, "%x:%x:%x:%x:%x:%x",
               &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) {
        ESP_LOGE(TAG, "bad address \"%s\" (want aa:bb:cc:dd:ee:ff)", addr_str_in);
        return ESP_ERR_INVALID_ARG;
    }
    for (int i = 0; i < 6; i++) {
        a.val[5 - i] = (uint8_t)v[i]; /* BLE addresses are little-endian */
    }
    a.type = addr_type;
    return connect_to(&a);
}

esp_err_t ldi_client_connect_by_name(const char *substr)
{
    for (int i = 0; i < s_seen_count; i++) {
        if (s_seen[i].name[0] && strstr(s_seen[i].name, substr) != NULL) {
            ESP_LOGI(TAG, "match: \"%s\" at %s",
                     s_seen[i].name, addr_str(s_seen[i].addr.val));
            return connect_to(&s_seen[i].addr);
        }
    }
    ESP_LOGW(TAG, "no scanned advertiser matching \"%s\" -- run `scan` first",
             substr);
    return ESP_ERR_NOT_FOUND;
}

void ldi_client_disconnect(void)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGI(TAG, "not connected");
        return;
    }
    /* Always terminate cleanly: a half-closed link could leave the bike's
     * single accessory slot occupied until it is power-cycled. */
    int rc = ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    ESP_LOGI(TAG, "terminate rc=%d", rc);
}

bool ldi_client_is_connected(void)
{
    return s_conn_handle != BLE_HS_CONN_HANDLE_NONE;
}

void ldi_client_set_dump(bool on) { s_dump = on; }
bool ldi_client_get_dump(void)    { return s_dump; }

/* ------------------------------------------------------------------ */
/*  Discovery                                                          */
/* ------------------------------------------------------------------ */

static int on_chr_disc(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg)
{
    if (error->status == BLE_HS_EDONE) {
        s_disc_svc_idx++;
        discover_next_service_chrs();
        return 0;
    }
    if (error->status != 0) {
        ESP_LOGW(TAG, "chr disc status=%d", error->status);
        s_disc_svc_idx++;
        discover_next_service_chrs();
        return 0;
    }
    if (s_chr_count >= CONFIG_BRIDGE_MAX_DISCOVERED_CHRS) {
        ESP_LOGW(TAG, "characteristic table full (%d) -- raise "
                      "CONFIG_BRIDGE_MAX_DISCOVERED_CHRS",
                 CONFIG_BRIDGE_MAX_DISCOVERED_CHRS);
        return 0;
    }

    chr_info_t *c = &s_chrs[s_chr_count++];
    memset(c, 0, sizeof(*c));

    /*
     * Recognise the live-data characteristic by UUID, not by handle: handles
     * are assigned by the server and are not durable across firmware
     * revisions, whereas ebike_uuid(eb21) is fixed by the specification.
     */
    if (ldi_uuid_is(&chr->uuid.u, LDI_CHR_LIVEDATA_SHORT)) {
        s_ldi_handle = chr->val_handle;
        ESP_LOGI(TAG, "LIVE DATA characteristic found: handle=%u",
                 chr->val_handle);
        capture_event("live data characteristic eb21 at handle=%u",
                      chr->val_handle);
    }
    c->def_handle = chr->def_handle;
    c->val_handle = chr->val_handle;
    c->properties = chr->properties;
    ble_uuid_copy(&c->uuid, &chr->uuid.u);
    ble_uuid_copy(&c->svc_uuid, &s_svcs[s_disc_svc_idx].uuid.u);
    c->min_len = 0xffff;

    char pr[8];
    props_str(chr->properties, pr, sizeof(pr));
    ESP_LOGI(TAG, "  chr %s  props=%s  val_handle=%u",
             uuid_str(&chr->uuid.u), pr, chr->val_handle);
    return 0;
}

static void discover_next_service_chrs(void)
{
    if (s_disc_svc_idx >= s_svc_count) {
        /* Discovery complete: subscribe to everything notifiable. */
        ESP_LOGI(TAG, "discovery complete: %d service(s), %d characteristic(s)",
                 s_svc_count, s_chr_count);

        {
            /* Record the whole GATT map: it is the single most useful thing to
             * recover from a session run away from the computer. */
            ble_link_quality_t q;
            ble_link_verify_get(&q);
            char line[160];
            snprintf(line, sizeof(line),
                     "mtu=%u ll_rx=%u ll_tx=%u dle_seen=%d enc=%d",
                     q.att_mtu, q.max_rx_octets, q.max_tx_octets,
                     q.dle_event_seen, q.encrypted);
            capture_write(CAP_REC_LINKQUAL, 0, line, (uint16_t)strlen(line));

            for (int i = 0; i < s_svc_count; i++) {
                snprintf(line, sizeof(line), "svc %s",
                         uuid_str(&s_svcs[i].uuid.u));
                capture_write(CAP_REC_GATT, 0, line, (uint16_t)strlen(line));
            }
            for (int i = 0; i < s_chr_count; i++) {
                char pr[8];
                props_str(s_chrs[i].properties, pr, sizeof(pr));
                snprintf(line, sizeof(line), "chr %s props=%s handle=%u svc=%s",
                         uuid_str(&s_chrs[i].uuid.u), pr, s_chrs[i].val_handle,
                         uuid_str(&s_chrs[i].svc_uuid.u));
                capture_write(CAP_REC_GATT, 0, line, (uint16_t)strlen(line));
            }
        }

        int wanted = 0;
        for (int i = 0; i < s_chr_count; i++) {
            uint8_t p = s_chrs[i].properties;
            if (!(p & (BLE_GATT_CHR_PROP_NOTIFY | BLE_GATT_CHR_PROP_INDICATE))) {
                continue;
            }
            wanted++;

            /*
             * Write the CCCD directly rather than going through a helper: we
             * do not know these characteristics, so we enable whichever of
             * notify/indicate the peer actually advertises.
             */
            uint16_t val = (p & BLE_GATT_CHR_PROP_NOTIFY) ? 0x0001 : 0x0002;
            uint16_t cccd = s_chrs[i].val_handle + 1; /* conventional layout */

            int rc = ble_gattc_write_no_rsp_flat(s_conn_handle, cccd,
                                                 &val, sizeof(val));
            s_chrs[i].subscribed = (rc == 0);
            ESP_LOGI(TAG, "subscribe %s via cccd=%u %s rc=%d",
                     uuid_str(&s_chrs[i].uuid.u), cccd,
                     (val == 1) ? "notify" : "indicate", rc);
        }

        if (wanted == 0) {
            ESP_LOGW(TAG, "no notifiable characteristics found -- is this the "
                          "right device?");
        } else {
            ESP_LOGI(TAG, "subscribed to %d characteristic(s); waiting for "
                          "notifications. Pedal the bike.", wanted);
        }
        return;
    }

    ESP_LOGI(TAG, "svc %s (handles %u..%u)",
             uuid_str(&s_svcs[s_disc_svc_idx].uuid.u),
             s_svcs[s_disc_svc_idx].start, s_svcs[s_disc_svc_idx].end);

    int rc = ble_gattc_disc_all_chrs(s_conn_handle,
                                     s_svcs[s_disc_svc_idx].start,
                                     s_svcs[s_disc_svc_idx].end,
                                     on_chr_disc, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "disc_all_chrs rc=%d", rc);
        s_disc_svc_idx++;
        discover_next_service_chrs();
    }
}

static int on_svc_disc(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *svc, void *arg)
{
    if (error->status == BLE_HS_EDONE) {
        s_disc_svc_idx = 0;
        discover_next_service_chrs();
        return 0;
    }
    if (error->status != 0) {
        ESP_LOGE(TAG, "svc disc status=%d", error->status);
        return 0;
    }
    if (s_svc_count >= (int)(sizeof(s_svcs) / sizeof(s_svcs[0]))) {
        return 0;
    }
    s_svcs[s_svc_count].start = svc->start_handle;
    s_svcs[s_svc_count].end   = svc->end_handle;
    ble_uuid_copy(&s_svcs[s_svc_count].uuid, &svc->uuid.u);
    s_svc_count++;
    return 0;
}

static void start_discovery(void)
{
    s_svc_count    = 0;
    s_chr_count    = 0;
    s_disc_svc_idx = 0;

    int rc = ble_gattc_disc_all_svcs(s_conn_handle, on_svc_disc, NULL);
    ESP_LOGI(TAG, "disc_all_svcs rc=%d", rc);
}

/* ------------------------------------------------------------------ */
/*  GAP callback -- the CENTRAL's own, separate from the peripheral's   */
/* ------------------------------------------------------------------ */

static int ldi_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    /* Let the verification module consume its events first. */
    if (ble_link_verify_on_gap_event(event)) {
        return 0;
    }

    switch (event->type) {

    case BLE_GAP_EVENT_DISC:
        log_adv(&event->disc);
        return 0;

    case BLE_GAP_EVENT_DISC_COMPLETE:
        s_scanning = false;
        ESP_LOGI(TAG, "scan complete (reason=%d); %d advertiser(s) seen",
                 event->disc_complete.reason, s_seen_count);
        return 0;

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            ESP_LOGE(TAG, "connect failed status=%d", event->connect.status);
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            return 0;
        }
        s_conn_handle = event->connect.conn_handle;
        s_notify_total = 0;
        s_decode_ok = 0;
        s_decode_err = 0;
        s_ldi_handle = BLE_HS_CONN_HANDLE_NONE;
        /* Fresh state per connection: carrying values across a reconnect would
         * present stale readings as current. */
        memset(&s_bike, 0, sizeof(s_bike));
        {
            struct ble_gap_conn_desc d;
            if (ble_gap_conn_find(s_conn_handle, &d) == 0) {
                s_peer = d.peer_id_addr;
            }
        }
        ESP_LOGI(TAG, "CONNECT handle=%u", s_conn_handle);
        capture_event("connect handle=%u peer=%02x:%02x:%02x:%02x:%02x:%02x",
                      s_conn_handle, s_peer.val[5], s_peer.val[4], s_peer.val[3],
                      s_peer.val[2], s_peer.val[1], s_peer.val[0]);
        /*
         * Deliberately do NOT start verification here.  v5.5.5 posts
         * LINK_ESTAB once synchronisation actually completes; starting the
         * data-length request now could target a link that fails sync with
         * reason 0x3E, which would look exactly like the bike refusing DLE.
         */
        return 0;

#ifdef BLE_GAP_EVENT_LINK_ESTAB
    case BLE_GAP_EVENT_LINK_ESTAB:
        if (event->link_estab.status != 0) {
            ESP_LOGE(TAG, "link establish failed status=%d",
                     event->link_estab.status);
            return 0;
        }
        ESP_LOGI(TAG, "LINK ESTABLISHED handle=%u", event->link_estab.conn_handle);
        ble_link_verify_begin(event->link_estab.conn_handle);
        /*
         * Encryption is enforced per connection rather than globally, because
         * CONFIG_BT_NIMBLE_SM_LVL >= 2 would make the host silently discard
         * unencrypted incoming notifications with no error and no log.
         */
        {
            int rc = ble_gap_security_initiate(event->link_estab.conn_handle);
            ESP_LOGI(TAG, "security_initiate rc=%d%s", rc,
                     rc == 0 ? "" : " (continuing unencrypted for the dump)");
        }
        return 0;
#endif

    case BLE_GAP_EVENT_ENC_CHANGE: {
        struct ble_gap_conn_desc desc;
        bool enc = false;
        if (ble_gap_conn_find(event->enc_change.conn_handle, &desc) == 0) {
            enc = desc.sec_state.encrypted;
        }
        ble_link_verify_set_encrypted(enc);

        int bonds = 0;
        (void)ble_store_util_count(BLE_STORE_OBJ_TYPE_OUR_SEC, &bonds);

        ESP_LOGI(TAG, "ENC_CHANGE status=%d encrypted=%d authenticated=%d "
                      "bonded=%d bonds_stored=%d",
                 event->enc_change.status, enc, desc.sec_state.authenticated,
                 desc.sec_state.bonded, bonds);

        /*
         * Into the log, not just the console: whether pairing completed is the
         * one thing a session run away from the computer cannot otherwise
         * report, and it is what distinguishes "the bike refused us" from
         * "the bike accepted us but the app never registered the accessory".
         */
        capture_event("enc_change status=%d encrypted=%d authenticated=%d "
                      "bonded=%d bonds_stored=%d",
                      event->enc_change.status, enc,
                      desc.sec_state.authenticated, desc.sec_state.bonded,
                      bonds);

        /*
         * Proceed to discovery either way, and say so.  For a raw-byte dump we
         * would rather have unencrypted data plus a loud warning than no data:
         * the bike pairing has to be completed once through the eBike Flow
         * app, and finding out that has not happened is itself a useful result.
         */
        if (!enc) {
            ESP_LOGW(TAG, "link is NOT encrypted -- if the bike refuses to send "
                          "live data, complete accessory pairing in the eBike "
                          "Flow app first");
        }
        start_discovery();
        return 0;
    }

    case BLE_GAP_EVENT_NOTIFY_RX: {
        chr_info_t *c = find_chr_by_val_handle(event->notify_rx.attr_handle);
        uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);

        if (s_notify_total == 0) {
            /* First frame from this peer: the session produced data, which is
             * the outcome the autonomous hunt is looking for. */
            ldi_auto_note_session();
            capture_event("first notification from handle=%u len=%u",
                          event->notify_rx.attr_handle, len);
        }
        s_notify_total++;

        /*
         * Capture payloads at about 1 Hz, not at the full notification rate.
         *
         * While riding, the bike notifies at roughly 4 Hz -- one 80-minute ride
         * produced 20,112 notifications and 1.45 MB of flash writes. Every one
         * of those is a synchronous esp_partition_write from the NimBLE host
         * task, and crossing a sector boundary adds a blocking 4 KB erase that
         * stalls the CPU with the cache disabled. That is a lot of jitter to
         * inject into the timing of the very link being measured.
         *
         * 1 Hz is all the protocol analysis ever needed, and events (connects,
         * disconnects, decode failures, link quality) are still captured in
         * full because they are rare. Skipped frames are counted so the log
         * does not silently misrepresent the true rate.
         */
        {
            static int64_t last_cap_us;
            int64_t now_us = esp_timer_get_time();

            if (last_cap_us == 0 ||
                (now_us - last_cap_us) >= CONFIG_BRIDGE_CAPTURE_PAYLOAD_MS * 1000) {
                uint8_t cap[256];
                uint16_t n = len < sizeof(cap) ? len : sizeof(cap);
                if (os_mbuf_copydata(event->notify_rx.om, 0, n, cap) == 0) {
                    capture_notify(event->notify_rx.attr_handle, cap, n);
                }
                if (s_notify_skipped) {
                    capture_event("(%lu payload(s) not captured since the last)",
                                  (unsigned long)s_notify_skipped);
                    s_notify_skipped = 0;
                }
                last_cap_us = now_us;
            } else {
                s_notify_skipped++;
            }
        }

        if (c) {
            c->notify_count++;
            c->notify_bytes += len;
            if (len < c->min_len) c->min_len = len;
            if (len > c->max_len) c->max_len = len;
        }

        /*
         * Decode only frames from eb21. The bike also notifies on other
         * services (we saw a 65-byte frame on 00000011 at connect) which are
         * not LiveData and would produce garbage if fed to the decoder.
         */
        if (event->notify_rx.attr_handle == s_ldi_handle) {
            uint8_t frame[256];
            uint16_t n = len < sizeof(frame) ? len : sizeof(frame);
            if (os_mbuf_copydata(event->notify_rx.om, 0, n, frame) == 0) {
                uint32_t fields = 0;
                /*
                 * Merge into the running state, then publish. s_bike persists
                 * between frames on purpose: notifications are partial, so a
                 * field absent from this frame means unchanged, not zero.
                 */
                if (ldi_decode_frame(frame, n, &s_bike, &fields) == ESP_OK) {
                    bike_state_commit(&s_bike);
                    s_decode_ok++;
                } else {
                    s_decode_err++;
                    /* No recognised field is the signature of a truncated
                     * payload -- exactly the LDI-003 failure. */
                    capture_event("decode FAILED on %u bytes (truncation?)", n);
                }
            }
        }

        if (s_dump) {
            ESP_LOGI(TAG, "RX handle=%u len=%u %s%s",
                     event->notify_rx.attr_handle, len,
                     event->notify_rx.indication ? "(indication) " : "",
                     c ? uuid_str(&c->uuid.u) : "(unknown handle)");

            uint8_t buf[256];
            uint16_t n = len < sizeof(buf) ? len : sizeof(buf);
            if (os_mbuf_copydata(event->notify_rx.om, 0, n, buf) == 0) {
                hexdump(buf, n);
            }
            /*
             * A payload of exactly 20 bytes, repeatedly, is the signature of
             * DLE not having taken effect -- the empirical cross-check for the
             * one thing the API cannot report directly.
             */
            if (len == 20 && !ble_link_verify_is_ok()) {
                ESP_LOGW(TAG, "  ^ exactly 20 bytes with DLE unverified: this "
                              "is very likely transport truncation, not the "
                              "payload format");
            }
        }
        return 0;
    }

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "DISCONNECT reason=%d", event->disconnect.reason);
        capture_event("disconnect reason=%d after %lu notification(s)",
                      event->disconnect.reason, (unsigned long)s_notify_total);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ble_link_verify_reset();
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE: {
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->conn_update.conn_handle, &desc) == 0) {
            ESP_LOGI(TAG, "conn update itvl=%u(%.2fms) latency=%u timeout=%u",
                     desc.conn_itvl, desc.conn_itvl * 1.25,
                     desc.conn_latency, desc.supervision_timeout);
        }
        return 0;
    }

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
            ble_store_util_delete_peer(&desc.peer_id_addr);
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    default:
        return 0;
    }
}

/* ------------------------------------------------------------------ */
/*  Console output                                                     */
/* ------------------------------------------------------------------ */

void ldi_client_print_gatt(void)
{
    if (s_chr_count == 0) {
        printf("no GATT discovered yet\n");
        return;
    }
    printf("%-38s %-6s %-6s %-5s %8s %6s %6s\n",
           "characteristic", "props", "handle", "sub", "notifies", "min", "max");
    for (int i = 0; i < s_chr_count; i++) {
        char pr[8];
        props_str(s_chrs[i].properties, pr, sizeof(pr));
        printf("%-38s %-6s %-6u %-5s %8lu %6u %6u\n",
               uuid_str(&s_chrs[i].uuid.u), pr, s_chrs[i].val_handle,
               s_chrs[i].subscribed ? "yes" : "-",
               (unsigned long)s_chrs[i].notify_count,
               s_chrs[i].min_len == 0xffff ? 0 : s_chrs[i].min_len,
               s_chrs[i].max_len);
    }
}

void ldi_client_print_status(void)
{
    ble_link_quality_t q;
    ble_link_verify_get(&q);

    printf("bike connected  : %s\n", ldi_client_is_connected() ? "yes" : "no");
    printf("raw dump        : %s\n", s_dump ? "on" : "off");
    printf("advertisers seen: %d\n", s_seen_count);
    for (int i = 0; i < s_seen_count; i++) {
        printf("   %s  rssi=%-4d score=%-4u%s%s \"%s\"\n",
               addr_str(s_seen[i].addr.val), s_seen[i].rssi, s_seen[i].score,
               s_seen[i].ldi_svc ? " [0xFE02]" : "",
               s_seen[i].ldi_name ? " [name]" : "", s_seen[i].name);
    }
    if (ldi_client_is_connected()) {
        printf("att mtu         : %u (%s)\n", q.att_mtu, q.mtu_ok ? "ok" : "small");
        printf("ll rx / tx      : %u / %u octets\n", q.max_rx_octets, q.max_tx_octets);
        printf("dle             : %s\n",
               !q.dle_event_seen ? "UNVERIFIED (no DATA_LEN_CHG)"
                                 : (q.dle_ok ? "ok" : "too small"));
        printf("services / chrs : %d / %d\n", s_svc_count, s_chr_count);
        printf("live data handle: %u%s\n", s_ldi_handle,
               s_ldi_handle == BLE_HS_CONN_HANDLE_NONE ? " (NOT FOUND)" : "");
        printf("frames decoded  : %lu ok, %lu failed\n",
               (unsigned long)s_decode_ok, (unsigned long)s_decode_err);

        bike_state_t b;
        bike_state_snapshot(&b);
        printf("  speed         : %.2f km/h\n", b.speed_mps * 3.6f);
        printf("  cadence       : %u rpm\n", b.cadence_rpm);
        printf("  rider power   : %u W\n", b.rider_power_w);
        printf("  battery       : %u %%\n", b.battery_soc_pct);
        printf("  odometer      : %.2f km\n", b.odometer_m / 1000.0f);
        printf("  brightness    : %u lux\n", b.ambient_brightness);
        printf("  standstill    : %s\n", b.standstill ? "yes" : "no");
    }
}

/* ------------------------------------------------------------------ */
/*  Init                                                              */
/* ------------------------------------------------------------------ */

uint32_t ldi_client_notify_total(void)
{
    return s_notify_total;
}

bool ldi_client_peer_addr(ble_addr_t *out)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return false;
    }
    if (out) {
        *out = s_peer;
    }
    return true;
}

/*
 * Highest-scoring scanned advertiser that is not blacklisted. See
 * ldi_score_adv() for what the score means and why it is not simply
 * "advertises a 128-bit UUID" -- that earlier test rejected the real bike.
 *
 * Ties break on RSSI, so with two bikes in range the near one wins.
 */
bool ldi_client_best_candidate(ble_addr_t *out)
{
    int best = -1;
    for (int i = 0; i < s_seen_count; i++) {
        if (s_seen[i].score < LDI_CANDIDATE_MIN_SCORE) {
            continue;
        }
        if (ldi_auto_is_blacklisted(&s_seen[i].addr)) {
            continue;
        }
        if (best < 0 || s_seen[i].score > s_seen[best].score ||
            (s_seen[i].score == s_seen[best].score &&
             s_seen[i].rssi > s_seen[best].rssi)) {
            best = i;
        }
    }
    if (best < 0) {
        return false;
    }
    ESP_LOGI(TAG, "best candidate %s score=%u rssi=%d name=\"%s\"",
             addr_str(s_seen[best].addr.val), s_seen[best].score,
             s_seen[best].rssi, s_seen[best].name);
    if (out) {
        *out = s_seen[best].addr;
    }
    return true;
}

void ldi_client_on_sync(uint8_t own_addr_type)
{
    s_own_addr_type = own_addr_type;

    /*
     * Ask the controller to start every future link wide, so a peer that does
     * initiate DLE gets a generous answer without us having to race it.
     */
    int rc = ble_gap_write_sugg_def_data_len(BLE_LINK_TX_OCTETS, BLE_LINK_TX_TIME);
    ESP_LOGI(TAG, "write_sugg_def_data_len rc=%d", rc);
    ldi_auto_init();
    ESP_LOGI(TAG, "central ready -- `scan` then `bike connect <addr>`, "
                  "or `bike auto on` to hunt unattended");
}
