#include "ldi_client.h"
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
    uint16_t   svc_end;      /* last handle of the owning service */
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

/* When the bike link was adopted, and when it first said anything. */
static int64_t    s_adopt_us;
static int64_t    s_first_notify_us;

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

static int  ldi_scan_gap_event(struct ble_gap_event *event, void *arg);
static void discover_next_service_chrs(void);
static void start_discovery(void);
static void subscribe_live_data(void);

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

    /*
     * Nothing else here owns the radio any more -- the bridge never initiates
     * a connection, so a scan can simply start. Advertising continues
     * alongside it; scanning and advertising are independent.
     */
    int rc = ble_gap_disc(s_own_addr_type, duration_ms, &p,
                          ldi_scan_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_disc rc=%d", rc);
        capture_event("ble_gap_disc rc=%d", rc);
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

/* ------------------------------------------------------------------ */
/*  Inbound links: the eBike connects to US                            */
/* ------------------------------------------------------------------ */

/*
 * The bridge no longer connects to the bike. It advertises with the Live Data
 * Service in the Service Solicitation AD type and waits to be connected to,
 * because that is what the profile actually specifies:
 *
 *     2.1.3.4  The accessory shall use the GAP peripheral role.
 *              The eBike shall use the GAP central role.
 *
 * Doing it the other way round worked -- bonded, encrypted, streaming -- but
 * it took the eBike's single PERIPHERAL slot, which is the one the eBike Flow
 * app uses. Measured on 2026-09-12: a Flow recording and a bridge recording of
 * the same 5 h 53 ride were perfectly complementary, never once holding data
 * in the same minute, and a bench test confirmed it is first-come-first-served
 * with no eviction. Accessories are not supposed to be in that queue at all;
 * 2.1.3.3 expects several of them alongside whatever else is connected.
 *
 * Which peer is which is decided by asking. Both the bike and the watch arrive
 * through the same advertisement, so on every inbound link we look for the
 * Live Data Service on the peer: found means bike, not found means watch. That
 * needs no stored address and works on the very first connection, before any
 * bond exists -- and a service discovery per connection is mandatory anyway
 * (2.1.5.5, handles must never be persisted).
 */

/* The link we are still deciding about, if any. */
static uint16_t s_probing = BLE_HS_CONN_HANDLE_NONE;

static void adopt_connection(uint16_t conn_handle)
{
    s_conn_handle  = conn_handle;
    s_notify_total = 0;
    s_decode_ok    = 0;
    s_decode_err   = 0;
    s_ldi_handle   = BLE_HS_CONN_HANDLE_NONE;
    /* Fresh state per connection: carrying values across a reconnect would
     * present stale readings as current. */
    memset(&s_bike, 0, sizeof(s_bike));

    struct ble_gap_conn_desc d;
    if (ble_gap_conn_find(conn_handle, &d) == 0) {
        s_peer = d.peer_id_addr;
    }

    s_adopt_us        = esp_timer_get_time();
    s_first_notify_us = 0;

    ESP_LOGI(TAG, "BIKE ADOPTED handle=%u peer=%s",
             conn_handle, addr_str(s_peer.val));
    capture_event("bike adopted handle=%u peer=%02x:%02x:%02x:%02x:%02x:%02x",
                  conn_handle, s_peer.val[5], s_peer.val[4], s_peer.val[3],
                  s_peer.val[2], s_peer.val[1], s_peer.val[0]);

    /*
     * Verification, then security, then the full discovery.
     *
     * The order is forced by LDI-001: the bike does not initiate the Data
     * Length Update procedure even though the profile requires it, so the
     * accessory must -- and a 27-octet link truncates notifications to 20
     * bytes with no error anywhere, which looks exactly like a protobuf that
     * needs reverse-engineering.
     */
    ble_link_verify_begin(conn_handle);

    /*
     * A bonded bike encrypts as soon as it connects, which is BEFORE the probe
     * that identifies it has finished -- so that ENC_CHANGE went to the watch
     * path and will not come again. Discovery has to be started here instead,
     * or a reconnecting bike would sit encrypted and silent forever.
     */
    bool encrypted = (ble_gap_conn_find(conn_handle, &d) == 0) &&
                     d.sec_state.encrypted;
    ble_link_verify_set_encrypted(encrypted);

    if (encrypted) {
        ESP_LOGI(TAG, "link is already encrypted -- straight to discovery");
        start_discovery();
        return;
    }

    /*
     * Otherwise ask. As the peripheral this is a Security Request, not a
     * pairing start: the bike decides, and ENC_CHANGE will start discovery.
     */
    int rc = ble_gap_security_initiate(conn_handle);
    ESP_LOGI(TAG, "security_initiate rc=%d%s", rc,
             rc == 0 ? "" : " (the bike may decline; discovery waits on it)");
}

static int on_probe_svc(uint16_t conn_handle, const struct ble_gatt_error *error,
                        const struct ble_gatt_svc *svc, void *arg)
{
    (void)arg;

    if (error->status == 0 && svc) {
        ESP_LOGI(TAG, "handle=%u has the Live Data Service -- this is the bike",
                 conn_handle);
        s_probing = BLE_HS_CONN_HANDLE_NONE;
        adopt_connection(conn_handle);
        return 0;
    }

    if (error->status == BLE_HS_EDONE) {
        if (s_probing == conn_handle) {
            s_probing = BLE_HS_CONN_HANDLE_NONE;
            ESP_LOGI(TAG, "handle=%u has no Live Data Service -- not the bike",
                     conn_handle);
            capture_event("inbound handle=%u is not the bike", conn_handle);
        }
        return 0;
    }

    /*
     * A probe that errors is not evidence either way, so say so rather than
     * silently classifying the peer as a watch and streaming nothing.
     */
    if (s_probing == conn_handle) {
        s_probing = BLE_HS_CONN_HANDLE_NONE;
        ESP_LOGW(TAG, "probe on handle=%u failed status=%d", conn_handle,
                 error->status);
        capture_event("bike probe handle=%u FAILED status=%d",
                      conn_handle, error->status);
    }
    return 0;
}

void ldi_client_offer_inbound(uint16_t conn_handle)
{
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        /* Already have a bike. 2.2 allows only one Live Data Service, and we
         * only want one source of truth. */
        return;
    }

    static const ble_uuid128_t livedata = LDI_SVC_LIVEDATA_UUID128;

    s_probing = conn_handle;
    int rc = ble_gattc_disc_svc_by_uuid(conn_handle, &livedata.u,
                                        on_probe_svc, NULL);
    if (rc != 0) {
        s_probing = BLE_HS_CONN_HANDLE_NONE;
        ESP_LOGW(TAG, "probe start on handle=%u rc=%d", conn_handle, rc);
        capture_event("bike probe handle=%u could not start rc=%d",
                      conn_handle, rc);
    }
}

bool ldi_client_owns(uint16_t conn_handle)
{
    return conn_handle != BLE_HS_CONN_HANDLE_NONE &&
           conn_handle == s_conn_handle;
}

void ldi_client_disconnect(void)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGI(TAG, "no bike link to drop");
        return;
    }
    /*
     * Terminate cleanly so the bike frees the accessory slot and, being the
     * central in this profile, comes back and reconnects on its own.
     */
    int rc = ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    ESP_LOGI(TAG, "terminate rc=%d", rc);
    capture_event("bike link dropped by us, rc=%d", rc);
}

bool ldi_client_is_connected(void)
{
    return s_conn_handle != BLE_HS_CONN_HANDLE_NONE;
}

void ldi_client_set_dump(bool on) { s_dump = on; }
bool ldi_client_get_dump(void)    { return s_dump; }

/* ------------------------------------------------------------------ */
/*  Subscribing to the live data characteristic                        */
/* ------------------------------------------------------------------ */

/*
 * ONE characteristic, and its CCCD found by discovery rather than by guessing.
 *
 * Both halves of that sentence were wrong before, and both failed silently:
 *
 *  - It subscribed to every notifiable characteristic the bike exposed -- ten
 *    of them, on undocumented vendor channels -- which was Phase 2's
 *    discover-and-dump behaviour left in the shipping path. Spec 2.2 defines
 *    the Live Data Service as a single characteristic; everything else belongs
 *    to other Bosch protocols and only costs radio time on a link already
 *    shared with the watch.
 *
 *  - It wrote val_handle + 1 with ble_gattc_write_no_rsp_flat(). Spec 2.1.5.6.1.1
 *    says the accessory SHALL discover the CCCD, and 2.2.1.3 Table 2-9 makes
 *    Write Characteristic Descriptors mandatory for the server -- that is
 *    ATT_WRITE_REQ, not a write command. A command has no response, so rc == 0
 *    meant only "queued locally" and the status table then reported a
 *    subscription that may never have existed. On the wire that is
 *    indistinguishable from a bike that connects and says nothing.
 */

static uint16_t s_cccd_handle;   /* discovered, never assumed */

/*
 * The initial full-snapshot read (spec 2.2.3.2). Decoded through exactly the
 * same path as a notification, because it is the same protobuf message -- a
 * second decoder would be a second thing to keep correct.
 */
static int on_livedata_read(uint16_t conn_handle,
                            const struct ble_gatt_error *error,
                            struct ble_gatt_attr *attr, void *arg)
{
    (void)conn_handle; (void)arg;

    if (error->status != 0 || attr == NULL) {
        ESP_LOGW(TAG, "initial live-data read failed status=%d", error->status);
        capture_event("initial read FAILED status=%d", error->status);
        return 0;
    }

    uint8_t buf[256];
    uint16_t n = OS_MBUF_PKTLEN(attr->om);
    if (n > sizeof(buf)) {
        n = sizeof(buf);
    }
    if (os_mbuf_copydata(attr->om, 0, n, buf) != 0) {
        return 0;
    }

    capture_notify(s_ldi_handle, buf, n, 0);

    uint32_t fields = 0;
    if (ldi_decode_frame(buf, n, &s_bike, &fields) == ESP_OK) {
        bike_state_commit(&s_bike);
        s_decode_ok++;
        ESP_LOGI(TAG, "initial snapshot: %u bytes, fields=0x%08lx",
                 n, (unsigned long)fields);
        capture_event("initial snapshot %u bytes fields=0x%08lx",
                      n, (unsigned long)fields);
    } else {
        s_decode_err++;
        capture_event("initial snapshot decode FAILED on %u bytes", n);
    }
    return 0;
}

static int on_cccd_write(uint16_t conn_handle,
                         const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr, void *arg)
{
    (void)conn_handle; (void)attr; (void)arg;

    chr_info_t *c = find_chr_by_val_handle(s_ldi_handle);

    if (error->status != 0) {
        ESP_LOGE(TAG, "CCCD write FAILED status=%d -- not subscribed",
                 error->status);
        capture_event("cccd write FAILED handle=%u status=%d",
                      s_cccd_handle, error->status);
        return 0;
    }

    if (c) {
        c->subscribed = true;
    }
    ESP_LOGI(TAG, "subscribed to eb21 (cccd=%u confirmed) -- pedal the bike",
             s_cccd_handle);
    capture_event("subscribed to eb21 cccd=%u confirmed", s_cccd_handle);

    /*
     * One read, straight away.
     *
     * Spec 2.2.3.2: "When a client reads the Live Data characteristic, the
     * server shall return the latest values of all available LiveData fields."
     * Notifications only carry what CHANGED, so without this the bridge knows
     * nothing about a field until it next moves -- battery, odometer and the
     * light and lock states could stay unknown for the whole ride if they
     * happen to be steady. The read fills all of them at once.
     */
    int rc = ble_gattc_read(conn_handle, s_ldi_handle, on_livedata_read, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "initial live-data read could not be issued rc=%d", rc);
    }
    return 0;
}

static int on_dsc_disc(uint16_t conn_handle,
                       const struct ble_gatt_error *error,
                       uint16_t chr_val_handle,
                       const struct ble_gatt_dsc *dsc, void *arg)
{
    (void)chr_val_handle; (void)arg;

    if (error->status == 0 && dsc) {
        if (ble_uuid_u16(&dsc->uuid.u) == BLE_GATT_DSC_CLT_CFG_UUID16) {
            s_cccd_handle = dsc->handle;
            ESP_LOGI(TAG, "  cccd 0x2902 at handle=%u", dsc->handle);
        }
        return 0;
    }

    if (error->status != BLE_HS_EDONE) {
        ESP_LOGE(TAG, "descriptor discovery failed status=%d", error->status);
        capture_event("cccd discovery FAILED status=%d", error->status);
        return 0;
    }

    /* EDONE: the walk finished. */
    if (s_cccd_handle == 0) {
        ESP_LOGE(TAG, "eb21 has no Client Characteristic Configuration "
                      "descriptor -- cannot subscribe");
        capture_event("eb21 has NO cccd -- cannot subscribe");
        return 0;
    }

    /*
     * Notify, not indicate. Spec 2.2.3 Table 2-10 gives Live Data the
     * mandatory properties Read and Notify; Indicate is not among them, so
     * asking for it would be requesting something the server never offered.
     */
    static const uint16_t val = 0x0001;
    int rc = ble_gattc_write_flat(conn_handle, s_cccd_handle, &val, sizeof(val),
                                  on_cccd_write, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "cccd write could not be issued rc=%d", rc);
        capture_event("cccd write not issued rc=%d", rc);
    }
    return 0;
}

/*
 * Descriptors live between a characteristic's value handle and whatever comes
 * next -- the following characteristic's declaration, or the end of the
 * service. Both bounds come from discovery; none of it is assumed.
 */
static void subscribe_live_data(void)
{
    chr_info_t *c = find_chr_by_val_handle(s_ldi_handle);
    if (c == NULL) {
        ESP_LOGE(TAG, "no eb21 characteristic -- is this a Bosch smart system "
                      "bike with software v19 or newer?");
        capture_event("eb21 NOT FOUND among %d characteristics", s_chr_count);
        return;
    }

    /*
     * start_handle is the characteristic's VALUE handle, not the handle after
     * it. ble_gattc_disc_all_dscs() sets prev_handle = start_handle and then
     * requests prev_handle + 1 upwards, so passing val_handle + 1 both skips
     * the first descriptor and, when the characteristic owns exactly one (the
     * CCCD, immediately after the value), inverts the range: eb21's value sits
     * at 40 with its CCCD at 41, so 41..41 became a request for 42..41 and the
     * host rejected it with BLE_HS_EINVAL before a single packet went out.
     */
    uint16_t start = c->val_handle;
    uint16_t end   = c->svc_end;
    for (int i = 0; i < s_chr_count; i++) {
        uint16_t d = s_chrs[i].def_handle;
        if (d > c->val_handle && (d - 1) < end) {
            end = d - 1;
        }
    }
    /* There must be at least one handle AFTER the value for a descriptor to
     * live in, or there is nothing to search. */
    if (end <= start) {
        ESP_LOGE(TAG, "eb21 has no descriptor range (%u..%u)", start, end);
        capture_event("eb21 descriptor range empty %u..%u", start, end);
        return;
    }

    s_cccd_handle = 0;
    int rc = ble_gattc_disc_all_dscs(s_conn_handle, start, end,
                                     on_dsc_disc, NULL);
    ESP_LOGI(TAG, "discovering descriptors of eb21 in %u..%u rc=%d",
             start, end, rc);
    if (rc != 0) {
        capture_event("cccd discovery could not start rc=%d", rc);
    }
}
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
    c->svc_end    = s_svcs[s_disc_svc_idx].end;
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

        /*
         * One characteristic, discovered CCCD, confirmed write.
         * See subscribe_live_data() for why each of those three words matters.
         */
        subscribe_live_data();
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
        capture_event("service discovery done: %d service(s)", s_svc_count);
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
    /*
     * Captured, because everything this phase produces is written only
     * when discovery COMPLETES. A discovery that stalls left the log
     * blank between "bike adopted" and the bike hanging up 30 s later,
     * with nothing to say which step never finished.
     */
    capture_event("discovery started rc=%d", rc);
}

/* ------------------------------------------------------------------ */
/*  GAP events                                                         */
/* ------------------------------------------------------------------ */

/*
 * The scan is the only thing left that this module drives itself, and it is
 * diagnostic only -- a passive look at what is advertising nearby, used to
 * confirm the bike is in accessory pairing mode. It never connects, so it
 * cannot take a slot from anyone.
 */
static int ldi_scan_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        log_adv(&event->disc);
        return 0;

#if MYNEWT_VAL(BLE_EXT_ADV)
    case BLE_GAP_EVENT_EXT_DISC: {
        /*
         * With extended advertising enabled the host reports EVERY discovery
         * here, including ordinary legacy advertisements -- BLE_GAP_EVENT_DISC
         * simply stops arriving. Handling only the legacy event is why a scan
         * silently returned "0 advertiser(s) seen" with the bike sitting right
         * next to the board.
         *
         * The fields log_adv() needs are common to both descriptors, so this
         * narrows rather than converts.
         */
        const struct ble_gap_ext_disc_desc *e = &event->ext_disc;
        struct ble_gap_disc_desc d = {
            .event_type  = e->legacy_event_type,
            .length_data = e->length_data,
            .addr        = e->addr,
            .rssi        = e->rssi,
            .data        = e->data,
            .direct_addr = e->direct_addr,
        };
        log_adv(&d);
        return 0;
    }
#endif

    case BLE_GAP_EVENT_DISC_COMPLETE:
        s_scanning = false;
        ESP_LOGI(TAG, "scan complete (reason=%d); %d advertiser(s) seen",
                 event->disc_complete.reason, s_seen_count);
        return 0;

    default:
        return 0;
    }
}

/*
 * Events for the bike link, forwarded by the arbiter.
 *
 * There is no CONNECT case: the link is inbound, so cps_server sees the
 * connect and offers it here, and this module only starts receiving events
 * once the probe has identified the peer as the bike.
 */
int ldi_client_gap_event(struct ble_gap_event *event)
{
    /* Let the verification module consume its events first. */
    if (ble_link_verify_on_gap_event(event)) {
        return 0;
    }

    switch (event->type) {

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

        if (!enc) {
            ESP_LOGW(TAG, "link is NOT encrypted -- if the bike refuses to send "
                          "live data, register the bridge as an accessory in "
                          "the eBike Flow app first");
        }
        start_discovery();
        return 0;
    }

    case BLE_GAP_EVENT_NOTIFY_RX: {
        chr_info_t *c = find_chr_by_val_handle(event->notify_rx.attr_handle);
        uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);

        if (s_notify_total == 0) {
            s_first_notify_us = esp_timer_get_time();
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
         * does not silently misrepresent the true rate -- the count rides in
         * the captured record's own header rather than in a text event beside
         * it, because a second record per frame doubled the size of the log
         * for four bits of information and is why 2.4 MB could not hold one
         * long ride.
         */
        {
            static int64_t last_cap_us;
            int64_t now_us = esp_timer_get_time();

            if (last_cap_us == 0 ||
                (now_us - last_cap_us) >= CONFIG_BRIDGE_CAPTURE_PAYLOAD_MS * 1000) {
                uint8_t cap[256];
                uint16_t n = len < sizeof(cap) ? len : sizeof(cap);
                uint8_t skipped = s_notify_skipped > 255
                                ? 255 : (uint8_t)s_notify_skipped;
                if (os_mbuf_copydata(event->notify_rx.om, 0, n, cap) == 0) {
                    capture_notify(event->notify_rx.attr_handle, cap, n, skipped);
                }
                s_notify_skipped = 0;
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
        ESP_LOGW(TAG, "BIKE DISCONNECTED reason=%d", event->disconnect.reason);
        capture_event("bike disconnect reason=%d after %lu notification(s)",
                      event->disconnect.reason, (unsigned long)s_notify_total);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_ldi_handle  = BLE_HS_CONN_HANDLE_NONE;
        ble_link_verify_reset();
        /*
         * Nothing to do about it here. The bike is the central: 2.1.6.1.3 puts
         * reconnection after link loss on its side, and ours is only to keep
         * advertising -- which cps_server resumes as soon as the slot frees.
         */
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

void ldi_client_on_sync(uint8_t own_addr_type)
{
    s_own_addr_type = own_addr_type;

    /*
     * Ask the controller to start every future link wide, so a peer that does
     * initiate DLE gets a generous answer without us having to race it.
     */
    int rc = ble_gap_write_sugg_def_data_len(BLE_LINK_TX_OCTETS, BLE_LINK_TX_TIME);
    ESP_LOGI(TAG, "write_sugg_def_data_len rc=%d", rc);
    ESP_LOGI(TAG, "accessory ready -- advertising the Live Data Service as a "
                  "solicitation; register the bridge from eBike Flow");
}

/* ------------------------------------------------------------------ */
/*  Supervision                                                        */
/* ------------------------------------------------------------------ */

/*
 * Connected, subscribed -- and silent.
 *
 * Worth distinguishing from a bike that never connects: it usually means the
 * accessory registration was never completed in eBike Flow, so the link comes
 * up and the Live Data characteristic never notifies. Dropping the link makes
 * the bike re-establish, which is the only recovery available from this side.
 */
#define SILENCE_MS 45000

void ldi_client_supervise(void)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        s_adopt_us = 0;
        return;
    }
    if (s_notify_total > 0 || s_adopt_us == 0) {
        return;
    }
    if ((esp_timer_get_time() - s_adopt_us) < (int64_t)SILENCE_MS * 1000) {
        return;
    }

    ESP_LOGW(TAG, "bike link silent for %d ms -- dropping so it re-establishes",
             SILENCE_MS);
    capture_event("bike link silent for %d ms -- dropping", SILENCE_MS);
    s_adopt_us = 0;
    ldi_client_disconnect();
}
