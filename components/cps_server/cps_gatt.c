#include "cps_gatt.h"
#include "cps_ctrl_point.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_app_desc.h"
#include "esp_mac.h"

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "cps_gatt";

uint16_t g_cps_measurement_handle;
uint16_t g_cps_control_point_handle;
uint16_t g_csc_measurement_handle;

/* Serial number, derived from the eFuse MAC at init. */
static char s_serial[13];

/* Firmware revision, built at init from the app descriptor. */
static char s_fw_rev[DIS_FIRMWARE_REV_MAX];

/* ------------------------------------------------------------------ */
/*  Access callbacks                                                   */
/* ------------------------------------------------------------------ */

/*
 * CP Measurement is notify-only.  CPS excludes Read on it, so a read must
 * be refused rather than returning a stale packet.
 */
static int cps_access_measurement(uint16_t conn_handle, uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)ctxt; (void)arg;
    return BLE_ATT_ERR_READ_NOT_PERMITTED;
}

/* CSC Measurement is notify-only, exactly as CP Measurement is. */
static int csc_access_measurement(uint16_t conn_handle, uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)ctxt; (void)arg;
    return BLE_ATT_ERR_READ_NOT_PERMITTED;
}

static int csc_access_feature(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;

    /* uint16 here -- CP Feature is uint32. Mixing them up is the standard
     * copy-paste bug between these two services. */
    uint8_t buf[2];
    put_le16(buf, CSC_FEATURE_VALUE);
    int rc = os_mbuf_append(ctxt->om, buf, sizeof(buf));
    return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int cps_access_feature(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;

    /*
     * CP Feature is uint32.  Note that CSC Feature (0x2A5C) is uint16, so
     * the 16-bit append used by the blecsc example must not be copied here.
     * Packed endian-explicitly rather than appending a native integer.
     */
    uint8_t buf[4];
    put_le32(buf, CPS_FEATURE_VALUE);
    int rc = os_mbuf_append(ctxt->om, buf, sizeof(buf));
    return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int cps_access_sensor_location(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;

    uint8_t loc = CPS_SENSOR_LOCATION_OTHER;
    int rc = os_mbuf_append(ctxt->om, &loc, sizeof(loc));
    return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int cps_access_control_point(uint16_t conn_handle, uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)attr_handle; (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }
    return cps_ctrl_point_handle_write(conn_handle, ctxt);
}

/* A read-only constant string characteristic; used for all of DIS. */
static int dis_access_str(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle;

    const char *s = (const char *)arg;
    int rc = os_mbuf_append(ctxt->om, s, strlen(s));
    return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

/* ------------------------------------------------------------------ */
/*  The GATT database                                                  */
/* ------------------------------------------------------------------ */
/*
 * NimBLE adds the CCCD (0x2902) automatically for any characteristic with
 * NOTIFY or INDICATE set.  Do not declare it by hand.
 *
 * No BLE_GATT_CHR_F_*_ENC / _AUTHEN flags anywhere.  That is load-bearing:
 * an unbonded watch must be able to subscribe and receive data immediately.
 * Requiring encryption here would turn "does the Suunto actually perform
 * SMP bonding?" from a harmless unknown into a hard failure with a
 * confusing symptom.
 *
 * Device Information Service is hand-declared rather than using NimBLE's
 * ble_svc_dis_init(), because that service compile-time gates each
 * characteristic on BLE_SVC_DIS_<x>_READ_PERM >= 0, and
 * BLE_SVC_DIS_DEFAULT_READ_PERM defaults to -1, meaning "remove this
 * characteristic".  Only Model Number overrides it, so the stock service
 * would expose Model Number alone, losing the firmware-revision string we
 * rely on to mark synthetic-data workouts.  Declaring DIS here is
 * version-proof and yields exactly the four characteristics we want.
 */
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        /* ---- Cycling Power Service 0x1818 --------------------------- */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(CPS_SVC_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid       = BLE_UUID16_DECLARE(CPS_CHR_MEASUREMENT_UUID16),
                .access_cb  = cps_access_measurement,
                .val_handle = &g_cps_measurement_handle,
                .flags      = BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid      = BLE_UUID16_DECLARE(CPS_CHR_FEATURE_UUID16),
                .access_cb = cps_access_feature,
                .flags     = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid      = BLE_UUID16_DECLARE(CPS_CHR_SENSOR_LOCATION_UUID16),
                .access_cb = cps_access_sensor_location,
                .flags     = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid       = BLE_UUID16_DECLARE(CPS_CHR_CONTROL_POINT_UUID16),
                .access_cb  = cps_access_control_point,
                .val_handle = &g_cps_control_point_handle,
                .flags      = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_INDICATE,
            },
            { 0 },
        },
    },
    {
        /* ---- Cycling Speed and Cadence Service 0x1816 --------------- */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(CSC_SVC_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid       = BLE_UUID16_DECLARE(CSC_CHR_MEASUREMENT_UUID16),
                .access_cb  = csc_access_measurement,
                .val_handle = &g_csc_measurement_handle,
                .flags      = BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid      = BLE_UUID16_DECLARE(CSC_CHR_FEATURE_UUID16),
                .access_cb = csc_access_feature,
                .flags     = BLE_GATT_CHR_F_READ,
            },
            /*
             * No Sensor Location and no SC Control Point.  Both are optional
             * while Multiple Sensor Locations and Wheel Revolution Data are
             * unsupported, and the Control Point exists mainly to let a client
             * Set Cumulative Value on wheel revolutions we do not have.
             */
            { 0 },
        },
    },
    {
        /* ---- Device Information Service 0x180A ---------------------- */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(DIS_SVC_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid      = BLE_UUID16_DECLARE(DIS_CHR_MANUFACTURER_UUID16),
                .access_cb = dis_access_str,
                .arg       = (void *)DIS_MANUFACTURER_NAME,
                .flags     = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid      = BLE_UUID16_DECLARE(DIS_CHR_MODEL_NUMBER_UUID16),
                .access_cb = dis_access_str,
                .arg       = (void *)DIS_MODEL_NUMBER,
                .flags     = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid      = BLE_UUID16_DECLARE(DIS_CHR_SERIAL_NUMBER_UUID16),
                .access_cb = dis_access_str,
                .arg       = s_serial,
                .flags     = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid      = BLE_UUID16_DECLARE(DIS_CHR_FIRMWARE_REV_UUID16),
                .access_cb = dis_access_str,
                .arg       = s_fw_rev,
                .flags     = BLE_GATT_CHR_F_READ,
            },
            {
                /* Same string as 0x2A26: which of the two an app shows is
                 * not pinned down by the spec, so publish both. */
                .uuid      = BLE_UUID16_DECLARE(DIS_CHR_SOFTWARE_REV_UUID16),
                .access_cb = dis_access_str,
                .arg       = s_fw_rev,
                .flags     = BLE_GATT_CHR_F_READ,
            },
            { 0 },
        },
    },
    { 0 },
};

/* ------------------------------------------------------------------ */
/*  Packing                                                            */
/* ------------------------------------------------------------------ */

size_t cps_gatt_pack_measurement(uint8_t *buf, size_t buflen, int16_t power_w,
                                 uint16_t cum_crank_rev,
                                 uint16_t last_crank_evt_1024)
{
    if (buflen < CPS_MEAS_MAX_LEN) {
        return 0;
    }

    const uint16_t flags = CPS_MEAS_FLAG_CRANK_REV_DATA; /* 0x0020 */
    size_t off = 0;

    put_le16(&buf[off], flags);              off += 2;
    put_le16(&buf[off], (uint16_t)power_w);  off += 2;

    /*
     * Optional fields follow Instantaneous Power in ASCENDING FLAG-BIT
     * ORDER.  Offsets are derived from the flags rather than hardcoded, so
     * that adding e.g. Pedal Power Balance (bit 0) later correctly shifts
     * the crank data instead of silently corrupting it.  This is the
     * classic CPS packing bug.
     */
    if (flags & CPS_MEAS_FLAG_CRANK_REV_DATA) {
        put_le16(&buf[off], cum_crank_rev);       off += 2;
        put_le16(&buf[off], last_crank_evt_1024); off += 2;
    }

    return off;
}

size_t csc_gatt_pack_measurement(uint8_t *buf, size_t buflen,
                                 uint16_t cum_crank_rev,
                                 uint16_t last_crank_evt_1024)
{
    if (buflen < CSC_MEAS_MAX_LEN) {
        return 0;
    }

    /*
     * Crank only.  Wheel revolution data, if it were ever set, would come
     * FIRST -- ascending flag-bit order, same rule as CPS -- so the crank
     * fields are written at an offset derived from the flags rather than
     * hardcoded, for the same reason.
     */
    const uint8_t flags = CSC_MEAS_FLAG_CRANK_REV_DATA; /* 0x02 */
    size_t off = 0;

    buf[off] = flags; off += 1;

    if (flags & CSC_MEAS_FLAG_WHEEL_REV_DATA) {
        off += 6; /* uint32 cumulative wheel revs + uint16 last event time */
    }
    if (flags & CSC_MEAS_FLAG_CRANK_REV_DATA) {
        put_le16(&buf[off], cum_crank_rev);       off += 2;
        put_le16(&buf[off], last_crank_evt_1024); off += 2;
    }

    return off;
}

void csc_gatt_notify(uint16_t conn_handle, uint16_t cum_crank_rev,
                     uint16_t last_crank_evt_1024)
{
    uint8_t buf[CSC_MEAS_MAX_LEN];
    size_t len = csc_gatt_pack_measurement(buf, sizeof(buf), cum_crank_rev,
                                           last_crank_evt_1024);
    if (len == 0) {
        return;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, (uint16_t)len);
    if (om == NULL) {
        ESP_LOGW(TAG, "csc notify: out of mbufs");
        return;
    }

    int rc = ble_gatts_notify_custom(conn_handle, g_csc_measurement_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "csc notify rc=%d", rc);
    }
}

void cps_gatt_notify(uint16_t conn_handle, int16_t power_w,
                     uint16_t cum_crank_rev, uint16_t last_crank_evt_1024)
{
    uint8_t buf[CPS_MEAS_MAX_LEN];
    size_t len = cps_gatt_pack_measurement(buf, sizeof(buf), power_w,
                                           cum_crank_rev, last_crank_evt_1024);
    if (len == 0) {
        return;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, (uint16_t)len);
    if (om == NULL) {
        ESP_LOGW(TAG, "notify: out of mbufs");
        return;
    }

    /*
     * Log and continue; never assert.  ble_gatts_notify_custom()
     * legitimately returns non-zero on transient mbuf exhaustion
     * (BLE_HS_ENOMEM) or if the peer vanished between the timer firing and
     * this call (BLE_HS_ENOTCONN).  The blecsc example asserts on this
     * return, which would panic the device mid-ride.
     */
    int rc = ble_gatts_notify_custom(conn_handle, g_cps_measurement_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "notify rc=%d", rc);
    }
}

/* ------------------------------------------------------------------ */
/*  Boot-time self-check                                               */
/* ------------------------------------------------------------------ */
/*
 * Packs the known-good vector from the design (80 rpm sustained, 200 W,
 * 1 Hz notifications: crank period is 768 ticks exactly) and logs the bytes.
 *
 * This puts byte-layout verification in the very first lines of serial
 * output, so a packing regression is caught before anything is paired and
 * without needing nRF Connect.  Costs a few hundred bytes of flash and runs
 * once.
 */
static void cps_gatt_selftest(void)
{
    static const struct {
        int16_t  power;
        uint16_t rev;
        uint16_t evt;
        const char *expect;
    } vectors[] = {
        { 200, 1,  768, "20 00 C8 00 01 00 00 03" },
        { 200, 2, 1536, "20 00 C8 00 02 00 00 06" },
        { 200, 5, 3840, "20 00 C8 00 05 00 00 0F" },
        {   0, 5, 3840, "20 00 00 00 05 00 00 0F" }, /* stopped: crank frozen */
    };

    bool all_ok = true;

    for (size_t i = 0; i < sizeof(vectors) / sizeof(vectors[0]); i++) {
        uint8_t buf[CPS_MEAS_MAX_LEN];
        size_t len = cps_gatt_pack_measurement(buf, sizeof(buf),
                                              vectors[i].power,
                                              vectors[i].rev, vectors[i].evt);
        char hex[3 * CPS_MEAS_MAX_LEN];
        size_t o = 0;
        for (size_t b = 0; b < len; b++) {
            o += snprintf(&hex[o], sizeof(hex) - o, "%s%02X", b ? " " : "", buf[b]);
        }

        bool ok = (len == 8) && (strcmp(hex, vectors[i].expect) == 0);
        all_ok = all_ok && ok;
        ESP_LOGI(TAG, "selftest CPM[%u] %s  %s", (unsigned)i, hex,
                 ok ? "ok" : "MISMATCH");
        if (!ok) {
            ESP_LOGE(TAG, "  expected %s", vectors[i].expect);
        }
    }

    /*
     * The same crank values through the CSC packer.
     *
     * Checked against CPS on every boot because the two formats are similar
     * enough to confuse and different enough to matter: CSC's flags field is
     * uint8 where CPS's is uint16, so a shared packer -- or a copied one --
     * silently shifts both crank values by a byte and the watch reads garbage
     * cadence with no error anywhere.
     */
    for (size_t i = 0; i < sizeof(vectors) / sizeof(vectors[0]); i++) {
        uint8_t cbuf[CSC_MEAS_MAX_LEN];
        size_t clen = csc_gatt_pack_measurement(cbuf, sizeof(cbuf),
                                                vectors[i].rev, vectors[i].evt);
        /* The crank pair must be byte-identical to the CPS one, just without
         * the flags/power prefix: CPS puts it at offset 4, CSC at offset 1. */
        uint8_t pbuf[CPS_MEAS_MAX_LEN];
        size_t plen = cps_gatt_pack_measurement(pbuf, sizeof(pbuf),
                                                vectors[i].power,
                                                vectors[i].rev, vectors[i].evt);
        bool ok = (clen == CSC_MEAS_MAX_LEN) && (plen == CPS_MEAS_MAX_LEN) &&
                  (cbuf[0] == CSC_MEAS_FLAG_CRANK_REV_DATA) &&
                  (memcmp(&cbuf[1], &pbuf[4], 4) == 0);
        all_ok = all_ok && ok;
        ESP_LOGI(TAG, "selftest CSC[%u] %02X %02X %02X %02X %02X  %s",
                 (unsigned)i, cbuf[0], cbuf[1], cbuf[2], cbuf[3], cbuf[4],
                 ok ? "ok" : "MISMATCH");
    }

    /* Features and location, as a scanner would read them. */
    uint8_t f[4];
    put_le32(f, CPS_FEATURE_VALUE);
    ESP_LOGI(TAG, "selftest 0x2A65 CP feature  = %02X %02X %02X %02X (expect 08 00 00 00)",
             f[0], f[1], f[2], f[3]);
    uint8_t cf[2];
    put_le16(cf, CSC_FEATURE_VALUE);
    ESP_LOGI(TAG, "selftest 0x2A5C CSC feature = %02X %02X (expect 02 00, crank only)",
             cf[0], cf[1]);
    ESP_LOGI(TAG, "selftest 0x2A5D location    = %02X (expect 00 = Other)",
             CPS_SENSOR_LOCATION_OTHER);

    if (all_ok) {
        ESP_LOGI(TAG, "selftest PASSED");
    } else {
        ESP_LOGE(TAG, "selftest FAILED -- do not trust recorded data");
    }
}

/* ------------------------------------------------------------------ */
/*  Init                                                               */
/* ------------------------------------------------------------------ */

/*
 * The exact string the eBike (and eBike Flow) reads as the accessory software
 * version. Exposed so the console can print it: comparing what Flow shows
 * against what the board says is the only way to be certain the bike is
 * talking to the build that was just flashed, rather than a cached record.
 */
const char *cps_gatt_firmware_revision(void)
{
    return s_fw_rev;
}
esp_err_t cps_gatt_init(void)
{
    int rc;

    /* Serial number: last 3 bytes of the eFuse MAC, hex. */
    uint8_t mac[6] = {0};
    if (esp_read_mac(mac, ESP_MAC_BT) != ESP_OK) {
        (void)esp_read_mac(mac, ESP_MAC_WIFI_STA);
    }
    snprintf(s_serial, sizeof(s_serial), "%02X%02X%02X", mac[3], mac[4], mac[5]);

    /*
     * Firmware revision: `git describe` plus a short ELF hash.
     *
     * The version alone is not enough during development -- it does not change
     * between rebuilds of uncommitted work, and the point of surfacing this in
     * eBike Flow is to answer "is the bike talking to the build I just
     * flashed?". The build timestamp from the app descriptor looked like the
     * answer and is not: date and time come from __DATE__/__TIME__ baked into
     * esp_app_desc.c, which only recompiles when that file does, so it sat
     * unchanged across separate flashes.
     *
     * The ELF SHA-256 changes whenever anything in the image changes, which is
     * exactly the property required.
     */
    const esp_app_desc_t *app = esp_app_get_description();
    char sha[9] = "????????";
    (void)esp_app_get_elf_sha256(sha, sizeof(sha));

    /*
     * TWENTY CHARACTERS, hard limit.
     *
     * eBike Flow showed "v1.0.0-3-g08165e2-di" for a 32-character string: the
     * eBike reads Device Information at the default 23-byte ATT MTU -- before
     * the exchange that raises it -- and does not follow up with a Read Blob,
     * so anything past 20 bytes is simply lost. A version that silently
     * truncates is worse than a short one, because the part that identifies
     * the build is at the end.
     *
     * So: base version, '*' if the tree was dirty, and the ELF hash. At a
     * clean tag that is "2.0.0+e7730a59" -- 14 characters, with the unique
     * part intact.
     */
    const char *v = (app != NULL) ? app->version : "unknown";
    if (*v == 'v') {
        v++;                      /* git describe prefixes tags with 'v' */
    }
    char base[12];
    size_t i = 0;
    while (v[i] != '\0' && v[i] != '-' && i < sizeof(base) - 1) {
        base[i] = v[i];
        i++;
    }
    base[i] = '\0';

    snprintf(s_fw_rev, sizeof(s_fw_rev), "%s%s+%s",
             base, strstr(v, "dirty") ? "*" : "", sha);
    ESP_LOGI(TAG, "firmware revision string: \"%s\"", s_fw_rev);

    /*
     * Register GAP (0x1800) and GATT (0x1801).  The blecsc example skips
     * both, which leaves it with no Device Name characteristic, no Service
     * Changed, and, critically for us, no Peripheral Preferred Connection
     * Parameters characteristic for the PPCP Kconfig values to live in.
     */
    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_svc_gap_device_name_set(CPS_DEVICE_NAME);
    if (rc != 0) {
        ESP_LOGE(TAG, "device_name_set rc=%d", rc);
        return ESP_FAIL;
    }

    /*
     * Must be set explicitly.  The backing value is
     * MYNEWT_VAL(BLE_SVC_GAP_APPEARANCE), whose Kconfig default is 0
     * ("Unknown").  The blecsc example reads it back for its advertisement
     * without ever setting it, and so broadcasts Unknown.
     */
    rc = ble_svc_gap_device_appearance_set(CPS_APPEARANCE_POWER_SENSOR);
    if (rc != 0) {
        ESP_LOGE(TAG, "appearance_set rc=%d", rc);
        return ESP_FAIL;
    }

    cps_ctrl_point_init();

    rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "gatts_count_cfg rc=%d", rc);
        return ESP_FAIL;
    }
    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "gatts_add_svcs rc=%d", rc);
        return ESP_FAIL;
    }

    cps_gatt_selftest();

    ESP_LOGI(TAG,
             "GATT registered: CPS 0x%04X feature=0x%08X loc=0x%02X serial=%s",
             CPS_SVC_UUID16, (unsigned)CPS_FEATURE_VALUE,
             CPS_SENSOR_LOCATION_OTHER, s_serial);
    return ESP_OK;
}
