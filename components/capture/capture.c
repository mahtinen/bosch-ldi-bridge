#include "capture.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "capture";

#define SECTOR 4096
#define ALIGN4(n) (((n) + 3u) & ~3u)
/* Caps the automatic buffers in capture_write()/capture_dump() at ~280 bytes,
 * which is safe on the NimBLE host, esp_timer and console task stacks.  251 is
 * the largest BLE payload possible with data length extension, plus the 2-byte
 * attribute handle capture_notify() prepends. */
#define MAX_PAYLOAD 256

static const esp_partition_t *s_part;
static size_t   s_off;          /* append point */
static uint32_t s_records;
static int      s_erased_upto;  /* highest sector index known erased, -1 none */
static SemaphoreHandle_t s_lock;

/* ------------------------------------------------------------------ */
/*  Init: walk the existing records to find the append point           */
/* ------------------------------------------------------------------ */

esp_err_t capture_init(void)
{
    if (s_part) {
        return ESP_OK;
    }

    s_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                      (esp_partition_subtype_t)0x40, "capture");
    if (!s_part) {
        ESP_LOGE(TAG, "no 'capture' partition -- did the partition table flash?");
        return ESP_ERR_NOT_FOUND;
    }

    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) {
        return ESP_ERR_NO_MEM;
    }

    /*
     * Append-only, so the log is a contiguous run of valid records followed by
     * erased flash.  Walk it a block at a time rather than reading each header
     * separately -- a full partition is ~70k records and per-record reads would
     * make boot noticeably slow.
     *
     * The block buffer is heap-allocated on purpose. capture_init() runs on the
     * main task, whose stack is 3584 bytes by default; a 4096-byte automatic
     * here overflows it and panics with a stack protection fault before the
     * first log line even appears.
     */
    uint8_t *buf = malloc(SECTOR);
    if (!buf) {
        ESP_LOGE(TAG, "no memory for the scan buffer");
        return ESP_ERR_NO_MEM;
    }
    size_t off = 0;
    bool done = false;

    while (off < s_part->size && !done) {
        size_t chunk = s_part->size - off;
        if (chunk > SECTOR) {
            chunk = SECTOR;
        }
        if (esp_partition_read(s_part, off, buf, chunk) != ESP_OK) {
            break;
        }

        size_t p = 0;
        while (p + sizeof(capture_hdr_t) <= chunk) {
            capture_hdr_t h;
            memcpy(&h, &buf[p], sizeof(h));

            if (h.magic != CAPTURE_MAGIC) {
                done = true;
                break;
            }
            size_t reclen = ALIGN4(sizeof(capture_hdr_t) + h.len);
            if (h.len > MAX_PAYLOAD || reclen == 0) {
                /* Corrupt length: stop here rather than chasing garbage. */
                done = true;
                break;
            }
            if (p + reclen > chunk) {
                /* Record straddles the block boundary; resume from its start. */
                break;
            }
            p += reclen;
            s_records++;
        }

        if (done) {
            off += p;
        } else if (p == 0) {
            /* Nothing parsed at all -- treat as the end. */
            break;
        } else {
            off += p;
        }
    }

    free(buf);

    s_off = off;

    /*
     * Whether the sector at the append point is safe to write depends on where
     * in the sector we landed:
     *
     *  - Mid-sector: append-only guarantees the tail is still erased, so it can
     *    be written without erasing (and erasing would destroy the records
     *    already in that sector).
     *  - Exactly on a boundary (including offset 0 on a partition holding
     *    unparseable stale data): that sector may be dirty, so mark it NOT
     *    erased and let the first write erase it. Skipping this would write
     *    into dirty flash, which silently ANDs bits rather than failing.
     */
    if ((s_off % SECTOR) == 0) {
        s_erased_upto = (int)(s_off / SECTOR) - 1;
    } else {
        s_erased_upto = (int)(s_off / SECTOR);
    }

    ESP_LOGI(TAG, "log ready: %u records, %u/%u bytes used (%.1f%%)",
             (unsigned)s_records, (unsigned)s_off, (unsigned)s_part->size,
             100.0 * s_off / s_part->size);

    /*
     * Record WHY we booted.  A session that ends away from the computer leaves
     * only the log to explain itself, and the difference between a brownout, a
     * panic and a clean restart is the difference between a power problem and a
     * firmware problem.  Diagnosing the first bike session took a chain of
     * inference from a dimming power LED; esp_reset_reason() says it outright.
     */
    static const char *const reasons[] = {
        "unknown", "poweron", "ext", "sw", "panic", "int_wdt", "task_wdt",
        "wdt", "deepsleep", "brownout", "sdio", "usb", "jtag", "efuse",
        "pwr_glitch", "cpu_lockup",
    };
    esp_reset_reason_t rr = esp_reset_reason();
    const char *why = ((unsigned)rr < sizeof(reasons) / sizeof(reasons[0]))
                      ? reasons[rr] : "?";

    char ver[64];
    int n = snprintf(ver, sizeof(ver), "boot reset=%s(%d) heap=%u",
                     why, (int)rr, (unsigned)esp_get_free_heap_size());
    capture_write(CAP_REC_BOOT, (uint8_t)rr, ver, (uint16_t)(n > 0 ? n : 0));

    if (rr == ESP_RST_BROWNOUT) {
        ESP_LOGE(TAG, "PREVIOUS SESSION ENDED IN A BROWNOUT -- check the power "
                      "source, not the firmware");
    }
    return ESP_OK;
}

bool capture_ready(void)
{
    return s_part != NULL && s_off < s_part->size;
}

void capture_stats(size_t *used, size_t *total, uint32_t *records)
{
    if (used)    *used = s_off;
    if (total)   *total = s_part ? s_part->size : 0;
    if (records) *records = s_records;
}

/* ------------------------------------------------------------------ */
/*  Append                                                             */
/* ------------------------------------------------------------------ */

esp_err_t capture_write(capture_rec_type_t type, uint8_t flags,
                        const void *payload, uint16_t len)
{
    if (!s_part) {
        return ESP_ERR_INVALID_STATE;
    }
    if (len > MAX_PAYLOAD) {
        len = MAX_PAYLOAD;
    }

    size_t reclen = ALIGN4(sizeof(capture_hdr_t) + len);

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(200)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = ESP_OK;

    if (s_off + reclen > s_part->size) {
        /* Full. Say so once, then stay quiet. */
        static bool warned;
        if (!warned) {
            warned = true;
            ESP_LOGW(TAG, "capture partition FULL at %u records -- "
                          "run `capture erase`", (unsigned)s_records);
        }
        err = ESP_ERR_NO_MEM;
        goto out;
    }

    /*
     * Erase any sector this record reaches into that we have not already
     * erased.  Everything at or beyond the append point is either erased or
     * stale from a previous session, so erasing ahead is always safe.
     */
    int last_sector = (int)((s_off + reclen - 1) / SECTOR);
    while (s_erased_upto < last_sector) {
        s_erased_upto++;
        err = esp_partition_erase_range(s_part,
                                        (size_t)s_erased_upto * SECTOR, SECTOR);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "erase sector %d failed: %s",
                     s_erased_upto, esp_err_to_name(err));
            goto out;
        }
    }

    uint8_t rec[sizeof(capture_hdr_t) + MAX_PAYLOAD + 4];
    memset(rec, 0xFF, reclen); /* pad bytes stay erased-looking */

    capture_hdr_t h = {
        .magic = CAPTURE_MAGIC,
        .type  = (uint8_t)type,
        .flags = flags,
        .len   = len,
        .t_us  = (uint64_t)esp_timer_get_time(),
    };
    memcpy(rec, &h, sizeof(h));
    if (len && payload) {
        memcpy(rec + sizeof(h), payload, len);
    }

    err = esp_partition_write(s_part, s_off, rec, reclen);
    if (err == ESP_OK) {
        s_off += reclen;
        s_records++;
    } else {
        ESP_LOGE(TAG, "write at %u failed: %s",
                 (unsigned)s_off, esp_err_to_name(err));
    }

out:
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t capture_event(const char *fmt, ...)
{
    char line[192];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n < 0) {
        return ESP_FAIL;
    }
    if (n > (int)sizeof(line) - 1) {
        n = sizeof(line) - 1;
    }
    return capture_write(CAP_REC_EVENT, 0, line, (uint16_t)n);
}

esp_err_t capture_notify(uint16_t attr_handle, const void *data, uint16_t len)
{
    uint8_t buf[2 + MAX_PAYLOAD];
    if (len > MAX_PAYLOAD - 2) {
        len = MAX_PAYLOAD - 2;
    }
    buf[0] = (uint8_t)(attr_handle & 0xff);
    buf[1] = (uint8_t)(attr_handle >> 8);
    if (len && data) {
        memcpy(&buf[2], data, len);
    }
    return capture_write(CAP_REC_NOTIFY, 0, buf, (uint16_t)(len + 2));
}

esp_err_t capture_adv(const uint8_t addr[6], uint8_t addr_type, int8_t rssi,
                      const uint8_t *data, uint8_t len)
{
    uint8_t buf[8 + 31];
    memcpy(buf, addr, 6);
    buf[6] = addr_type;
    buf[7] = (uint8_t)rssi;
    if (len > 31) {
        len = 31;
    }
    if (len && data) {
        memcpy(&buf[8], data, len);
    }
    return capture_write(CAP_REC_ADV, addr_type, buf, (uint16_t)(len + 8));
}

/* ------------------------------------------------------------------ */
/*  Dump                                                               */
/* ------------------------------------------------------------------ */

static const char *type_name(uint8_t t)
{
    switch (t) {
    case CAP_REC_BOOT:     return "BOOT";
    case CAP_REC_ADV:      return "ADV ";
    case CAP_REC_EVENT:    return "EVT ";
    case CAP_REC_NOTIFY:   return "RX  ";
    case CAP_REC_GATT:     return "GATT";
    case CAP_REC_LINKQUAL: return "LINK";
    default:               return "??  ";
    }
}

static void print_hex(const uint8_t *d, uint16_t len, const char *indent)
{
    for (uint16_t off = 0; off < len; off += 16) {
        int n = (len - off) < 16 ? (len - off) : 16;
        printf("%s%04x  ", indent, off);
        for (int i = 0; i < 16; i++) {
            if (i < n) printf("%02x ", d[off + i]);
            else       printf("   ");
        }
        printf(" |");
        for (int i = 0; i < n; i++) {
            uint8_t c = d[off + i];
            putchar((c >= 0x20 && c < 0x7f) ? c : '.');
        }
        printf("|\n");
    }
}

void capture_dump(bool hex_payloads)
{
    if (!s_part) {
        printf("capture: no partition\n");
        return;
    }

    printf("==== capture dump: %u records, %u bytes ====\n",
           (unsigned)s_records, (unsigned)s_off);
    printf("# t_ms      type  detail\n");

    uint8_t rec[sizeof(capture_hdr_t) + MAX_PAYLOAD + 4];
    size_t off = 0;
    uint32_t n = 0;

    while (off + sizeof(capture_hdr_t) <= s_off) {
        capture_hdr_t h;
        if (esp_partition_read(s_part, off, &h, sizeof(h)) != ESP_OK) {
            break;
        }
        if (h.magic != CAPTURE_MAGIC || h.len > MAX_PAYLOAD) {
            break;
        }

        size_t reclen = ALIGN4(sizeof(h) + h.len);
        if (esp_partition_read(s_part, off, rec, reclen) != ESP_OK) {
            break;
        }
        const uint8_t *p = rec + sizeof(h);

        printf("%-10llu %s  ", (unsigned long long)(h.t_us / 1000), type_name(h.type));

        switch (h.type) {
        case CAP_REC_BOOT:
        case CAP_REC_EVENT:
        case CAP_REC_GATT:
        case CAP_REC_LINKQUAL:
            printf("%.*s\n", (int)h.len, (const char *)p);
            break;

        case CAP_REC_ADV:
            if (h.len >= 8) {
                printf("%02x:%02x:%02x:%02x:%02x:%02x type=%u rssi=%d len=%u\n",
                       p[5], p[4], p[3], p[2], p[1], p[0], p[6], (int8_t)p[7],
                       h.len - 8);
                if (hex_payloads && h.len > 8) {
                    print_hex(p + 8, h.len - 8, "                     ");
                }
            } else {
                printf("(short)\n");
            }
            break;

        case CAP_REC_NOTIFY:
            if (h.len >= 2) {
                uint16_t handle = (uint16_t)(p[0] | (p[1] << 8));
                printf("handle=%u len=%u\n", handle, h.len - 2);
                if (hex_payloads && h.len > 2) {
                    print_hex(p + 2, h.len - 2, "                     ");
                }
            } else {
                printf("(short)\n");
            }
            break;

        default:
            printf("len=%u\n", h.len);
            break;
        }

        off += reclen;
        n++;

        /* Yield periodically: a long dump must not starve the BLE host. */
        if ((n % 64) == 0) {
            fflush(stdout);
            vTaskDelay(1);
        }
    }

    printf("==== end of dump (%u records) ====\n", (unsigned)n);
}

esp_err_t capture_erase(void)
{
    if (!s_part) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = esp_partition_erase_range(s_part, 0, s_part->size);
    if (err == ESP_OK) {
        s_off = 0;
        s_records = 0;
        s_erased_upto = (int)(s_part->size / SECTOR) - 1;
    }
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "erase %s", esp_err_to_name(err));
    return err;
}
