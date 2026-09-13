/*
 * bridge_console -- a small serial REPL.
 *
 * Needed for bring-up rather than as a nicety.  Two things are impossible
 * without it:
 *
 *  - Hand-verifying the 1/1024 s crank arithmetic in nRF Connect.  The
 *    scripted ride is always moving, so packet-to-packet deltas cannot be
 *    checked by hand against a known cadence.  "sim cadence 80" pins it.
 *  - Clearing bonds between pairing attempts.  Re-pairing the watch is the
 *    single most repeated operation in Stage 2, and a full erase-flash to
 *    achieve it is absurd.
 *
 * Uses esp_console over the C3 SuperMini's native USB Serial/JTAG.  (Not the
 * `scli` helper the NimBLE examples ship: that exists only to inject
 * passkeys, which NO_INPUT_NO_OUTPUT means we never need.)
 */
#include "bridge_console.h"
#include "bridge.h"
#include "cps_server.h"
#include "cps_gatt.h"
#include "cps_source.h"
#include "capture.h"
#include "data_source.h"
#include "ldi_uuids.h"
#include "ldi_client.h"
#include "sim_source.h"
#include "status_led.h"

#include <stdlib.h>
#include <string.h>

#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "host/ble_hs.h"
#include "host/ble_store.h"

static const char *TAG = "console";

/* ---------------- status ---------------- */

static int cmd_status(int argc, char **argv)
{
    (void)argc; (void)argv;

    int bond_count = 0;
    (void)ble_store_util_count(BLE_STORE_OBJ_TYPE_OUR_SEC, &bond_count);

    size_t cap_used = 0, cap_total = 0;
    uint32_t cap_recs = 0;
    capture_stats(&cap_used, &cap_total, &cap_recs);

    bool sub_power = false, sub_cadence = false;
    (void)cps_server_subscriptions(&sub_power, &sub_cadence);

    printf("firmware        : %s\n", cps_gatt_firmware_revision());
    printf("watch connected : %s\n", cps_server_is_connected() ? "yes" : "no");
    printf("subscribed      : power=%s cadence=%s\n",
           sub_power ? "yes" : "no", sub_cadence ? "yes" : "no");
    /*
     * Two separate questions, and conflating them is what made a five-hour
     * data outage invisible: the watch can be subscribed while the bridge is
     * deliberately sending nothing because there is no bike to relay.
     */
    printf("sending data    : %s\n",
           cps_source_present() ? "yes"
                                : "no -- SUSPENDED, no data source");
    printf("source          : %s (mode %s, sim phase %s)\n",
           data_source_active_name(),
           data_source_mode_name(data_source_get_mode()),
           sim_source_phase_name());
    printf("capture log     : %lu records, %s\n",
           (unsigned long)cap_recs,
           capture_full() ? "FULL -- nothing is being logged" : "ok");
    printf("led state       : %s\n", status_led_state_name(status_led_get()));
    printf("bonds stored    : %d\n", bond_count);
    printf("-- bike side --\n");
    ldi_client_print_status();
    return 0;
}

/* ---------------- bonds ---------------- */

static int cmd_bonds(int argc, char **argv)
{
    if (argc >= 2 && strcmp(argv[1], "clear") == 0) {
        bridge_clear_bonds();
        return 0;
    }

    ble_addr_t peers[MYNEWT_VAL(BLE_STORE_MAX_BONDS)];
    int num = 0;
    int rc = ble_store_util_bonded_peers(peers, &num,
                                        sizeof(peers) / sizeof(peers[0]));
    if (rc != 0) {
        printf("bonded_peers rc=%d\n", rc);
        return 1;
    }

    printf("%d bonded peer(s)\n", num);
    for (int i = 0; i < num; i++) {
        const uint8_t *v = peers[i].val;
        printf("  [%d] type=%d %02X:%02X:%02X:%02X:%02X:%02X\n",
               i, peers[i].type, v[5], v[4], v[3], v[2], v[1], v[0]);
    }
    printf("use \"bonds clear\" to remove all\n");
    return 0;
}

/* ---------------- sim ---------------- */

static int cmd_sim(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: sim script on|off\n"
               "       sim power <watts>\n"
               "       sim cadence <rpm>\n"
               "       sim set <watts> <rpm>\n"
               "       sim stop\n");
        return 1;
    }

    /* Manual mode holds the last commanded pair, so a single-value command
     * must not clobber the other one. */
    static int16_t last_power;
    static float   last_cadence;

    if (strcmp(argv[1], "script") == 0) {
        if (argc < 3) {
            printf("usage: sim script on|off\n");
            return 1;
        }
        sim_source_set_script(strcmp(argv[2], "on") == 0);
        return 0;
    }
    if (strcmp(argv[1], "stop") == 0) {
        last_power = 0;
        last_cadence = 0.f;
        sim_source_set_manual(0, 0.f);
        return 0;
    }
    if (strcmp(argv[1], "power") == 0 && argc >= 3) {
        last_power = (int16_t)atoi(argv[2]);
        sim_source_set_manual(last_power, last_cadence);
        return 0;
    }
    if (strcmp(argv[1], "cadence") == 0 && argc >= 3) {
        last_cadence = strtof(argv[2], NULL);
        sim_source_set_manual(last_power, last_cadence);
        return 0;
    }
    if (strcmp(argv[1], "set") == 0 && argc >= 4) {
        last_power   = (int16_t)atoi(argv[2]);
        last_cadence = strtof(argv[3], NULL);
        sim_source_set_manual(last_power, last_cadence);
        return 0;
    }

    printf("unknown: sim %s\n", argv[1]);
    return 1;
}

/* ---------------- bike (LDI central) ---------------- */

static int cmd_scan(int argc, char **argv)
{
    int32_t ms = 8000;
    if (argc >= 2) {
        ms = (int32_t)atoi(argv[1]);
    }
    if (ldi_client_scan(ms) != ESP_OK) {
        printf("scan failed (LDI central compiled out?)\n");
        return 1;
    }
    printf("scanning %ld ms -- results are logged as they arrive\n", (long)ms);
    return 0;
}

static int cmd_bike(int argc, char **argv)
{
    if (argc < 2) {
        /*
         * No connect command any more, deliberately.  The bridge is a GAP
         * peripheral now and the eBike is the central, so there is nothing to
         * connect TO -- and the old `bike connect` is exactly the behaviour
         * that took the phone's slot and cost a five-hour ride its data.
         * Register the bridge from eBike Flow's accessory menu instead.
         */
        printf("usage: bike disconnect            drop the link; the bike "
               "reconnects\n"
               "       bike gatt                 discovered services/chrs\n"
               "       bike dump on|off          raw notification hex dump\n"
               "       bike status\n"
               "\n"
               "The bridge advertises the Live Data Service as a solicitation\n"
               "and waits for the bike to connect. Pair it once from the\n"
               "eBike Flow app's accessory menu.\n");
        return 1;
    }

    if (strcmp(argv[1], "disconnect") == 0) {
        ldi_client_disconnect();
        return 0;
    }
    if (strcmp(argv[1], "gatt") == 0) {
        ldi_client_print_gatt();
        return 0;
    }
    if (strcmp(argv[1], "dump") == 0 && argc >= 3) {
        ldi_client_set_dump(strcmp(argv[2], "on") == 0);
        printf("raw dump %s\n", ldi_client_get_dump() ? "on" : "off");
        return 0;
    }
    if (strcmp(argv[1], "status") == 0) {
        ldi_client_print_status();
        return 0;
    }

    printf("unknown: bike %s\n", argv[1]);
    return 1;
}

/* ---------------- capture ---------------- */

static int cmd_capture(int argc, char **argv)
{
    size_t used = 0, total = 0;
    uint32_t recs = 0;

    if (argc < 2 || strcmp(argv[1], "status") == 0) {
        capture_stats(&used, &total, &recs);
        printf("capture records : %lu\n", (unsigned long)recs);
        printf("capture used    : %u / %u bytes (%.1f%%)\n",
               (unsigned)used, (unsigned)total,
               total ? 100.0 * used / total : 0.0);
        printf("capture free    : %u bytes\n", (unsigned)(total - used));
        printf("capture ready   : %s\n",
               capture_ready() ? "yes"
                               : "NO -- FULL, nothing is being logged");
        if (argc < 2) {
            printf("usage: capture status | dump | dump short | erase\n");
        }
        return 0;
    }
    if (strcmp(argv[1], "dump") == 0) {
        bool hex = !(argc >= 3 && strcmp(argv[2], "short") == 0);
        capture_dump(hex);
        return 0;
    }
    if (strcmp(argv[1], "erase") == 0) {
        printf("erasing ...\n");
        printf("%s\n", capture_erase() == ESP_OK ? "erased" : "FAILED");
        return 0;
    }
    printf("unknown: capture %s\n", argv[1]);
    return 1;
}

/* ---------------- disconnect ---------------- */

static int cmd_disconnect(int argc, char **argv)
{
    (void)argc; (void)argv;

    if (!cps_server_is_connected()) {
        printf("not connected\n");
        return 0;
    }
    bridge_disconnect_watch();
    return 0;
}

/* ---------------- source ---------------- */

static int cmd_source(int argc, char **argv)
{
    if (argc >= 2) {
        if (strcmp(argv[1], "sim") == 0) {
            data_source_set_mode(DATA_SOURCE_SIM);
        } else if (strcmp(argv[1], "bike") == 0) {
            data_source_set_mode(DATA_SOURCE_BIKE);
        } else if (strcmp(argv[1], "auto") == 0) {
            data_source_set_mode(DATA_SOURCE_AUTO);
        } else {
            printf("usage: source sim|bike|auto\n");
            return 1;
        }
    }
    printf("mode   : %s\n", data_source_mode_name(data_source_get_mode()));
    printf("active : %s\n", data_source_active_name());
    printf("\nauto uses the bike when its data is fresh and sends NOTHING\n"
           "otherwise -- it never substitutes simulated power, which would\n"
           "write fabricated data into a real workout unnoticed.\n");
    return 0;
}

/* ---------------- reboot ---------------- */

/*
 * A deterministic restart.  Resetting via the serial DTR/RTS lines is
 * unreliable on the C3's native USB Serial/JTAG -- it silently does nothing
 * often enough to invalidate a test that assumes it worked -- and pulling the
 * plug is not an option when the board is being driven over that same port.
 */
static int cmd_reboot(int argc, char **argv)
{
    (void)argc; (void)argv;
    printf("rebooting\n");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
    return 0;
}

/* ---------------- registration ---------------- */

void bridge_console_start(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_cfg.prompt = "bridge>";
    repl_cfg.max_cmdline_length = 128;

    const esp_console_cmd_t cmds[] = {
        { .command = "status",
          .help = "Connection, source, LED state and bond count",
          .func = cmd_status },
        { .command = "bonds",
          .help = "List bonded peers, or \"bonds clear\" to remove all",
          .func = cmd_bonds },
        { .command = "sim",
          .help = "Control the simulated ride (script/power/cadence/set/stop)",
          .func = cmd_sim },
        { .command = "disconnect",
          .help = "Drop the watch connection and resume advertising",
          .func = cmd_disconnect },
        { .command = "scan",
          .help = "Scan for BLE advertisers (default 8000 ms)",
          .func = cmd_scan },
        { .command = "bike",
          .help = "Bike link: connect / name / auto / target / gatt / status",
          .func = cmd_bike },
        { .command = "source",
          .help = "Select what feeds the watch: sim | bike | auto",
          .func = cmd_source },
        { .command = "reboot",
          .help = "Restart the device (reliable, unlike a serial DTR/RTS reset)",
          .func = cmd_reboot },
        { .command = "capture",
          .help = "Flash capture log: status / dump / dump short / erase",
          .func = cmd_capture },
    };

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t dev_cfg =
        ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    esp_err_t err = esp_console_new_repl_usb_serial_jtag(&dev_cfg, &repl_cfg, &repl);
#else
    esp_console_dev_uart_config_t dev_cfg = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    esp_err_t err = esp_console_new_repl_uart(&dev_cfg, &repl_cfg, &repl);
#endif
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "console init failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_ERROR_CHECK(esp_console_register_help_command());
    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
    ESP_LOGI(TAG, "console ready -- try \"help\"");
}
