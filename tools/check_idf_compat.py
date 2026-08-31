"""
Check the version-sensitive assumptions this firmware makes against an
ESP-IDF tree.

Everything in the firmware was originally verified against v5.3.1.  5.x is
API-stable across minors, but a handful of items were identified as
genuinely churn-prone.  This script checks them mechanically so the answer
does not depend on anyone remembering to look.

Usage:
    python tools/check_idf_compat.py %USERPROFILE%\\esp\\v5.5.5\\esp-idf
"""
import os
import re
import sys

idf = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("IDF_PATH", "")
if not idf or not os.path.isdir(idf):
    sys.exit("usage: check_idf_compat.py <path-to-esp-idf>")

NIMBLE = os.path.join(idf, "components", "bt", "host", "nimble")
HOSTINC = os.path.join(NIMBLE, "nimble", "nimble", "nimble", "host", "include", "host")
if not os.path.isdir(HOSTINC):
    # Layout differs in some releases; fall back to a search.
    for root, dirs, files in os.walk(NIMBLE):
        if os.path.basename(root) == "host" and "ble_gap.h" in files:
            HOSTINC = root
            break


def read(path):
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return f.read()
    except OSError:
        return ""


findings = []


def report(level, name, detail):
    findings.append((level, name, detail))
    mark = {"ok": "  ok  ", "info": " info ", "ACTION": "ACTION"}[level]
    print("[%s] %-34s %s" % (mark, name, detail))


print("Checking: %s\n" % idf)

ver = read(os.path.join(idf, "version.txt")).strip()
if ver:
    print("version.txt: %s\n" % ver)

gap = read(os.path.join(HOSTINC, "ble_gap.h"))
gatt = read(os.path.join(HOSTINC, "ble_gatt.h"))
att = read(os.path.join(HOSTINC, "ble_att.h"))
hs = read(os.path.join(HOSTINC, "ble_hs.h"))
# Symbols are spread across two files: the NimBLE wrapper Kconfig, and
# components/bt/Kconfig which holds BT_NIMBLE_ENABLED inside `choice BT_HOST`.
kconf = (read(os.path.join(NIMBLE, "Kconfig.in")) + "\n" +
         read(os.path.join(idf, "components", "bt", "Kconfig")))

if not gap:
    sys.exit("could not read ble_gap.h under %s" % HOSTINC)

# ---- 1. BLE_GAP_EVENT_LINK_ESTAB -----------------------------------------
# The highest-impact item.  Upstream NimBLE split link establishment out of
# BLE_GAP_EVENT_CONNECT.  If it exists, the Phase 2 MTU/DLE sequence must
# hang off the new event or it fires before the link is usable.
if "BLE_GAP_EVENT_LINK_ESTAB" in gap:
    report("ACTION", "BLE_GAP_EVENT_LINK_ESTAB",
           "PRESENT -- hang the MTU/DLE sequence off this, not _CONNECT")
else:
    report("ok", "BLE_GAP_EVENT_LINK_ESTAB",
           "absent; BLE_GAP_EVENT_CONNECT is still the link-up event")

# ---- 2. Data length: the only read-back path ----------------------------
if "BLE_GAP_EVENT_DATA_LEN_CHG" in gap:
    report("ok", "BLE_GAP_EVENT_DATA_LEN_CHG", "present (the only DLE read-back)")
else:
    report("ACTION", "BLE_GAP_EVENT_DATA_LEN_CHG", "MISSING -- rework verification")

for fn in ("ble_gap_set_data_len", "ble_gap_read_sugg_def_data_len",
           "ble_gap_write_sugg_def_data_len"):
    report("ok" if fn in gap else "ACTION", fn,
           "present" if fn in gap else "MISSING")

# Is there now a real per-connection read-back?  If so, prefer it.
if re.search(r"ble_gap_read_(current_)?data_len|ble_gap_get_data_len", gap):
    report("info", "per-connection data-len getter",
           "one appears to exist now -- prefer it over latching the event")

# ---- 3. Core API we rely on ---------------------------------------------
for fn, blob, where in (
    ("ble_gattc_exchange_mtu", gatt, "ble_gatt.h"),
    ("ble_gatts_notify_custom", gatt, "ble_gatt.h"),
    ("ble_gatts_indicate_custom", gatt, "ble_gatt.h"),
    ("ble_att_mtu", att, "ble_att.h"),
    ("ble_gap_security_initiate", gap, "ble_gap.h"),
    ("ble_gap_conn_find", gap, "ble_gap.h"),
    ("ble_gap_terminate", gap, "ble_gap.h"),
    ("ble_gap_update_params", gap, "ble_gap.h"),
):
    report("ok" if fn in blob else "ACTION", fn,
           "present in %s" % where if fn in blob else "MISSING from %s" % where)

# ---- 4. ble_hs_cfg security fields --------------------------------------
for f in ("sm_io_cap", "sm_bonding", "sm_mitm", "sm_sc",
          "sm_our_key_dist", "sm_their_key_dist", "store_status_cb"):
    report("ok" if f in hs else "ACTION", "ble_hs_cfg." + f,
           "present" if f in hs else "MISSING -- bridge.c assigns it")

# ---- 5. The PARING_COMPLETE typo ---------------------------------------
if "BLE_GAP_EVENT_PARING_COMPLETE" in gap:
    report("ok", "BLE_GAP_EVENT_PARING_COMPLETE", "still misspelled (unused here)")
elif "BLE_GAP_EVENT_PAIRING_COMPLETE" in gap:
    report("info", "BLE_GAP_EVENT_PAIRING_COMPLETE", "typo fixed upstream")

# ---- 6. Kconfig symbols used by sdkconfig.defaults ---------------------
if kconf:
    used = [
        "BT_NIMBLE_ENABLED", "BT_NIMBLE_ROLE_CENTRAL", "BT_NIMBLE_ROLE_PERIPHERAL",
        "BT_NIMBLE_ROLE_OBSERVER", "BT_NIMBLE_ROLE_BROADCASTER",
        "BT_NIMBLE_MAX_CONNECTIONS", "BT_NIMBLE_EXT_ADV",
        "BT_NIMBLE_ATT_PREFERRED_MTU", "BT_NIMBLE_MSYS_1_BLOCK_COUNT",
        "BT_NIMBLE_NVS_PERSIST", "BT_NIMBLE_MAX_BONDS", "BT_NIMBLE_MAX_CCCDS",
        "BT_NIMBLE_SECURITY_ENABLE", "BT_NIMBLE_SM_SC", "BT_NIMBLE_SM_LEGACY",
        "BT_NIMBLE_SM_LVL", "BT_NIMBLE_SVC_GAP_APPEARANCE",
        "BT_NIMBLE_SVC_GAP_PPCP_MIN_CONN_INTERVAL",
        "BT_NIMBLE_SVC_GAP_PPCP_MAX_CONN_INTERVAL",
        "BT_NIMBLE_SVC_GAP_PPCP_SLAVE_LATENCY",
        "BT_NIMBLE_SVC_GAP_PPCP_SUPERVISION_TMO",
        "BT_NIMBLE_ENABLE_CONN_REATTEMPT", "BT_NIMBLE_HOST_TASK_STACK_SIZE",
        "BT_NIMBLE_USE_ESP_TIMER",
    ]
    # Note: some are declared as `menuconfig` (e.g. BT_NIMBLE_SECURITY_ENABLE)
    # and some are indented inside a `choice` block (BT_NIMBLE_ENABLED).
    missing = [s for s in used
               if not re.search(r"^\s*(?:config|menuconfig)\s+%s\s*$" % s,
                                kconf, re.M)]
    if missing:
        report("ACTION", "Kconfig symbols", "not found: %s" % ", ".join(missing))
    else:
        report("ok", "Kconfig symbols", "all %d present" % len(used))

    # BT_NIMBLE_USE_ESP_TIMER decides ble_npl_callout_reset()'s units.  We
    # avoid that API entirely, so this is informational only.
    m = re.search(r"config BT_NIMBLE_USE_ESP_TIMER(.*?)(?=\n\s*config |\Z)",
                  kconf, re.S)
    if m and "default y" in m.group(1):
        report("info", "BT_NIMBLE_USE_ESP_TIMER",
               "defaults y -> callout units are ms; we use esp_timer anyway")

# ---- 7. C3 controller Kconfig naming ------------------------------------
c3k = read(os.path.join(idf, "components", "bt", "controller", "esp32c3",
                        "Kconfig.in"))
if c3k:
    if re.search(r"^\s*config\s+BT_CTRL_BLE_MAX_ACT\s*$", c3k, re.M):
        report("ok", "BT_CTRL_BLE_MAX_ACT", "still the C3 symbol")
    elif re.search(r"^\s*config\s+BT_LE_MAX_", c3k, re.M):
        report("ACTION", "C3 controller symbols",
               "migrated to BT_LE_* -- update sdkconfig.defaults.esp32c3")
else:
    report("info", "C3 controller Kconfig", "not found at the 5.3 path")

# ---- 8. MSYS block size / SOC_ESP_NIMBLE_CONTROLLER ---------------------
# If the C3 adopts the ESP-NimBLE controller, MSYS_1_BLOCK_SIZE drops
# 256 -> 128 and a 251-byte DLE packet no longer fits one block.
caps = read(os.path.join(idf, "components", "soc", "esp32c3", "include",
                         "soc", "soc_caps.h"))
if caps:
    if "SOC_ESP_NIMBLE_CONTROLLER" in caps:
        report("ACTION", "SOC_ESP_NIMBLE_CONTROLLER",
               "now defined for C3 -- pin BT_NIMBLE_MSYS_1_BLOCK_SIZE=256")
    else:
        report("ok", "SOC_ESP_NIMBLE_CONTROLLER",
               "not defined for C3; MSYS blocks stay 256 B")
    for cap in ("SOC_BLE_50_SUPPORTED", "SOC_BLE_SUPPORTED"):
        report("ok" if cap in caps else "info", cap,
               "defined" if cap in caps else "absent")

# ---- 9. The SM_LVL silent-drop trap -------------------------------------
svr = ""
for root, dirs, files in os.walk(NIMBLE):
    if "ble_att_svr.c" in files:
        svr = read(os.path.join(root, "ble_att_svr.c"))
        break
if svr:
    # Two spellings, and getting this wrong is dangerous in the "all clear"
    # direction.  v5.3.1 used the compile-time macro:
    #     if (MYNEWT_VAL(BLE_SM_LVL) >= 2 && !sec_state.encrypted)
    # v5.5.5 refactored it to the RUNTIME field, initialised from the same
    # Kconfig at ble_hs_cfg.c:
    #     if (ble_hs_cfg.sm_sec_lvl >= 2 && !sec_state.encrypted)
    # Matching only the first form reports "trap gone" when it is very much
    # still there.
    hits = re.findall(
        r"(?:MYNEWT_VAL\(BLE_SM_LVL\)|ble_hs_cfg\.sm_sec_lvl)\s*>=\s*2\s*&&"
        r"\s*!\s*sec_state\.encrypted", svr)
    if hits:
        report("ok", "SM_LVL silent-drop guard",
               "present (%d sites) -- keep CONFIG_BT_NIMBLE_SM_LVL=0 and never "
               "assign ble_hs_cfg.sm_sec_lvl" % len(hits))
    else:
        report("info", "SM_LVL silent-drop guard",
               "not found in either spelling; SM_LVL=0 remains correct anyway")

    # The runtime field is a new footgun: it can be set from application code,
    # not only from Kconfig.  Make sure we never do.
    ours = ""
    for f in ("components/bridge/bridge.c", "components/cps_server/cps_server.c"):
        ours += read(os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), f))
    if re.search(r"ble_hs_cfg\.sm_sec_lvl\s*=", ours):
        report("ACTION", "our sm_sec_lvl assignment",
               "this firmware SETS sm_sec_lvl -- it must stay 0")
    else:
        report("ok", "our sm_sec_lvl assignment",
               "never assigned by this firmware (correct)")

# ---- 10. protobuf-c, for Phase 3 ---------------------------------------
pbc = os.path.join(idf, "components", "protobuf-c")
if os.path.isdir(pbc):
    h = ""
    for root, dirs, files in os.walk(pbc):
        if "protobuf-c.h" in files:
            h = read(os.path.join(root, "protobuf-c.h"))
            break
    m = re.search(r'PROTOBUF_C_VERSION\s+"([^"]+)"', h)
    report("ok", "components/protobuf-c",
           "bundled, version %s" % (m.group(1) if m else "?"))
else:
    report("info", "components/protobuf-c", "not bundled in this release")

# ---- 11. Prior art we reference ----------------------------------------
for ex in ("blecsc", "bleprph", "blecent",
           "ble_multi_conn/ble_multi_conn_cent"):
    p = os.path.join(idf, "examples", "bluetooth", "nimble", *ex.split("/"))
    report("ok" if os.path.isdir(p) else "info", "example " + ex,
           "present" if os.path.isdir(p) else "MOVED or removed")

# ---- summary ------------------------------------------------------------
actions = [f for f in findings if f[0] == "ACTION"]
print()
if actions:
    print("%d item(s) NEED ACTION:" % len(actions))
    for _, name, detail in actions:
        print("  - %s: %s" % (name, detail))
    sys.exit(1)
print("No action needed: every version-sensitive assumption still holds.")
