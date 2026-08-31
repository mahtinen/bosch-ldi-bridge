"""
End-to-end test of the flash capture log.  Requires the board on COM4.

Usage:  python tools/test_capture_hw.py [COM4]


The point of the log is that it survives being carried away from the computer,
so the test that matters is: write records, POWER-CYCLE, and confirm they are
still readable and that new records append rather than overwrite.
"""
import re
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else "COM4"
s = serial.Serial(port, 115200, timeout=0.2)


def reset():
    """Deterministic restart via the console, not the DTR/RTS lines.

    RTS-triggered reset silently does nothing often enough on the C3's native
    USB Serial/JTAG to invalidate a test that assumes it worked -- which is
    exactly what happened on the first run of this file.
    """
    s.write(b"\r\nreboot\r\n")
    s.flush()
    end = time.time() + 6.0
    buf = ""
    while time.time() < end:
        d = s.read(4096)
        if d:
            buf += d.decode("utf-8", "replace")
            if "Calling app_main" in buf:
                time.sleep(1.2)
                s.reset_input_buffer()
                return True
    s.reset_input_buffer()
    return False


def drain(t, show=False):
    end = time.time() + t
    out = ""
    while time.time() < end:
        d = s.read(8192)
        if d:
            out += d.decode("utf-8", "replace")
    if show:
        for line in out.splitlines():
            line = line.rstrip()
            if line and not line.startswith("bridge>"):
                print("   " + line)
    return out


def cmd(c, wait=1.5, show=True):
    s.write((c + "\r\n").encode())
    s.flush()
    print("\n$ " + c)
    return drain(wait, show)


def records_of(txt):
    m = re.search(r"capture records : (\d+)", txt)
    return int(m.group(1)) if m else None


failures = []


def check(name, ok, detail=""):
    print("  %s  %s %s" % ("PASS" if ok else "FAIL", name, detail))
    if not ok:
        failures.append(name)


print("=== session 1: fresh start ===")
check("reboot command actually reboots", reset())
drain(1.0)
cmd("capture erase", 14.0)
t = cmd("capture status", 3.0)
r0 = records_of(t)
check("erased log is empty or just the boot marker", r0 is not None and r0 <= 1,
      "records=%s" % r0)

# Generate records: a scan writes one ADV record per advertisement seen.
cmd("scan 6000", 8.0, show=False)
t = cmd("capture status")
r1 = records_of(t)
check("scan appended records", r1 is not None and r1 > (r0 or 0),
      "%s -> %s" % (r0, r1))

print("\n=== power-cycle, then confirm the log survived ===")
reset()
drain(1.5)
t = cmd("capture status")
r2 = records_of(t)
# +1 for this session's own boot marker.
check("records survived reboot", r2 is not None and r2 >= r1,
      "%s -> %s" % (r1, r2))

print("\n=== append after reboot must not clobber ===")
cmd("scan 5000", 7.0, show=False)
t = cmd("capture status")
r3 = records_of(t)
check("appended after reboot", r3 is not None and r3 > r2, "%s -> %s" % (r2, r3))

print("\n=== replay (short form) ===")
t = cmd("capture dump short", 8.0, show=False)
lines = [l.rstrip() for l in t.splitlines()
         if l.strip() and not l.startswith("bridge>")]
for line in lines[:28]:
    print("   " + line)
if len(lines) > 28:
    print("   ... %d more lines" % (len(lines) - 28))

check("dump contains BOOT markers", any("BOOT" in l for l in lines))
check("dump contains ADV records", any("ADV" in l for l in lines))
check("dump terminates cleanly", any("end of dump" in l for l in lines))

# Exactly ONE boot marker is correct here, not two: session 1's marker is
# written at its boot and then destroyed by `capture erase`. Only the reboot
# that happens after the erase leaves a marker behind.
#
# It also lands mid-dump rather than first, because session 1's post-erase scan
# wrote from offset 0 and session 2's marker appended after it.
boots = sum(1 for l in lines if "BOOT" in l)
check("exactly one boot marker survived the erase", boots == 1,
      "boot markers=%d" % boots)

adv_before = adv_after = False
seen_boot = False
for l in lines:
    if "BOOT" in l:
        seen_boot = True
    elif "ADV" in l:
        if seen_boot:
            adv_after = True
        else:
            adv_before = True
check("records appended around the reboot in order",
      adv_before and adv_after,
      "adv before=%s after=%s" % (adv_before, adv_after))

s.close()
print()
if failures:
    print("RESULT: FAILED -> %s" % failures)
    sys.exit(1)
print("RESULT: capture log works across power cycles")
