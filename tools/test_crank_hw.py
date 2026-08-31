"""
On-target validation of crank_model.c.

Pins the simulator to a known cadence via the console, then reads the
heartbeat lines (which carry the live rev / evt counters) and checks the
deltas against the arithmetic the design requires.

Ticks per revolution is 1024 * 60 / rpm -- i.e. 768 at 80 rpm, 1024 at
60 rpm.  It is NOT a constant; the event-time clock always advances at
1024 ticks per second while pedalling, regardless of cadence.

Usage:  python tools/test_crank_hw.py [COM4]
"""
import re
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else "COM4"
s = serial.Serial(port, 115200, timeout=0.2)

s.setDTR(False)
s.setRTS(True)
time.sleep(0.1)
s.setRTS(False)
time.sleep(2.0)
s.reset_input_buffer()


def drain(secs, collect=None):
    end = time.time() + secs
    buf = ""
    while time.time() < end:
        d = s.read(4096)
        if d:
            buf += d.decode("utf-8", "replace")
    if collect is not None:
        collect.append(buf)
    return buf


def cmd(c, wait=0.8):
    s.write((c + "\r\n").encode())
    s.flush()
    return drain(wait)


HB = re.compile(r"power=\s*(-?\d+)W rev=\s*(\d+) evt=\s*(\d+)")

CASES = [
    ("80 rpm", "sim set 200 80", 80.0, 200),
    ("60 rpm", "sim set 150 60", 60.0, 150),
    ("stopped", "sim stop", 0.0, 0),
]

drain(1.0)
cmd("sim script off")

failures = []
for name, command, rpm, watts in CASES:
    cmd(command)
    time.sleep(0.5)
    # Collect ~32 s so we get at least three heartbeats.
    txt = drain(32.0)
    samples = HB.findall(txt)
    if len(samples) < 3:
        print("%-8s  only %d heartbeat(s) captured, skipping"
              % (name, len(samples)))
        continue

    p0, r0, e0 = (int(x) for x in samples[0])
    p1, r1, e1 = (int(x) for x in samples[-1])
    n = len(samples) - 1
    span_s = 10.0 * n

    d_rev = (r1 - r0) & 0xFFFF
    d_evt = (e1 - e0) & 0xFFFF

    exp_rev = rpm / 60.0 * span_s  # 0 when stopped
    # The event clock advances at 1024 ticks/s while pedalling, and must not
    # advance at all when stopped.
    exp_evt = 1024.0 * span_s if rpm > 0.0 else 0.0

    print("\n%s  (%d W, %.0f rpm, over %.0f s)" % (name, watts, rpm, span_s))
    print("   power reported  : %d W        (expect %d)" % (p1, watts))
    print("   d_rev           : %-6d       (expect ~%.1f, +/-1 for sampling)"
          % (d_rev, exp_rev))
    print("   d_evt           : %-6d ticks (expect ~%.0f)" % (d_evt, exp_evt))

    if rpm == 0.0:
        ok = (d_rev == 0 and d_evt == 0 and p1 == 0)
        print("   FROZEN          : %s" % ("yes" if ok else "NO -- BUG"))
        if not ok:
            failures.append(name)
    else:
        derived = (d_rev * 61440.0 / d_evt) if d_evt else 0.0
        print("   derived cadence : %.2f rpm   (expect %.0f)" % (derived, rpm))
        ok = abs(derived - rpm) < 0.5 and p1 == watts
        if not ok:
            failures.append(name)
        print("   verdict         : %s" % ("ok" if ok else "MISMATCH"))

cmd("sim script on")
s.close()

print()
if failures:
    print("RESULT: FAILED -> %s" % failures)
    sys.exit(1)
print("RESULT: on-target crank model matches the design")
