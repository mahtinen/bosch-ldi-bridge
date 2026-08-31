"""
Phase 4 wiring test, at the desk with no bike present.

The property that matters is a negative one: in auto mode with no bike, the
bridge must send NOTHING rather than quietly substituting simulated power,
which would write fabricated data into a real workout with no indication.
"""
import re
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else "COM4"
s = serial.Serial(port, 115200, timeout=0.2)


def drain(t):
    end = time.time() + t
    out = ""
    while time.time() < end:
        d = s.read(8192)
        if d:
            out += d.decode("utf-8", "replace")
    return out


def cmd(c, wait=2.0, show=False):
    s.write((c + "\r\n").encode())
    s.flush()
    txt = drain(wait)
    if show:
        print("\n$ " + c)
        for line in txt.splitlines():
            ln = line.rstrip()
            if ln and not ln.startswith("bridge>") and c not in ln:
                print("   " + ln)
    return txt


failures = []


def check(name, ok, detail=""):
    print("  %s  %-50s %s" % ("PASS" if ok else "FAIL", name, detail))
    if not ok:
        failures.append(name)


s.write(b"\r\n")
drain(0.8)

print("=== default mode ===")
t = cmd("source", 2.5, show=True)
m = re.search(r"mode   : (\w+)", t)
check("defaults to auto, not bike", m and m.group(1) == "auto",
      "mode=%s" % (m.group(1) if m else "?"))

print("\n=== auto with no bike must yield NO data ===")
t = cmd("source", 2.5)
a = re.search(r"active : (\S+)", t)
check("active source is 'none' with no bike", a and a.group(1) == "none",
      "active=%s" % (a.group(1) if a else "?"))

print("\n=== explicit sim still works (the test harness) ===")
cmd("source sim")
cmd("sim script off")
cmd("sim set 200 80")
time.sleep(1.5)
t = cmd("source", 2.5)
a = re.search(r"active : (\S+)", t)
check("sim mode reports active=sim", a and a.group(1) == "sim",
      "active=%s" % (a.group(1) if a else "?"))

# The heartbeat should show the simulator driving the crank counters again.
t = drain(11.0)
hb = re.findall(r"power=\s*(-?\d+)W rev=\s*(\d+) evt=\s*(\d+)", t)
if hb:
    check("sim drives power through to the CPS layer", int(hb[-1][0]) == 200,
          "power=%sW rev=%s evt=%s" % hb[-1])
else:
    check("heartbeat seen", False, "no heartbeat captured")

print("\n=== bike mode with no bike must also yield nothing ===")
cmd("source bike")
time.sleep(1.5)
t = cmd("source", 2.5)
a = re.search(r"active : (\S+)", t)
check("bike mode reports stale, not fabricated data",
      a and "bike" in a.group(1), "active=%s" % (a.group(1) if a else "?"))

t = drain(11.0)
hb = re.findall(r"power=\s*(-?\d+)W", t)
if hb:
    check("power is 0 in bike mode with no bike", int(hb[-1]) == 0,
          "power=%sW" % hb[-1])

print("\n=== restore auto ===")
cmd("source auto")
cmd("sim script on")
t = cmd("status", 3.0, show=True)

s.close()
print()
if failures:
    print("RESULT: FAILED -> %s" % failures)
    sys.exit(1)
print("RESULT: Phase 4 wiring behaves correctly with no bike present")
