"""
How long did each session last, and why did it end?

Answers the question a power-source experiment actually poses: did the supply
hold overnight, or cut out after twenty minutes?

Each boot writes a BOOT record carrying esp_reset_reason(); every record after
it carries milliseconds since that boot. So a session's duration is the largest
timestamp seen before the next BOOT record, and the NEXT boot's reason says how
the previous session ended.

Usage:
    python tools/uptime.py <capture.log>
"""
import re
import sys

# What the reset reason of the FOLLOWING boot implies about how the PREVIOUS
# session ended.
ENDED_BY = {
    "poweron":    "supply removed or cut off  <- power source gave up",
    "brownout":   "supply voltage sagged      <- power source too weak",
    "sw":         "software restart (reboot command)",
    "usb":        "reset over USB (reflash, or the serial port reset it)",
    "ext":        "external reset pin",
    "panic":      "FIRMWARE CRASH",
    "int_wdt":    "interrupt watchdog  <- firmware fault",
    "task_wdt":   "task watchdog       <- firmware fault",
    "wdt":        "watchdog            <- firmware fault",
    "deepsleep":  "woke from deep sleep",
    "pwr_glitch": "power glitch detected",
    "cpu_lockup": "CPU lockup",
}


def human(sec):
    if sec < 60:
        return "%ds" % sec
    if sec < 3600:
        return "%dm %02ds" % (sec // 60, sec % 60)
    return "%dh %02dm" % (sec // 3600, (sec % 3600) // 60)


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)

    lines = [l for l in open(sys.argv[1], encoding="utf-8",
                             errors="replace").read().splitlines() if l.strip()]

    sessions = []   # (reset_reason, max_t_ms, last_alive_t_ms, n_records)
    cur = None

    for l in lines:
        m = re.match(r"^(\d+)\s+(BOOT|ADV|EVT|RX|GATT|LINK)\s+(.*)$", l.rstrip())
        if not m:
            continue
        t_ms, typ, rest = int(m.group(1)), m.group(2), m.group(3)

        if typ == "BOOT":
            rm = re.search(r"reset=(\w+)", rest)
            cur = {"reason": rm.group(1) if rm else "?",
                   "max_t": t_ms, "alive_t": None, "n": 0,
                   "heap_first": None, "heap_last": None}
            sessions.append(cur)
            continue

        if cur is None:
            cur = {"reason": "(log starts mid-session)", "max_t": t_ms,
                   "alive_t": None, "n": 0, "heap_first": None,
                   "heap_last": None}
            sessions.append(cur)

        cur["n"] += 1
        cur["max_t"] = max(cur["max_t"], t_ms)

        am = re.search(r"alive up=(\d+)s heap=(\d+)", rest)
        if am:
            cur["alive_t"] = int(am.group(1))
            heap = int(am.group(2))
            if cur["heap_first"] is None:
                cur["heap_first"] = heap
            cur["heap_last"] = heap

    if not sessions:
        sys.exit("no sessions found in the log")

    print("%d session(s) in the log\n" % len(sessions))
    print("  #  started by     lasted      records  how it ended")
    print("  " + "-" * 74)

    for i, s in enumerate(sessions):
        # Prefer the explicit "alive" marker: it is written on a fixed 60 s
        # cadence, so it bounds uptime even when nothing else was logged.
        if s["alive_t"] is not None:
            lasted = max(s["alive_t"], s["max_t"] // 1000)
            precision = "+/-60s"
        else:
            lasted = s["max_t"] // 1000
            precision = "at least"

        if i + 1 < len(sessions):
            nxt = sessions[i + 1]["reason"]
            ended = ENDED_BY.get(nxt, nxt)
        else:
            ended = "still running (this is the current session)"

        print("  %-2d %-14s %-11s %7d  %s"
              % (i + 1, s["reason"], human(lasted), s["n"], ended))
        if precision == "at least":
            print("     %-14s (no 'alive' markers -- older firmware; duration "
                  "is a lower bound)" % "")
        if s["heap_first"] and s["heap_last"]:
            drift = s["heap_last"] - s["heap_first"]
            if abs(drift) > 4096:
                print("     %-14s heap %d -> %d (%+d bytes) <- possible leak"
                      % ("", s["heap_first"], s["heap_last"], drift))

    # The headline for a power-source experiment.
    print()
    longest = max(sessions, key=lambda s: (s["alive_t"] or s["max_t"] // 1000))
    best = longest["alive_t"] or longest["max_t"] // 1000
    print("longest session: %s" % human(best))

    power_ends = 0
    for i in range(len(sessions) - 1):
        if sessions[i + 1]["reason"] in ("poweron", "brownout", "pwr_glitch"):
            power_ends += 1
    if power_ends:
        print("%d session(s) ended in a power loss, not a firmware fault"
              % power_ends)
    else:
        print("no session ended in a power loss")


if __name__ == "__main__":
    main()
