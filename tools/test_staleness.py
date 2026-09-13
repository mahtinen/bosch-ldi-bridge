"""
Replay a capture through the old and new power/cadence gating rules.

This is the regression test for the defect that cost the 2026-09-12 ride most
of its data. LDI omits a field when it has not changed (spec 2.2.4.3), so
"no update for N seconds" means "unchanged", not "gone" -- but the firmware
zeroed both rider power and cadence after 3 s, against fields the bike sends
every 3.0-4.8 s. The result was 0 W for roughly half of every ride on a
perfectly healthy link.

Mirrors data_source.c the way test_crank_model.py mirrors crank_model.c.

Usage:
    python tools/test_staleness.py <capture.log>

NOTE ON THE NUMBERS: the capture stores about one frame in five, so the replay
sees a fifth of the updates the firmware saw. Every figure here is therefore a
LOWER bound on the real coverage -- which is the safe direction for a test.
"""
import sys

sys.path.insert(0, __import__("os").path.dirname(__file__))
from decode_ldi import load_frames, parse  # noqa: E402

F_SPEED, F_CADENCE, F_POWER, F_ODO, F_STANDSTILL = 1, 2, 5, 12, 25

OLD_STALE_MS   = 3000
VALUE_HOLD_MS  = 15000
SILENT_MS      = 8000


def replay(frames, hold_ms, use_standstill):
    """Tick once a second and count seconds that carry a real rider power."""
    last = {}
    standstill = False
    real = zeroed = suspended = 0
    i = 0
    t = frames[0][0]
    end = frames[-1][0]
    while t <= end:
        while i < len(frames) and frames[i][0] <= t:
            for fnum, _, val in parse(frames[i][1]):
                last[fnum] = frames[i][0]
                if fnum == F_STANDSTILL:
                    standstill = bool(val)
            i += 1
        alive = any(t - last.get(f, -10**9) <= SILENT_MS
                    for f in (F_ODO, F_STANDSTILL))
        if not alive:
            suspended += 1
        elif use_standstill and standstill:
            zeroed += 1                      # a real stop: zero is a measurement
        elif t - last.get(F_POWER, -10**9) <= hold_ms:
            real += 1
        else:
            zeroed += 1
        t += 1000
    return real, zeroed, suspended


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    frames = load_frames(sys.argv[1], 40)
    if not frames:
        sys.exit("no frames on handle 40")
    span = (frames[-1][0] - frames[0][0]) / 1000.0
    print("replaying %d frames over %.0f s\n" % (len(frames), span))

    rows = [("old: zero after %d ms" % OLD_STALE_MS,
             replay(frames, OLD_STALE_MS, False)),
            ("new: hold %d ms + standstill" % VALUE_HOLD_MS,
             replay(frames, VALUE_HOLD_MS, True))]

    print("%-34s %10s %10s %10s" % ("rule", "real W", "zeroed", "suspended"))
    print("-" * 68)
    for name, (real, zeroed, susp) in rows:
        tot = real + zeroed + susp
        print("%-34s %6d %3.0f%% %6d %3.0f%% %6d %3.0f%%"
              % (name, real, 100.0*real/tot, zeroed, 100.0*zeroed/tot,
                 susp, 100.0*susp/tot))

    old_real = rows[0][1][0]
    new_real = rows[1][1][0]
    print("\nseconds carrying real rider power: %d -> %d (%+.0f%%)"
          % (old_real, new_real, 100.0*(new_real-old_real)/max(old_real, 1)))
    ok = new_real > old_real
    print("\nRESULT: %s" % ("the new rule recovers data the old one discarded"
                            if ok else "FAILED -- no improvement"))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
