"""
Host-side validation of the crank_model algorithm.

This is a direct transcription of components/cps_server/crank_model.c.
Its job is to prove the 1/1024 s arithmetic, the modulo-65536 wrap and the
zero-cadence freeze BEFORE any of it is trusted on the watch -- exactly the
checks the plan's Stage 1 does by hand in nRF Connect, but automated.

Run:  python tools/test_crank_model.py
"""


class CrankModel:
    def __init__(self, now_us):
        self.t0_us = now_us
        self.prev_us = now_us
        self.phase_rev = 0.0
        self.cum_crank_rev = 0
        self.last_evt_1024 = 0

    def tick(self, cadence_rpm, now_us):
        dt_us = now_us - self.prev_us
        if dt_us <= 0:
            return
        self.prev_us = now_us

        # THE critical line: not pedalling -> nothing advances.
        if cadence_rpm <= 0.5:
            return

        dt_s = dt_us / 1e6
        rev_period_s = 60.0 / cadence_rpm
        self.phase_rev += (cadence_rpm / 60.0) * dt_s

        while self.phase_rev >= 1.0:
            self.phase_rev -= 1.0
            evt_us = now_us - int(self.phase_rev * rev_period_s * 1e6)
            self.cum_crank_rev = (self.cum_crank_rev + 1) & 0xFFFF
            ticks = ((evt_us - self.t0_us) * 1024) // 1000000
            self.last_evt_1024 = ticks & 0xFFFF


def derive_rpm(rev_prev, evt_prev, rev_now, evt_now):
    """What a conforming receiver (the watch) does. Modular UNSIGNED."""
    d_rev = (rev_now - rev_prev) & 0xFFFF
    d_t = (evt_now - evt_prev) & 0xFFFF
    if d_t == 0:
        return None  # no new crank event; hold previous cadence
    return d_rev * 61440 / d_t


def run(cadence_profile, tick_hz=50, notify_hz=1, t_end_s=10.0):
    """Returns the list of (t, cum_rev, evt) a 1 Hz notifier would send."""
    m = CrankModel(0)
    out = []
    tick_us = int(1e6 / tick_hz)
    notify_us = int(1e6 / notify_hz)
    next_notify = notify_us
    t = 0
    while t <= int(t_end_s * 1e6):
        t += tick_us
        m.tick(cadence_profile(t / 1e6), t)
        if t >= next_notify:
            out.append((t / 1e6, m.cum_crank_rev, m.last_evt_1024))
            next_notify += notify_us
    return out


failures = []


def check(name, cond, detail=""):
    if cond:
        print(f"  PASS  {name}")
    else:
        print(f"  FAIL  {name}  {detail}")
        failures.append(name)


print("\n=== 1. Sustained 80 rpm: crank period must be 768 ticks exactly ===")
pkts = run(lambda t: 80.0, t_end_s=10.0)
for t, rev, evt in pkts[:5]:
    print(f"    t={t:4.1f}s  rev={rev:3d}  evt={evt:6d}   "
          f"wire: 20 00 C8 00 {rev & 0xFF:02X} {rev >> 8:02X} "
          f"{evt & 0xFF:02X} {evt >> 8:02X}")
rpms = [derive_rpm(pkts[i][1], pkts[i][2], pkts[i + 1][1], pkts[i + 1][2])
        for i in range(len(pkts) - 1)]
rpms = [r for r in rpms if r is not None]
check("all derived cadences within 0.5 rpm of 80",
      all(abs(r - 80.0) < 0.5 for r in rpms),
      f"got {[round(r, 2) for r in rpms]}")

print("\n=== 2. Awkward cadence 79 rpm: truncation must NOT compound ===")
pkts = run(lambda t: 79.0, t_end_s=120.0)
rpms = [derive_rpm(pkts[i][1], pkts[i][2], pkts[i + 1][1], pkts[i + 1][2])
        for i in range(len(pkts) - 1)]
rpms = [r for r in rpms if r is not None]
# Over 120 s the mean must stay locked to 79, not drift like the
# per-revolution-truncation approach blecsc uses.
mean = sum(rpms) / len(rpms)
print(f"    mean over 120 s = {mean:.3f} rpm   "
      f"min={min(rpms):.2f} max={max(rpms):.2f}")
check("mean cadence within 0.3 rpm of 79 over 120 s",
      abs(mean - 79.0) < 0.3, f"mean={mean:.3f}")

print("\n=== 3. Event-time wrap at exactly 64.000 s ===")
pkts = run(lambda t: 80.0, t_end_s=140.0)
wraps = [i for i in range(1, len(pkts)) if pkts[i][2] < pkts[i - 1][2]]
print(f"    wrapped at packets {wraps} (t = {[pkts[i][0] for i in wraps]} s)")
check("event time wraps (uint16 rollover observed)", len(wraps) >= 2)
rpms_across = []
for i in wraps:
    r = derive_rpm(pkts[i - 1][1], pkts[i - 1][2], pkts[i][1], pkts[i][2])
    rpms_across.append(r)
    print(f"    across wrap: evt {pkts[i-1][2]} -> {pkts[i][2]}  "
          f"=> {r:.2f} rpm")
check("modular subtraction still yields ~80 rpm across the wrap",
      all(r is not None and abs(r - 80.0) < 1.0 for r in rpms_across),
      f"got {rpms_across}")

print("\n=== 4. Hard stop: both counters must FREEZE ===")


def stop_profile(t):
    return 80.0 if t < 30.0 else 0.0


pkts = run(stop_profile, t_end_s=60.0)
after = [(t, rev, evt) for (t, rev, evt) in pkts if t > 31.5]
frozen_rev = len({rev for _, rev, _ in after}) == 1
frozen_evt = len({evt for _, _, evt in after}) == 1
print(f"    after the stop, rev stays {after[0][1]}, evt stays {after[0][2]}")
print(f"    packets still emitted after stop: {len(after)}")
check("cumulative revolutions frozen while stopped", frozen_rev)
check("last crank event time frozen while stopped", frozen_evt)
check("notifications continue during the stop", len(after) > 20)
d = derive_rpm(after[0][1], after[0][2], after[-1][1], after[-1][2])
check("receiver sees d_t == 0 (must hold, not divide)", d is None,
      f"derive_rpm returned {d}")

print("\n=== 5. Restart after stop: NO cadence spike ===")


def restart_profile(t):
    if t < 30.0:
        return 80.0
    if t < 60.0:
        return 0.0
    return 80.0


pkts = run(restart_profile, t_end_s=75.0)
post = [p for p in pkts if p[0] > 60.0]
spikes = []
for i in range(1, len(post)):
    r = derive_rpm(post[i - 1][1], post[i - 1][2], post[i][1], post[i][2])
    if r is not None:
        spikes.append(r)
print(f"    first derived cadences after restart: "
      f"{[round(r, 1) for r in spikes[:6]]}")
check("no derived cadence above 150 rpm after restart",
      all(r < 150.0 for r in spikes),
      f"max={max(spikes) if spikes else 'n/a'}")

print("\n=== 6. Cold start at zero cadence: no phantom data ===")
pkts = run(lambda t: 0.0, t_end_s=20.0)
check("counters stay at zero when never pedalling",
      all(rev == 0 and evt == 0 for _, rev, evt in pkts))

print()
if failures:
    print(f"RESULT: {len(failures)} FAILED -> {failures}")
    raise SystemExit(1)
print("RESULT: all crank_model checks passed")
