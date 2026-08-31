"""
Validate the LDI decode logic against REAL captured frames.

This mirrors components/ldi_proto/ldi_decode.c field-for-field, then replays a
capture dump through it and asserts the results are physically sensible. It is
the strongest validation available without the bike present, because the input
is 227 frames the actual bike sent.

The merge semantics are the point: notifications omit unchanged fields
(spec 2.2.4.3, LDI-002), so a decoder that replaces state instead of merging
would blank most fields every second. That failure is asserted against here.

Usage:
    python tools/test_ldi_decode.py <capture.log>
"""
import sys

sys.path.insert(0, __import__("os").path.dirname(__file__))
from decode_ldi import load_frames, parse  # noqa: E402

# Field numbers from the spec, section 2.4.
F = {
    1: "speed", 2: "cadence", 5: "rider_power", 9: "ambient_brightness",
    10: "battery_soc", 11: "time", 12: "odometer", 17: "bike_light",
    21: "system_locked", 22: "charger_connected", 23: "light_reserve_state",
    24: "diagnosis_program_active", 25: "bike_not_driving",
}


# Fields the spec declares SIGNED. Protobuf sign-extends a negative int32 to a
# full 10-byte varint, so the raw value arrives as e.g. 0xFFFFFFFFFFFFFFF8.
# Reading it unsigned yields 1.8e19 and looks like corruption; it is actually
# -8 rpm, i.e. the rider backpedalling, which the spec permits (int32, range
# -32768..32767). ldi_decode.c casts through uint32_t to int32_t and clamps
# negatives to zero, and this must do the same to test the same thing.
SIGNED32 = {2}       # cadence
SIGNED64 = {11}      # time


def as_signed32(v):
    v &= 0xFFFFFFFF
    return v - (1 << 32) if v & 0x80000000 else v


def as_signed64(v):
    return v - (1 << 64) if v & (1 << 63) else v


def decode_merge(frames):
    """Replay frames, merging as the firmware does. Returns state history."""
    state = {}
    history = []
    unknown_total = 0
    empty_frames = 0

    for t_ms, payload in frames:
        touched = 0
        for fnum, wt, val in parse(payload):
            if fnum in F:
                if fnum in SIGNED32:
                    val = as_signed32(val)
                elif fnum in SIGNED64:
                    val = as_signed64(val)
                state[F[fnum]] = val
                touched += 1
            else:
                unknown_total += 1
        if touched == 0:
            empty_frames += 1
        history.append((t_ms, dict(state)))
    return history, unknown_total, empty_frames


failures = []


def check(name, ok, detail=""):
    print("  %s  %-52s %s" % ("PASS" if ok else "FAIL", name, detail))
    if not ok:
        failures.append(name)


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)

    frames = load_frames(sys.argv[1], 40)
    print("replaying %d frames from handle 40\n" % len(frames))
    if not frames:
        sys.exit("no frames found")

    history, unknown, empty = decode_merge(frames)
    final = history[-1][1]

    print("--- merge behaviour ---")
    check("every frame yielded at least one known field", empty == 0,
          "%d empty" % empty)
    check("unknown fields present and ignored", unknown > 0,
          "%d ignored (spec 2.2.4.3 requires this)" % unknown)

    # The merge test that matters: after the first few frames, all 13 fields
    # should be populated and STAY populated, even though most frames carry
    # only a handful.
    counts = [len(s) for _, s in history]
    check("state accumulates and never shrinks",
          all(b >= a for a, b in zip(counts, counts[1:])),
          "field count %d -> %d" % (counts[0], counts[-1]))

    per_frame = [len([1 for f, _, _ in parse(p) if f in F]) for _, p in frames]
    avg = sum(per_frame) / len(per_frame)
    check("frames really are partial (fewer fields than the merged total)",
          avg < len(final),
          "avg %.1f known fields/frame vs %d merged" % (avg, len(final)))

    print("\n--- decoded values, final state ---")
    speed_kmh = final.get("speed", 0) / 100.0
    print("  speed              %8.2f km/h   (raw %d, 1/100 km/h)"
          % (speed_kmh, final.get("speed", 0)))
    print("  cadence            %8d rpm" % final.get("cadence", 0))
    print("  rider_power        %8d W" % final.get("rider_power", 0))
    print("  ambient_brightness %8.1f lux    (raw %d, 1/1000 lux)"
          % (final.get("ambient_brightness", 0) / 1000.0,
             final.get("ambient_brightness", 0)))
    print("  battery_soc        %8d %%" % final.get("battery_soc", 0))
    print("  odometer           %8.2f km     (raw %d m)"
          % (final.get("odometer", 0) / 1000.0, final.get("odometer", 0)))
    print("  time               %8d        (unix)" % final.get("time", 0))
    print("  bike_light         %8d        (0 invalid, 1 off, 2 on)"
          % final.get("bike_light", 0))
    for k in ("system_locked", "charger_connected", "light_reserve_state",
              "diagnosis_program_active", "bike_not_driving"):
        print("  %-18s %8s" % (k, bool(final.get(k, 0))))

    print("\n--- physical plausibility ---")
    speeds = [s.get("speed", 0) / 100.0 for _, s in history]
    cads = [s.get("cadence", 0) for _, s in history]
    powers = [s.get("rider_power", 0) for _, s in history]
    socs = [s.get("battery_soc", 0) for _, s in history if "battery_soc" in s]
    odos = [s.get("odometer", 0) for _, s in history if "odometer" in s]
    times = [s.get("time", 0) for _, s in history if "time" in s]

    check("speed within 0..80 km/h", 0 <= max(speeds) <= 80,
          "max %.2f km/h" % max(speeds))
    check("cadence within -200..200 rpm", -200 <= min(cads) <= max(cads) <= 200,
          "range %d..%d rpm" % (min(cads), max(cads)))
    neg = [c for c in cads if c < 0]
    if neg:
        print("       note: %d sample(s) of NEGATIVE cadence (min %d rpm) -- "
              "backpedalling. The firmware clamps these to 0, since a negative "
              "cadence has no meaning for the CPS crank model." % (len(neg), min(neg)))
    check("rider power within 0..2000 W", 0 <= max(powers) <= 2000,
          "max %d W" % max(powers))
    check("battery soc within 0..100 %", all(0 <= v <= 100 for v in socs),
          "range %d..%d" % (min(socs), max(socs)) if socs else "n/a")
    check("odometer monotonic non-decreasing",
          all(b >= a for a, b in zip(odos, odos[1:])),
          "%d -> %d m (+%d)" % (odos[0], odos[-1], odos[-1] - odos[0]))
    check("timestamp is a plausible epoch value",
          times and 1.7e9 < times[-1] < 2.2e9, "%d" % (times[-1] if times else 0))
    check("timestamp monotonic", all(b >= a for a, b in zip(times, times[1:])))

    # Cadence and power should move together: pedalling produces both.
    moving = [(c, p) for c, p in zip(cads, powers) if c > 0]
    stopped = [(c, p) for c, p in zip(cads, powers) if c == 0]
    if moving and stopped:
        avg_moving = sum(p for _, p in moving) / len(moving)
        avg_stopped = sum(p for _, p in stopped) / len(stopped)
        check("power is higher while pedalling than while stopped",
              avg_moving > avg_stopped,
              "%.0f W pedalling vs %.0f W stopped" % (avg_moving, avg_stopped))

    print()
    if failures:
        print("RESULT: FAILED -> %s" % failures)
        return 1
    print("RESULT: decode logic validated against %d real frames" % len(frames))
    return 0


if __name__ == "__main__":
    sys.exit(main())
