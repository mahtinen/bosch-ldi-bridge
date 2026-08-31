"""
Summarise a decoded LDI capture as a ride profile.

The point is semantic validation rather than plausibility: if the decode is
correct, the profile printed here should match what the rider actually did. A
field mapping can pass every range check and still be wrong; matching the
described shape of the ride is much harder to fake.

Usage:
    python tools/ride_profile.py <capture.log> [--bucket 10]
"""
import sys

sys.path.insert(0, __import__("os").path.dirname(__file__))
from decode_ldi import load_frames, parse  # noqa: E402
from test_ldi_decode import F, SIGNED32, SIGNED64, as_signed32, as_signed64  # noqa: E402


def series(path):
    frames = load_frames(path, 40)
    state = {}
    out = []
    for t_ms, payload in frames:
        for fnum, wt, val in parse(payload):
            if fnum in F:
                if fnum in SIGNED32:
                    val = as_signed32(val)
                elif fnum in SIGNED64:
                    val = as_signed64(val)
                state[F[fnum]] = val
        out.append((t_ms, dict(state)))
    return out


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    path = sys.argv[1]
    bucket = 10
    if "--bucket" in sys.argv:
        bucket = int(sys.argv[sys.argv.index("--bucket") + 1])

    hist = series(path)
    if not hist:
        sys.exit("no frames")

    t0 = hist[0][0]
    print("ride profile, %d-second buckets (%d frames, %.0f s)\n"
          % (bucket, len(hist), (hist[-1][0] - t0) / 1000.0))
    print("   t(s)   power W   cadence   speed km/h   bar")
    print("   " + "-" * 66)

    buckets = {}
    for t_ms, st in hist:
        b = int((t_ms - t0) / 1000 // bucket)
        buckets.setdefault(b, []).append(st)

    peak = 1
    for vals in buckets.values():
        for st in vals:
            peak = max(peak, st.get("rider_power", 0))

    for b in sorted(buckets):
        vals = buckets[b]
        n = len(vals)
        p = sum(v.get("rider_power", 0) for v in vals) / n
        c = sum(max(0, v.get("cadence", 0)) for v in vals) / n
        sp = sum(v.get("speed", 0) for v in vals) / n / 100.0
        bar = "#" * int(round(p / peak * 34))
        print("   %5d   %7.0f   %7.0f   %10.1f   %s"
              % (b * bucket, p, c, sp, bar))

    # Overall shape, described in words so it can be compared with the ride.
    powers = [st.get("rider_power", 0) for _, st in hist]
    cads = [max(0, st.get("cadence", 0)) for _, st in hist]
    pedalling = [p for p, c in zip(powers, cads) if c > 0]

    print("\nsummary")
    print("   pedalling for %d of %d samples (%.0f%%)"
          % (len(pedalling), len(powers), 100.0 * len(pedalling) / len(powers)))
    if pedalling:
        print("   power while pedalling: avg %.0f W, peak %d W"
              % (sum(pedalling) / len(pedalling), max(pedalling)))
    print("   cadence while pedalling: avg %.0f rpm, peak %d rpm"
          % (sum(c for c in cads if c > 0) / max(1, len([c for c in cads if c > 0])),
             max(cads)))

    # First third vs last third, to show whether effort rose.
    third = len(powers) // 3
    if third:
        early = [p for p, c in zip(powers[:third], cads[:third]) if c > 0]
        late = [p for p, c in zip(powers[-third:], cads[-third:]) if c > 0]
        if early and late:
            print("   first third avg %.0f W  ->  last third avg %.0f W"
                  % (sum(early) / len(early), sum(late) / len(late)))

    tail = powers[-8:]
    print("   final samples: %s" % tail)
    print("   ends stopped: %s" % all(p == 0 for p in tail[-4:]))


if __name__ == "__main__":
    main()
