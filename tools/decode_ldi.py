"""
Decode the LDI protobuf stream out of a capture dump and find which fields
carry which quantity.

The bike sends PARTIAL updates: the first frame after subscribing is a full
snapshot, then only changed fields follow. So this maintains merged state and
reports, per field, how much it varied across the session. A field that moves
while pedalling and returns to zero when you stop is a power or cadence
candidate; one that only ever increments is a counter or clock.

Usage:
    python tools/decode_ldi.py capture.log [--handle 40]
"""
import re
import sys
from collections import OrderedDict

WIRE_VARINT, WIRE_64, WIRE_LEN, WIRE_32 = 0, 1, 2, 5


def read_varint(buf, i):
    shift = 0
    val = 0
    while i < len(buf):
        b = buf[i]
        i += 1
        val |= (b & 0x7F) << shift
        if not (b & 0x80):
            return val, i
        shift += 7
        if shift > 63:
            break
    return None, i


def parse(buf):
    """Yield (field_number, wire_type, value) for one protobuf message."""
    i = 0
    out = []
    while i < len(buf):
        tag, i = read_varint(buf, i)
        if tag is None:
            break
        fnum, wt = tag >> 3, tag & 7
        if fnum == 0:
            break
        if wt == WIRE_VARINT:
            v, i = read_varint(buf, i)
            if v is None:
                break
            out.append((fnum, wt, v))
        elif wt == WIRE_64:
            if i + 8 > len(buf):
                break
            out.append((fnum, wt, int.from_bytes(buf[i:i + 8], "little")))
            i += 8
        elif wt == WIRE_32:
            if i + 4 > len(buf):
                break
            out.append((fnum, wt, int.from_bytes(buf[i:i + 4], "little")))
            i += 4
        elif wt == WIRE_LEN:
            n, i = read_varint(buf, i)
            if n is None or i + n > len(buf):
                break
            out.append((fnum, wt, bytes(buf[i:i + n])))
            i += n
        else:
            break
    return out


def load_frames(path, want_handle, with_skipped=False):
    """Pull (t_ms, payload) for RX records on the given attribute handle.

    With with_skipped=True, yields (t_ms, payload, skipped) instead, where
    skipped is how many notifications the capture's sampling interval dropped
    before this one. That count is what makes the TRUE notification rate
    recoverable: the log stores roughly one frame a second out of about five,
    so field-presence percentages read off the frames alone understate how
    often the bike actually sends -- which is exactly the arithmetic needed to
    tell an on-change field apart from a dead link.
    """
    raw = open(path, encoding="utf-8", errors="replace").read()
    # The serial path emits CR CR LF, which splitlines() turns into a
    # blank line between every record. Drop them so hexdump lines stay
    # adjacent to the header they belong to.
    text = [l for l in raw.splitlines() if l.strip()]
    frames = []
    i = 0
    while i < len(text):
        m = re.match(r"^(\d+)\s+RX\s+handle=(\d+) len=(\d+)"
                     r"(?: skipped=(\d+))?", text[i].rstrip())
        if not m:
            i += 1
            continue
        t_ms, handle = int(m.group(1)), int(m.group(2))
        # Absent in logs written before the count moved into the record
        # header; treat those as unknown rather than as zero skips.
        skipped = int(m.group(4)) if m.group(4) is not None else None
        data = bytearray()
        j = i + 1
        while j < len(text):
            hm = re.match(r"^\s+[0-9a-f]{4}\s+((?:[0-9a-f]{2} )+)", text[j])
            if not hm:
                break
            data += bytes(int(x, 16) for x in hm.group(1).split())
            j += 1
        if handle == want_handle and data:
            frames.append((t_ms, bytes(data), skipped) if with_skipped
                          else (t_ms, bytes(data)))
        i = j if j > i else i + 1
    return frames


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    path = sys.argv[1]
    handle = 40
    if "--handle" in sys.argv:
        handle = int(sys.argv[sys.argv.index("--handle") + 1])

    sampled = load_frames(path, handle, with_skipped=True)
    frames = [(t, p) for t, p, _ in sampled]
    print("frames on handle %d: %d" % (handle, len(frames)))
    if not frames:
        sys.exit("no frames found")

    span = (frames[-1][0] - frames[0][0]) / 1000.0
    print("time span: %.1f s  (captured %.2f Hz)"
          % (span, len(frames) / span if span else 0))

    # The captured frames are a sample, so quote the real rate alongside it.
    # Without this the "seen" column below reads as a rate when it is only a
    # sampling fraction -- the mistake that makes an on-change field look like
    # a broken one.
    skips = [s for _, _, s in sampled]
    total = None
    if span and all(s is not None for s in skips) and any(skips):
        total = len(frames) + sum(skips)
        print("true notification rate: %.2f Hz  (%d sent, %d captured, "
              "%d dropped by sampling)"
              % (total / span, total, len(frames), sum(skips)))
    elif not span:
        pass
    else:
        # Either the log predates the counter, or it is a mix of old and new
        # records, or every payload really was captured. Refusing to guess is
        # the point: quoting the sampled rate as if it were the real one is how
        # a field sent every 3.7 s reads as a field sent every 21 s.
        print("true notification rate: unknown -- no per-record skip counts "
              "(pre-2026-09 log, or CAPTURE_PAYLOAD_MS=0)")

    # Merge partial updates, and record each field's history.
    state = {}
    hist = OrderedDict()
    for t_ms, payload in frames:
        for fnum, wt, val in parse(payload):
            state[fnum] = val
            hist.setdefault(fnum, []).append((t_ms, val))

    print("\n%-6s %-6s %-7s %12s %12s %10s  %s"
          % ("field", "wire", "seen", "min", "max", "distinct", "note"))
    print("-" * 88)

    for fnum in sorted(hist):
        series = hist[fnum]
        vals = [v for _, v in series]
        wt = [w for f, w, _ in parse(frames[0][1]) if f == fnum]
        wire = {0: "varint", 1: "64bit", 2: "bytes", 5: "32bit"}.get(
            wt[0] if wt else 0, "?")

        if isinstance(vals[0], bytes):
            print("%-6d %-6s %-7d %12s %12s %10d  %r"
                  % (fnum, "bytes", len(vals), "-", "-",
                     len(set(vals)), vals[0][:16]))
            continue

        lo, hi = min(vals), max(vals)
        distinct = len(set(vals))
        note = ""

        # Monotonic non-decreasing with a wide range: a clock or a counter.
        if all(b >= a for a, b in zip(vals, vals[1:])) and distinct > 2:
            if 1.7e9 < lo < 2.1e9:
                note = "UNIX TIMESTAMP"
            else:
                note = "monotonic counter"
        # Returns to its low value after moving: rider input.
        elif distinct > 3 and lo == 0 and hi > 0:
            note = "VARIES, returns to 0  <- power/cadence candidate"
        elif distinct > 3:
            note = "varies"
        elif distinct == 1:
            note = "constant"

        # How often the bike actually SENDS this field, scaled back up through
        # the sampling ratio. A field carried in every frame and one carried in
        # 5% of them both merely count as "seen", and only this number
        # separates "the link died" from "the value did not change" -- the
        # distinction the 2026-09-12 ride turned on.
        if total:
            every_s = span * len(frames) / float(len(series) * total)
            note = "~every %.1fs, %s" % (every_s, note)

        print("%-6d %-6s %-7d %12d %12d %10d  %s"
              % (fnum, wire, len(vals), lo, hi, distinct, note))

    # Show the shape of the movement for the interesting fields.
    print("\n--- candidates over time (first 40 samples each) ---")
    for fnum in sorted(hist):
        vals = [v for _, v in hist[fnum]]
        if isinstance(vals[0], bytes) or len(set(vals)) <= 3:
            continue
        if all(b >= a for a, b in zip(vals, vals[1:])):
            continue  # counters are not what we are hunting
        print("field %-4d %s" % (fnum, [v for v in vals[:40]]))


if __name__ == "__main__":
    main()
