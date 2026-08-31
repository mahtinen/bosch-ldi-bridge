# bosch-ldi-bridge

Turn a Bosch smart system eBike into a **standard Bluetooth power meter**, so any
sports watch or head unit records your rider power and cadence as if you had a
crank-based power meter fitted.

Runs on a single ESP32-C3. No phone app, no watch app, nothing to install on the
watch — it pairs as an ordinary power pod.

```
   Bosch eBike ──BLE──▶ ESP32-C3 ──BLE──▶ watch / head unit
   LiveData Interface     bridge          Cycling Power Service
   (protobuf, custom)                     (0x1818, standard)
```

## Status

Working end to end on the author's hardware: a Performance Line CX Gen5 with a
Purion 200 (BRC3800) control unit, paired to a Suunto Race S.

| Feature | State |
|---|---|
| Cycling Power Service peripheral | works — watch pairs as a Power pod |
| Rider power and cadence in the recorded activity | **verified** |
| Bosch LDI client (bond, encrypt, MTU/DLE verify) | works |
| Protobuf decode of all 13 LDI fields | works, validated against 644 real frames |
| Flash capture log for diagnostics | works |
| Reconnect after link loss, bond persistence | works |

See [Known limitations](#known-limitations) before relying on it.

## Why this exists

Bosch's smart system exposes live rider data over BLE through its **LiveData
Interface** (LDI), published May 2026. It is a custom protobuf service that no
watch understands.

Watches, meanwhile, universally understand the Bluetooth SIG **Cycling Power
Service**. So the useful thing to build is a translator: subscribe to LDI as a
client, re-broadcast as a standard power meter. The watch never learns anything
unusual happened, which means power feeds native training-load and power-zone
features rather than being logged as an inert side channel.

Cadence rides along inside CPS as crank revolution data, so **one pairing
delivers both channels** — no second sensor.

## Hardware

- **ESP32-C3** board (developed on an ESP32-C3 SuperMini, 4 MB flash)
- USB power — a power bank, or a phone (see [Known limitations](#known-limitations))
- A Bosch smart system eBike with software **v19 or newer**
- Any watch or head unit that supports a **BLE** power meter

No wiring, no soldering. The bridge only listens to the bike; it never writes to
it.

> **BLE only, no ANT+.** Many cycling head units pair power meters over ANT+, and
> Garmin devices in particular are ANT+-first. The ESP32-C3 has no ANT+ radio, so
> a device that only speaks ANT+ cannot see this.

## Quick start

Requires [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/get-started/)
**v5.5 or newer**. Developed against v5.5.5.

```bash
git clone https://github.com/<you>/bosch-ldi-bridge
cd bosch-ldi-bridge
idf.py set-target esp32c3
idf.py build
idf.py -p <PORT> flash monitor
```

### 1. Pair the watch

On the watch: **Pair sensor → Power pod**. The bridge advertises as
`eBikePM-01`. Skip any "calibrate power pod" step — there is no strain gauge to
zero, and the bridge refuses the request cleanly.

Watches typically drop the link after pairing and reconnect when you start a
cycling activity. That is normal.

### 2. Pair the bike

Put the bike into accessory pairing mode from the **eBike Flow** app, then over
the serial console:

```
bridge> bike auto on
```

The bridge finds the bike, bonds, and verifies the link. Bonds persist in NVS, so
this is a one-time step — afterwards it reconnects on its own.

### 3. Ride

Start a cycling activity on the watch. Your power and cadence are recorded.

The onboard LED reports state without needing a console:

| LED (2 s cycle) | Meaning |
|---|---|
| solid | booting |
| **1 pulse** | advertising, nothing connected |
| **2 pulses** | watch connected, not yet subscribed |
| **3 pulses** | bike connected and link verified |
| solid + heartbeat blink | both connected — everything working |
| solid + longer dropout | streaming **simulated** data |
| 10 Hz flutter | degraded — connected but MTU/DLE unverified |

Pulse counts rather than blink rates, so a state can be counted rather than
judged.

## How it works

Two BLE roles at once on one radio: a **central** subscribing to the bike, and a
**peripheral** advertising to the watch. NimBLE routes each connection's events
to whichever callback created that link, so the roles never need
demultiplexing.

```
components/
  ldi_client/      BLE central -> bike: scan, bond, verify, subscribe
  ble_link_verify/ ATT MTU and LE Data Length negotiation, and verification
  ldi_proto/       protobuf wire scanner + LDI field mapping
  bike_state/      the 13 decoded fields, with per-field timestamps
  data_source/     chooses what feeds the watch: bike or simulator
  cps_server/      BLE peripheral -> watch: Cycling Power Service 0x1818
  sim_source/      scripted synthetic ride, for testing without a bike
  capture/         append-only flash log of BLE traffic
  status_led/      LED state machine
  bridge/          orchestration, NimBLE host, serial console
```

Three design points that are not obvious:

**The MTU/DLE check is verified, not assumed.** Bosch documents (LDI-003) that
the bike does *not* validate the negotiated ATT MTU or data length and does *not*
disconnect when they are too small — instead the BLE stack silently truncates
notifications to 20 bytes and the protobuf stream arrives corrupted. Worse, no
NimBLE API returns a connection's *negotiated* data length, so
`ble_link_verify` latches `BLE_GAP_EVENT_DATA_LEN_CHG` with a timeout and reports
a timeout as **unverified** rather than as success or failure.

**The decoder merges rather than replaces.** LDI notifications are partial: a
field is omitted when unchanged (spec 2.2.4.3), and which unchanged fields get
included anyway is explicitly not guaranteed (LDI-002). Absence therefore means
*unchanged*, not *zero*.

**Stale data is reported as zero, never held.** The spec warns that when a data
source disappears the bike sends no notification at all — the last value simply
stands, indistinguishable from fresh. Holding it would write a plateau into the
workout that never happened, and the rider cannot notice because watches do not
display the sensor name.

## Diagnostics

The bike is rarely next to a computer, so the bridge records its own sessions to
a dedicated 2.4 MB flash partition and can be read back afterwards. Take it to
the bike on a power bank, ride, bring it back:

```
bridge> capture status      records, bytes used
bridge> capture dump        full replay, hex + ASCII
bridge> capture erase
```

The log is append-only across power cycles, so several trips accumulate rather
than overwrite. It records advertisements, the GATT map, link-quality results,
pairing outcome, every notification payload, and a once-a-minute liveness marker
with uptime, free heap and reset reason.

Host-side tools read it back:

| Tool | Purpose |
|---|---|
| `tools/decode_ldi.py` | decode the protobuf stream and report how each field moved |
| `tools/ride_profile.py` | summarise a capture as a ride profile |
| `tools/uptime.py` | per-session duration and **why each session ended** |
| `tools/test_ldi_decode.py` | replay real frames through the field mapping |
| `tools/test_crank_model.py` | validate the 1/1024 s crank arithmetic, no hardware |
| `tools/test_crank_hw.py` | same, driving a connected board |
| `tools/test_capture_hw.py` | flash log survival across power cycles |
| `tools/test_phase4_hw.py` | source selection behaviour with no bike present |
| `tools/check_idf_compat.py` | check version-sensitive API assumptions against an ESP-IDF tree |

`uptime.py` distinguishes a supply that cut out (`poweron`, `brownout`) from a
firmware fault (`panic`, watchdogs) — a distinction worth having when a board
dies unattended.

## Testing without a bike

A built-in simulator drives the watch side with a scripted 8:30 ride, so the
whole peripheral path can be developed and verified indoors:

```
bridge> source sim
bridge> sim script on          the 8:30 loop
bridge> sim set 200 80         hold 200 W at 80 rpm
bridge> sim stop               tests the zero-cadence freeze
```

The script includes blocks at 150 W/60 rpm and 250 W/90 rpm — values a watch
could not plausibly synthesise, so seeing them in an exported file is proof of
provenance. It also includes a deliberate hard stop, which is where naive
implementations produce phantom cadence or a spike on restart.

`source auto` (the default) uses the bike when its data is fresh and sends
**nothing** otherwise. It never substitutes simulated power, which would write
fabricated data into a real workout unnoticed.

## Protocol notes

[`docs/LDI-PROTOCOL.md`](docs/LDI-PROTOCOL.md) documents what the bike actually
sends: the advertisement, the full GATT map, the field mapping with units, and
the behaviours that are easy to get wrong (partial updates, negative cadence,
sign-extended varints, the truncation failure mode).

Some of it was reverse-engineered from captures before the specification was
consulted, then confirmed against it. Both are noted, because the reverse
engineering is what future users with different firmware will have to repeat.

The specification PDF is **not** included — it is Bosch's to distribute. Get it
free from bosch-ebike.com → Service → Downloads → LiveData and drop it in
`docs/`.

## Known limitations

- **Power source, not firmware.** The bridge draws roughly 60–80 mA. Many power
  banks switch off below ~100 mA, and some phones cut USB accessory power after
  10–20 minutes. Sessions ending this way appear in the log as `poweron` reset
  reasons. A bank with a trickle/low-current mode, or a small dummy load, fixes
  it. Nothing in firmware can convince a bank that 70 mA is a real device.
- **One watch at a time.** The CPS server tracks a single connection, so a watch
  *and* a head unit cannot both read it simultaneously. Fixable by iterating an
  array of subscribers.
- **eBike Flow does not list the bridge as an accessory.** The specification
  wants the accessory to *advertise* with service solicitation and let the bike
  connect; this implementation connects to the bike as a central instead. It
  bonds, encrypts and streams correctly, but Flow never registers it. Cosmetic
  so far.
- **Motor power is deliberately unused.** An undocumented field carries a second
  power-shaped channel, almost certainly motor output. The spec requires clients
  to ignore unrecognised fields, and using it would record motor output as rider
  effort and inflate every training metric.
- **Tested on one bike and one watch.** Different control units may expose
  different UUIDs or fields. The capture log exists precisely so that can be
  investigated.
- **Bosch marks LDI experimental** and provides it as-is. The decoder ignores
  unknown fields and tolerates missing ones, but a future firmware could still
  change behaviour.

## Licence

[MIT](LICENSE).

`proto/ldi.proto` is transcribed from the Bosch LDI specification and carries its
own Apache-2.0 header and Bosch copyright.

Bosch grants a royalty-free right to use the LiveData Interface to build
compatible products, which is what this is. Not affiliated with or endorsed by
Robert Bosch GmbH. "Bosch" and "eBike Flow" are trademarks of their owner.
