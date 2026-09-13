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
| Cycling Speed and Cadence Service | built, crank-only; not advertised by default and not needed |
| Rider power and cadence in the recorded activity | **verified** |
| Bosch LDI client (bond, encrypt, MTU/DLE verify) | works |
| Protobuf decode of all 13 LDI fields | works, validated against 644 real frames |
| Flash capture log for diagnostics | works |
| Accessory role — bike connects to us, per spec | **verified** — the bike registers it |
| Coexisting with the eBike Flow app | works — Flow stays connected while the bridge streams |

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
delivers both channels** — no second sensor, and one power pod covers every
sport mode. A Cycling Speed and Cadence Service (`0x1816`) is also implemented
and left in the GATT database, but it is not advertised and is not needed; see
[Known limitations](#known-limitations) for the sport-mode behaviour that
briefly made it look necessary.

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

Open the **eBike Flow** app and add the bridge from its accessory menu. Nothing
is needed on the serial console — the bridge advertises the Live Data Service as
a *solicitation* ("connect to me, I want this") and the bike, which is the GAP
central in this profile, connects to it.

That direction matters. Connecting to the bike as a central instead — which this
project did originally — takes the bike's single peripheral slot, the one the
Flow app uses, and the two then lock each other out. Registering as a proper
accessory puts the bridge somewhere else entirely, so **you can ride with your
phone and the Flow app running**. See
[Connection direction](docs/LDI-PROTOCOL.md) for the measurements.

The bike searches on its own schedule, so the bridge does not always show up on
the first scan — search again if it does not. Once it appears, Flow lists it by
name and MAC and shows its firmware version, which is the running build's
`<version>+<hash>`, so you can confirm what the bike is actually talking to.

Bonds persist in NVS, so this is a one-time step. Reconnection afterwards is the
bike's job: switch it on and it comes looking.

### 3. Ride

Start a cycling activity on the watch. Your power and cadence are recorded.

The onboard LED reports state without needing a console:

| LED (2 s cycle) | Meaning |
|---|---|
| solid | booting |
| **1 pulse** | advertising, nothing connected |
| **2 pulses** | watch connected, **but nothing is being sent to it** |
| **3 pulses** | bike connected and link verified |
| solid + heartbeat blink | both connected — everything working |
| solid + longer dropout | streaming **simulated** data |
| 10 Hz flutter | degraded — connected but MTU/DLE unverified |

Pulse counts rather than blink rates, so a state can be counted rather than
judged.

**Two pulses is the one to notice mid-ride.** It means the watch is attached and
the bridge has no bike data to give it, so nothing is being recorded. It covers
both "not subscribed yet" and "the bike went away".

## How it works

One BLE role, two peers. The bridge is a **peripheral** to both: it advertises
the Cycling Power service for the watch and the Live Data Service *as a
solicitation* for the bike, in a single 25-byte advertisement, and waits. The
watch connects because it wants a power meter; the bike connects because the
profile makes it the central and the bridge is asking to be its accessory.

Because both arrive through the same advertisement, the two links have to be
told apart: on each new connection the bridge looks for the Live Data Service on
the peer. Found means bike, and it becomes a GATT **client** on a connection it
did not initiate — which is exactly what the profile specifies.

```
components/
  ldi_client/      bike link: identify, bond, verify, subscribe, decode
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

**Three questions, not one, about whether a value is usable.** "Is the bike
there?" is answered by the age of a field the bike sends in *every* frame
(odometer, standstill). "Is the rider stopped?" is answered by the standstill
flag, which the bike states outright. Only "is this value current?" is a
timeout — and it is sized *above* the bike's own update spacing for power and
cadence, which is 3.0–4.8 s. A single 3 s window for all three is what wrote
16,499 invented zeros into a 5 h 53 activity. When the bike is absent entirely
the bridge stops notifying, so the watch records a gap rather than a lie.

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

Never mid-ride, though: when less than 768 KB is free **at boot**, the partition
is reclaimed and the new session's `BOOT` marker says so. A ride costs roughly 76
bytes per second of riding, so 2.4 MB holds two ordinary rides, and reclaiming at
boot keeps the erase away from the one moment the append-only layout exists to
survive. Before that rule existed the partition filled and then stayed full for a
week — silently, while `capture status` still reported `ready: yes`.

Host-side tools read it back:

| Tool | Purpose |
|---|---|
| `tools/decode_ldi.py` | decode the protobuf stream and report how each field moved |
| `tools/ride_profile.py` | summarise a capture as a ride profile |
| `tools/uptime.py` | per-session duration and **why each session ended** |
| `tools/test_ldi_decode.py` | replay real frames through the field mapping |
| `tools/test_staleness.py` | replay a capture through the old and new power/cadence gating |
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
- **Pausing the activity on the watch may end the recording.** On at least one
  Suunto, pausing dropped the sensor link and resuming never re-acquired it —
  the bike kept streaming to the bridge for another 47 minutes while nothing
  reached the watch. The bridge now runs an advertising watchdog so it stays
  discoverable, and logs the watch's disconnect reason and subscribe reason so
  the cause can be identified. If you pause, glance at the LED afterwards: three
  pulses means only the bike is connected.
- **A sport mode connects a sensor only if one of its screens displays that
  sensor's data.** On a Suunto Race S the pod attaches in eMTB and MTB but not
  in Cycling or eBiking — and the cause is not sensor types, service UUIDs or
  appearance values, all of which we chased. Those modes simply had no screen
  field showing power or cadence, so there was nothing to connect a power pod
  *for*. **Add a power or cadence field to a screen** — a custom sport mode
  based on normal biking is enough — and the pod attaches.

  Worth knowing because the symptom is badly misleading: the capture log shows
  the watch connecting and subscribing in the failing mode, while the watch's
  own Connected pods list stays empty. One power pod covers every sport mode
  once the fields are there.
- **Discovery from the Flow app can take several attempts.** The bike scans on
  its own schedule, so the bridge does not always appear on the first search.
  Searching again, or power-cycling the bike, finds it. Once registered this
  does not recur — reconnection is automatic and does not involve the app.
- **A missing bike records a gap, not zeros.** When no LDI frame has arrived for
  8 s the bridge stops notifying rather than sending 0 W, so the watch records
  nothing instead of recording invented zeros. Every CP Measurement carries an
  Instantaneous Power field, so there is no way to transmit "no data" — silence
  is the only honest encoding. A 5 h 53 ride wrote 16,499 of those zeros into its
  activity, indistinguishable from freewheeling. The link stays up throughout (an
  idle ATT link does not disconnect), so the watch keeps the sensor and simply
  shows a hole.
- **One watch at a time.** The controller is built for two connection slots and
  the bike holds one, so a watch *and* a head unit cannot both read the bridge.
  Raising `BT_NIMBLE_MAX_CONNECTIONS` would lift it; nothing else assumes two.
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
