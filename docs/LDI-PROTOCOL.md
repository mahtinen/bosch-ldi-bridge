# Bosch LiveData Interface — as observed on the wire

What a Bosch smart system eBike actually sends, captured from a **Performance
Line CX Gen5 with a Purion 200 (BRC3800)** control unit and cross-checked
against the LiveData Interface specification v1.0 (2026-05-01).

The specification is free from bosch-ebike.com → Service → Downloads → LiveData,
but is not redistributable, so it is not included here. This document records
what was measured, which is what you need if your bike behaves differently.

Everything below was first reverse-engineered from captures and *then* confirmed
against the spec. Both are noted, because the reverse engineering is what anyone
with different firmware will have to repeat — and the tooling for it ships in
`tools/`.

---

## Advertisement

While in accessory pairing mode the bike advertises:

```
<public address>          type=0 (public), 27 bytes
02 01 05                              Flags = 0x05
03 02 02 fe                           16-bit svc UUID 0xFE02 (INCOMPLETE list)
13 09 "smart system eBike"            Complete Local Name
```

- **`0xFE02`** is a SIG-allocated **16-bit member** UUID — *not* a 128-bit
  custom service. This trips people up: the LDI service itself is 128-bit, but
  what the bike *advertises* is this 16-bit member UUID. Filtering scans on
  "has a 128-bit service UUID" finds nothing.
- **`Flags = 0x05`** is LE **Limited** Discoverable (plus BR/EDR unsupported).
  The bike only advertises this way while in pairing mode, so it doubles as a
  reliable "pairing mode is active" signal.
- AD type `0x02` is the **incomplete** 16-bit list — and indeed every real
  service is 128-bit and unadvertised.

The public address OUI observed was `00:04:63`, useful as weak corroboration
but not something to depend on across hardware revisions.

## GATT map

All Bosch services share the base `0000xxxx-eaa2-11e9-81b4-2a2ae2dbcce4`,
abbreviated below as `ebike_uuid(xxxx)`.

| Service | Characteristic | Props | Role |
|---|---|---|---|
| `00000010` | `00000011` | `N` | control channel, notify side |
| `00000010` | `00000012` | `w` | control channel, write side |
| **`0000eb20`** | **`0000eb21`** | **`R N`** | **LiveData — the protobuf stream** |
| `0000eb40` | `0000eb41` | `R N` | |
| `0000eb10` | `0000eb11` | `N` | |
| `0000ebd0` | `0000ebd1` | `w` | |
| `00000040` | `00000041` | `R` | |
| `00000040` | `00000042` | `R N w` | |
| `0000eba0` | `0000eba1` | `R N` | |
| `0000eba0` | `0000eba2` | `w` | |
| `00000020` | `00000021` | `N w` | |
| `0000eb60` | `0000eb61` | `N w` | |

Plus standard `0x1800` GAP, `0x1801` GATT and `0x180A` DIS.

**Only `ebike_uuid(eb21)` is LiveData.** The spec defines the service as
single-characteristic; everything else belongs to other Bosch protocols. On
connect, `00000011` sends an unrelated 65-byte frame that is *not* protobuf —
feeding it to a LiveData decoder produces garbage.

Match on **UUID, not attribute handle**: handles are assigned by the server and
are not durable across firmware revisions.

Security: `eb21` requires **encryption**. Pairing is Just Works / LE Secure
Connections, and the bond persists — reconnection needs no pairing mode.

## Link requirements — and the failure mode that matters

The spec requires ATT MTU ≥ 247 and LE Data Length ≥ 251. Three documented
defects make this the single most important thing to get right:

| ID | Issue |
|---|---|
| **LDI-001** | The bike does **not** initiate the Data Length Update procedure, despite the profile requiring it. The accessory must. |
| **LDI-002** | Notifications include unchanged fields unpredictably; clients must not rely on their presence. |
| **LDI-003** | The bike does **not** validate MTU or data length and does **not** disconnect when they are too small. |

LDI-003 is the dangerous one. If the ATT MTU stays at the 23-byte default, the
BLE stack **silently truncates notifications to 20 bytes** and the protobuf
stream arrives corrupted, with no error anywhere. It looks exactly like a
protocol you have failed to reverse-engineer.

Making it worse, **no NimBLE API returns a connection's negotiated data
length**. `ble_gap_read_sugg_def_data_len()` returns the host's suggested
default for *new* connections and will happily report 251 while the current link
sits at 27. The only read-back is latching `BLE_GAP_EVENT_DATA_LEN_CHG`.

So `components/ble_link_verify/` requests both, latches that event with a
timeout, and reports a timeout as **unverified** — deliberately not the same
claim as "failed". It also flags the empirical tell: payloads arriving at
*exactly* 20 bytes, repeatedly, mean DLE did not take.

Measured on working hardware:

```
mtu=247  ll_rx=251  ll_tx=251  dle_seen=1  enc=1
```

RX matters more than TX — the bike's notifications land on the accessory's
receive path, so a wide TX with a narrow RX still truncates.

## The LiveData message

Protobuf (proto3) on `eb21`, roughly 1.3 Hz, 51–80 bytes. Field numbers and
units from spec section 2.4; the full schema is transcribed in
[`../proto/ldi.proto`](../proto/ldi.proto).

| Field | Name | Type | Unit |
|---|---|---|---|
| 1 | `speed` | uint32 | **1/100 km/h** |
| 2 | `cadence` | **int32** | 1 rpm |
| 5 | `rider_power` | uint32 | 1 W |
| 9 | `ambient_brightness` | uint32 | **1/1000 lux** |
| 10 | `battery_soc` | uint32 | 1 % |
| 11 | `time` | int64 | seconds since epoch (UTC) |
| 12 | `odometer` | uint32 | 1 m |
| 17 | `bike_light` | enum | 0 invalid, 1 off, 2 on |
| 21 | `system_locked` | bool | |
| 22 | `charger_connected` | bool | |
| 23 | `light_reserve_state` | bool | |
| 24 | `diagnosis_program_active` | bool | |
| 25 | `bike_not_driving` | bool | standstill |

Thirteen fields, matching Bosch's published list. Motor power and assist mode
are **not** exposed.

### Notifications are partial — merge, never replace

A field is omitted when unchanged (spec 2.2.4.3), and LDI-002 records that which
unchanged fields get included anyway is not guaranteed. The first frame after
subscribing is a full snapshot; after that only changes arrive.

**A field absent from a frame means "unchanged", not "zero".** A decoder that
replaces state per frame blanks most of it every second. Measured: about 8 of 13
fields per frame.

### Staleness has no indicator

Spec 2.2.3.2 warns that when a data source disappears — a battery disconnected,
say — the bike sends **no notification and no indication**. The last value simply
stands, stale and indistinguishable from fresh.

This implementation timestamps every field individually and reports 0 W once a
field goes stale rather than holding it, because holding would write a plateau
into the workout that never happened.

### Cadence can be negative, and is sign-extended

Backpedalling produces **negative cadence** — observed down to −19 rpm. `cadence`
is `int32`, and protobuf sign-extends a negative int32 to a full 10-byte varint,
so −8 rpm arrives on the wire as:

```
0xFFFFFFFFFFFFFFF8
```

Read as unsigned that is 1.8×10¹⁹ and looks exactly like stream corruption. Cast
through `uint32_t` to `int32_t`. This implementation then clamps negatives to
zero, because the CPS crank model treats cadence ≤ 0.5 rpm as "not pedalling"
and freezes both counters — which is what a real power meter does when the cranks
are not turning forwards.

### Undocumented fields carry real data

Fields 7, 13, 14, 15, 16, 18, 19, 20 and 27 appear on the wire but are **absent
from the specification**. Field 7 in particular carries a second power-shaped
channel that correlates with `rider_power` at a ratio varying from 1.16 to 1.67 —
almost certainly motor power.

Spec 2.2.4.3 requires clients to **ignore unrecognised fields**, and this
implementation does. Using field 7 as rider power would record motor output as
the rider's effort and inflate every downstream training metric.

Note this contradicts a claim circulating in earlier community work, that
undocumented LDI fields carry no data. On this firmware they plainly do.

## Reproducing the analysis

`tools/decode_ldi.py` merges partial updates from a capture dump and reports how
each field moved across a session, separating rider inputs (rise while pedalling,
return to zero) from clocks and counters (monotonic) — enough to identify the
field map without the spec.

`tools/ride_profile.py` then summarises a capture as a ride profile, which is the
strongest validation available: a wrong field mapping can satisfy every range
check, but it cannot reproduce the shape of a ride it was not told about.

From a 157-second test ride described only as *"first slow and steady and then
more power until stop"*:

```
   t(s)   power W   cadence   speed km/h
      0         0         0          0.0     stationary
     45         4         8         10.6     slow and steady
     60         6        38         18.4
     75        29        53         19.0
     90        82        30         10.7
    105       445        71         27.2     more power
    120        17        29         15.8
    150         0         0          0.0     stop

   first third avg 28 W  ->  last third avg 293 W
```

Internal consistency check: the 445 W burst occurs at **27.2 km/h**, above the
25 km/h assist cutoff, where the motor stops assisting and the rider supplies all
of it.

## Open questions

- **The control channel** (`00000010`) sends a 65-byte non-protobuf frame at
  connect and has a write-no-response characteristic alongside. Its purpose is
  undocumented. It is plausibly where eBike Flow's accessory registration
  handshake lives — this implementation never replies, and Flow never lists the
  bridge as an accessory, though data flows regardless.
- **The seven other services** are unexplored.
- **Connection direction.** The spec has the accessory *advertise* with service
  solicitation for `ebike_uuid(eb20)` while acting as the GATT *client*, with the
  bike initiating the connection. This implementation does the reverse and
  connects as a central. It works — bonded, encrypted, streaming — but is not the
  specified flow, and is the likeliest reason Flow does not register it.


---

# The watch link, observed

Not part of the Bosch protocol, but recorded here because it caused the most
confusing failure in this project and the numbers are not what you would guess.

## A watch may impose a very short connection interval

```
watch connect itvl=8 latency=0 timeout=500 enc=0 bonded=0
```

`itvl=8` is **10 ms** — 100 connection events per second — imposed by the watch
despite the bridge requesting 30–50 ms and publishing PPCP saying the same. The
peripheral only gets to ask.

That rate shares one radio with the bike link. Combined with per-notification
flash writes it is a plausible source of the supervision timeouts below, which
is why payload capture is now rate-limited.

`enc=0 bonded=0`: at least one watch pairs as a "power pod" without any SMP
bonding at all. Not requiring encryption on the CPS characteristics is therefore
load-bearing, not merely permissive — requiring it would have made this watch
fail with a confusing symptom. It also means CCCDs are never persisted, so
subscribes always arrive as `reason=1` (write) rather than `reason=3` (restore).

## Disconnect reasons are NimBLE-encoded

NimBLE reports HCI reasons offset by `0x200`:

| Logged | HCI | Meaning |
|---|---|---|
| `0x208` | 0x08 | supervision timeout — RF, or a stalled host |
| `0x213` | 0x13 | remote user terminated — the watch hung up deliberately |
| `0x216` | 0x16 | local host terminated |
| `0x23e` | 0x3E | failed to establish |

Both `0x208` and `0x213` were observed within five minutes of each other, so
treating any single disconnect as diagnostic is a mistake — the reason is what
separates "we broke" from "the watch chose to leave".

## Pausing an activity drops the sensor

Pausing on the watch disconnects it. It normally reconnects on resume, observed
taking about **13 seconds**.

One 48-minute ride recorded only 24 minutes: the watch dropped at 33 minutes and
never returned, while the bike streamed on for another 47. Advertising is
restarted from the disconnect handler, and a watchdog now also restarts it if it
is ever found off while disconnected — but in later successful tests the
watchdog never had to fire, so that failure has **not** been reproduced and its
cause is not established.

If you pause, glance at the LED: three pulses means only the bike is connected.
