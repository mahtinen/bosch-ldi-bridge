# Contributing

Bug reports and patches welcome. This has been tested on exactly one bike and
one watch, so reports from other hardware are especially useful.

## If it does not work with your bike

The most valuable thing you can send is a capture log:

```
bridge> capture erase
   ... take it to the bike, let it connect, ride a little, come back ...
bridge> capture dump
```

Then:

```
python tools/decode_ldi.py <saved-dump>     # how each field moved
python tools/uptime.py     <saved-dump>     # session durations and why they ended
```

The log contains BLE addresses of nearby devices and your bike's odometer
reading. Redact anything you would rather not publish before attaching it.

Useful things to include:

- bike model, drive unit and control unit software versions (eBike Flow shows
  these)
- what `bike gatt` printed — if the UUIDs differ from `docs/LDI-PROTOCOL.md`,
  that is the interesting part
- the `LINK` record, which shows whether MTU/DLE negotiated

## If it does not work with your watch

Check `docs/LDI-PROTOCOL.md` is not the problem first: run `source sim`, then
`sim set 200 80`, and see whether the watch shows 200 W. If the simulator works
and the bike does not, the bug is on the bike side and vice versa. That one step
saves most of the guesswork.

## Code

- C for firmware, targeting ESP-IDF v5.5+. Match the surrounding style.
- Python for host tools, standard library plus `pyserial` only.
- The host-side tests in `tools/test_*.py` run without hardware where possible;
  please keep it that way for anything that can be tested that way.
- Comments should explain *why*, especially where the code is deliberately not
  the obvious thing. Several places in this codebase look wrong until you know
  which documented Bosch or NimBLE defect they are working around, and those are
  commented at the site.

## Scope

The bridge is **read-only** toward the bike. It does not write to the control
channel, change assist modes, or attempt anything the LiveData Interface does not
document as a client operation. Patches that change that will not be merged.
