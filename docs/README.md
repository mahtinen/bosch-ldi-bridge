# docs/

- [`LDI-PROTOCOL.md`](LDI-PROTOCOL.md) — what a Bosch smart system eBike actually
  sends over its LiveData Interface: advertisement, GATT map, field mapping with
  units, and the behaviours that are easy to get wrong.

## The specification PDF

Not included — it is Bosch's to distribute, not ours.

Get it free (no registration, no API key) from:

> bosch-ebike.com → Service → Downloads → **LiveData**

Direct HTTP fetch is bot-blocked, so download it in a browser and drop it in
this directory. `.gitignore` excludes `docs/*.pdf` so it will not be committed.

Nothing in the firmware needs the PDF to build or run — the field mapping is
already implemented in `components/ldi_proto/` and the schema is transcribed in
`proto/ldi.proto`. You want it to check units, to read the three documented
defects (LDI-001/2/3), or if your bike behaves differently from
`LDI-PROTOCOL.md`.

Bosch grants a non-exclusive, royalty-free right to use the interface to build
compatible products. Modifying the interface itself would require written
consent; building a client does not.
