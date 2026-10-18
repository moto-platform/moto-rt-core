# Golden vectors for the derived signals (D-070 item 2)

Generated C headers, **do not edit**: moto-server's Python reference of the derived signals
writes them from synthetic scenarios (no GPS, no real ride, D-033):

```bash
cd moto-server && uv run moto-server golden --out ../moto-rt-core/tests/host/vectors
```

- `derived_vectors.h`: row types, flag enum, tolerance type. `derived_<scenario>.h`: the
  DID inputs (`VEHICLE_CL250_IDX_*`, at each DID's poll period), the expected outputs
  (`VEHICLE_CL250_DERIVED_*`, valid or invalid with a flag) and one tolerance per signal
  (defs `resolution` / 2). Scenarios: `idle_warmup`, `accel_gears`, `stale_gap`, `edge_values`.
- Signals: class A with a formula in the reference, `ready_rt_core`, DID inputs only (28).
- Defs version: generated from **moto-vehicle-defs v0.14.0**. The expected outputs use that
  version's derived `stale_after_ms` (the age limit of a signal's DID inputs), so the
  features/derived test that compares against them needs a pin with the same derived table
  (v0.14.0 or later). The indices are the same in v0.13.0, which this repo pins now, so
  `test_derived_vectors` (shape only) builds here.
- Provenance: the banner of each file carries the defs version and a content SHA-256; moto-server
  keeps the same hashes in `tests/golden/MANIFEST.sha256` and fails its CI when the reference
  changes without regenerating. Regenerate and commit here whenever that manifest changes.
- Size about 1.3 MB; host tests only, never linked into the firmware.
