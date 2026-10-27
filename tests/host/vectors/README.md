# Golden vectors for the derived signals (D-070 item 2)

Generated C headers, **do not edit**: vehicle-server's Python reference of the derived signals
writes them from synthetic scenarios (no GPS, no real ride, D-033):

```bash
cd vehicle-server && uv run vehicle-server golden --out ../vehicle-rt-core/tests/host/vectors
```

- `derived_vectors.h`: row types, flag enum, tolerance type. `derived_<scenario>.h`: the
  DID inputs (`VEHICLE_CL250_IDX_*`, at each DID's poll period), the expected outputs
  (`VEHICLE_CL250_DERIVED_*`, valid or invalid with a flag) and one tolerance per signal
  (defs `resolution` / 2). Scenarios: `idle_warmup`, `accel_gears`, `stale_gap`, `edge_values`.
- Signals: class A with a formula in the reference, `ready_rt_core`, DID inputs only (28).
- Defs version: generated from **vehicle-defs v0.14.0**. The expected outputs use that
  version's derived `stale_after_ms` (the age limit of a derived input) and, for a non-event
  DID input, that DID's own `stale_after_ms` of the gen vehicle table (D-070), so the
  features/derived test that compares against them needs a pin with the same derived table
  (v0.14.0 or later). This repo pins v0.14.0 since the 0xFD03 PR (E-17 (a)), so the table
  matches; `test_derived_vectors` checks the shape only; `test_derived_core` (features/derived, C3) replays the scenarios through the core and compares every row, and fingerprints the gen table the vectors depend on.
- Provenance: the banner of each file carries the defs version and a content SHA-256; vehicle-server
  keeps the same hashes in `tests/golden/MANIFEST.sha256` and fails its CI when the reference
  changes without regenerating. Regenerate and commit here whenever that manifest changes.
- Size about 1.3 MB; host tests only, never linked into the firmware.
- Class B (C3 part 2a): `derived_vectors_b.h` (ride marker and closed-value row types) and
  `derived_<scenario>_b.h`: the rides table is the epoch-marker table (start_ms opens a stop epoch,
  end_ms ends it and the closed values of that ride are compared), one row per class-B scalar signal
  (12) with its tolerance. The four single-ride scenarios replay the DID inputs of the class-A header
  of the same name; `derived_two_rides_b.h` carries its own inputs (two rides and an ECU-absent gap).
  `test_derived_ride` replays them. No GPS or sensor input.
