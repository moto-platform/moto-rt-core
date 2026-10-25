# features/derived

Derived signals of D-070 (catalog section 5), computed on rt-core. **Part 1 (this PR): class A signals whose inputs are DIDs or other computed derived signals.** Everything else (class B ride summaries, signals with sensor/IMU/GPS inputs, the events other than `stop_event`, per-gear and histogram signals) is reported NOT_COMPUTED with a reason. No bus output, no VSS, no UDS DID (D-070 item 4: log only; the ride-summary DIDs come with class B). Outputs sit in a static table read by `derived_get()`; the future logger reads it.

**QM information (D-070 item 5).** It never feeds 0x020, safety-node, the heartbeat or the anomaly safety net. CI greps that `features/cornering`, `features/heartbeat`, `features/uds`, `features/vehicle_republish`, `services/` and `app/comms` never include `features/derived/` (the comms task only calls `services/derived_inputs`).

## Files

| File | What |
|---|---|
| `derived_core.{h,c}` | Pure core (no HAL, RTOS, services, clock): input array in, output table out. Float32, `isfinite` checks, `-fno-math-errno`, refuses `-ffast-math`. |
| `derived_gear.{h,c}` | Gear estimate from the measured ratio, pure and private to this feature for now; it moves to `services/` when a second feature needs a gear (architecture-guard). |
| `derived.{h,c}` | Glue of the derived task: reads `services/derived_inputs`, maps it to the core's inputs, steps the core, `derived_get(sig, &out)`. |
| `../../services/derived_inputs.{h,c}` | The comms task to derived task hand-over (below). |

## Task placement and the cross-task input

`services/vehicle_signals` is single-task (the UDS client writes it in the comms task), so the derived task never reads it. `derived_inputs_publish(now)` runs once per pass in `comms_pass()` as the last call, after `uds_server_step()` (nothing later writes `vehicle_signals`, and no frame work waits for the copy): one bounded pass over the 40 DIDs copies {value, sample stamp, state} into a `services/snapshot` (three static slots, initialised INVALID before either task starts). `derived_step()` is the one consumer call site and copies the whole record out in the same call (snapshot second-reader rule). The record carries a pass counter; `test_derived_inputs_stress` (ASan/UBSan and TSan) shows a record is never torn. Entries keep the sample's own stamp, so a stalled comms task makes every output stale by the signals' own limits.

The derived task runs below the EKF and below the comms task (CLAUDE.md rule 4, D-070 item 5): no blocking, no lock, static memory, every loop bounded by a compile-time size. In the host program it is stepped from the main loop after `comms_pass()`. The H7 task wiring comes with the H7 port. `derived_step()` and `derived_get()` are for the derived task only; a future logger runs in that task or gets its own snapshot of the output table.

## Rules (the reference engine: moto-server `derived/engine.py`, `context.py`)

- **Computed set** is decided at init from the gen table (class A, `rt_core_ready`, inputs only DIDs or computed derived signals with a lower index, a formula in the dispatch switch, windows that fit the sample buffers). It is not a hand-written list of 28: `test_derived_core` checks that it equals the set in the golden vectors (28 = 27 point/integral signals + `stop_event`; no difference). Evaluation order is the gen index order; a test asserts every derived input comes earlier.
- **Sample-and-hold with age.** A value is usable while `age <= stale_after_ms` (age equal to the limit is fresh, one more is stale, like the reference). The limit is the DID's own `stale_after_ms` from `vehicle_cl250_dids[]` for a DID input (D-070; e.g. 300 ms for the speed even where a signal's own limit is 330) and the producer's `stale_after_ms` for a derived input. The glue passes the sample stamp and the core applies that limit, the same one `vehicle_signals` uses for its STALE. **Open point (recorded, kept as in the reference):** the event signal `stop_event` has no `stale_after_ms` in gen/, so its inputs use `DERIVED_DID_AGE_LIMIT_POLLS` (3) times the DID's poll period (330 ms for speed and RPM) instead of the DID's own limit. A sample counts as lapsed when older than the longest limit and stays unusable until a new sample (a wrapped counter can not revive it).
- **Grid.** Each signal has a `period_ms` grid that starts at the latest first-sample time of its inputs (a derived input: the producer's start). A step evaluates the latest due tick of a signal (no catch-up) and stamps the output with the tick. Called every ms (tests) it is the reference exactly; called every few ms it is up to one step late.
- **Range, never clamped.** A result outside the gen `[min, max]` is invalid with OUT_OF_RANGE (the unranged value is kept for diagnostics). A non-finite input or result is MATH_ERROR (the reference reports a non-finite result as out_of_range; its inputs can not be NaN).
- **Flags** are the reference's, in the reference's input order (STALE, INVALID_INPUT, INSUFFICIENT_SAMPLES, STANDSTILL, NO_CONSUMPTION, NO_POWER, ENGINE_OFF). Every input read must be declared in the signal's gen inputs, else MATH_ERROR (fail closed).
- **Windows** (acceleration, jerk, fuel average, coolant and oil rate): fixed ring buffers per DID or derived producer, sized at compile time. gen/ exports window and period only as table data, not constant expressions, so a `_Static_assert` can not derive the capacity; instead `DERIVED_WIN_*_CAP` are named constants checked against the gen table at init (`window_ms / poll period + 2`): a window that does not fit makes its signal NOT_COMPUTED (reason SIZING), never silently truncated, and a test asserts none does.
- **Integrals** (trip distance, trip time, idle time) follow the reference's hold rule: a value is held from a sample until the next sample but never longer than the shortest remaining freshness of its inputs, so a gap adds nothing. The accumulators are `double` (a long ride would lose small increments in float32); the M7 has a double-precision FPU.
- **`stop_event`** is not on a grid: one row per stop at the time the vehicle moves off (value = duration in s, the reference's rule: speed 0 after moving, engine running, a gap of more than three speed polls ends it).

## Algorithm parameters ported from the reference (all provisional, D-070 item 6)

Vehicle facts (wheel radius, reduction ratios, gear ratios, stoichiometric AFR, fuel density, tank size) come only from gen/ `vehicle_cl250_constants.h`; signal indices, periods, ranges and inputs only from gen/ `vehicle_cl250_derived.h`. Numbers the reference hard-codes are named in `derived_core.h` / `derived_gear.h` with the reference file and line:

| Constant | Value | Reference | Kind |
|---|---|---|---|
| `DERIVED_GEAR_MATCH_TOLERANCE` | 0.05 | formulas.py:62 | algorithm parameter |
| `DERIVED_DID_AGE_LIMIT_POLLS` | 3 | context.py:27 | algorithm parameter (event signals only) |
| `DERIVED_MIN_WINDOW_SAMPLES` | 2 | formulas.py:160, :417 | algorithm parameter |
| `DERIVED_J1349_REFERENCE_PRESSURE_KPA` / `_TEMPERATURE_C` | 99, 25 | formulas.py:56-57 | algorithm parameter |
| `DERIVED_KMH_TO_MPS`, `DERIVED_RPM_TO_RAD_PER_S`, `DERIVED_PI` | 1/3.6, 2 pi/60 | formulas.py:39-40 | unit / math |
| `DERIVED_R_DRY_AIR`, `DERIVED_KELVIN_OFFSET`, `DERIVED_STANDARD_GRAVITY` | 287.058, 273.15, 9.80665 | formulas.py:43-47 | physics |
| `DERIVED_MG_PER_KG`, `_G_PER_KG`, `_SECONDS_PER_MINUTE`, `_SECONDS_PER_HOUR`, `_GPS_TO_KG_PER_H`, `_PA_PER_KPA`, `_PERCENT`, `_MS_PER_S`, `_REVOLUTIONS_PER_INJECTION` | unit factors, 2 (four-stroke) | formulas.py:49-54, :155, :168, :296, :342, :395 | unit |

No vehicle fact is missing from gen/ for these 28 signals, so no signal is left out for a `/signal-change`.

## Tests

- `test_derived_core`: golden replay of the four scenarios (10 801 rows, stepped every ms): grid time, valid flag and flag code exact, values within the per-signal tolerance (resolution / 2), equal counts of rows and core evaluations, event rows; zero rows needed the boundary allowance (the rule is documented at the top of the test; the skip list is asserted empty). Plus hand-computed unit tests (torque and power, air density and altitude, fuel formulas, residuals, gearbox ratio and gear, zero-guard flags, window slope and mean, integrals across a gap, the stop event and its gap rule, staleness at the limit and one past, range without clamping, NaN/inf, lapse, late steps, coarse stepping on the grid, not-computed reasons, dependency order, computed set = golden set) and the pinned-gen check below.
- **Vectors vs the pinned gen table.** The expected outputs depend on the gen table the vectors were generated from. `test_vectors_match_the_pinned_gen_table` hashes (FNV-1a) everything of gen/ they depend on (period, stale limit, window, range, resolution and inputs of every vector signal, the poll period of every DID read, and the twelve vehicle constants the generator uses) and compares it with the value recorded in the test; it also checks `abs_tol == resolution / 2` and that inputs follow the poll periods. A defs bump that changes any of them fails this test with "regenerate the vectors", not with unrelated value differences. The vectors' banner says v0.14.0; the content is identical under the pinned v0.18.1.
- `test_derived_gear`, `test_derived_inputs`, `test_derived` (glue), `test_derived_inputs_stress` (two threads, TSan too).

## Memory (D-070 item 6)

Static RAM for part 1, measured with `arm-none-eabi-size` on the M7 release build: **about 13.2 KB** (`derived.c` 11 704 B = the core with its tables and sample buffers, the 40 inputs and the newest record; `derived_inputs.c` 1 477 B = three snapshot slots; 14.4 KB with the host's alignment). That is **over D-070's estimate of at most about 10 KB for all signals**, already for 28 of them: the 60 s fuel-consumption window keeps 547 speed samples (4.4 KB), the biggest single block, and the per-signal tables over all 141 gen indices take 5.6 KB (host). Recorded here per D-070 item 6; trimming options are a shorter window or a running-sum average, and a table over the computed signals only. When the H7 linker script lands, this block must stay out of the EKF's DTCM.

## Cost (D-070 item 6)

Numbers are in the PR description. The target WCET comes with the board; the host numbers are only a relative guide. Bounded work per step: one pass over 40 DIDs, one over the gen table, at most one evaluation per computed signal plus three integral updates, every window loop limited by its buffer (the largest, the 60 s fuel average, is 548 speed samples).
