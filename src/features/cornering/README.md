# features/cornering

rt-core's cornering **Layer 2 only**: estimates that feed safety-node's cornering decision (hardware-architecture §5b.2, D-041, D-065). rt-core never makes the cornering warning/LED decision (CLAUDE.md safety rule 1). Today the module has one part, the lean (roll) estimate: the pure core, its glue and the 0x020 sender. µ, mass and the centre of gravity follow later.

## Status (D-065 item 4, PR order)

| PR | Content | State |
|---|---|---|
| (0) | defs `lean_angle_clamp_max_deg` (v0.9.0) | done (defs#52) |
| (1) | `services/snapshot`, `services/alive`, heartbeat `ekf_stalled` | done (rt-core#27) |
| (2) | `lean_core`: pure 2-state roll EKF + host tests (this module) | done (rt-core#28) |
| (3a) | `hal/imu.h`, `services/imu` + `imu_core` (interval-mean rates, z offset at standstill), host IMU fake (user split PR 3, 2026-10-10) | done (rt-core#29) |
| (3b) | glue `lean.c/.h` (EKF task, alive bump, snapshots), 0x020 send driven by `app/comms` with the output clamp (`lean_tx`, `lean_tx_core`), defs bump to v0.9.0, SIL IMU feed | done (rt-core#30) |
| (b) | two-thread `test_lean_link_stress` (E-17 (b), D-067) | done (rt-core#31) |
| (a) | DID 0xFD03 `LEAN_HEALTH` (E-17 (a), D-067), defs v0.14.0: EKF-side health fields in the lean record, `services/lean_health`, second-reader rule in `services/snapshot.h` | this PR (draft, stacked on rt-core#34 ← #33) |

Other code includes `lean.h` (EKF task) and `lean_tx.h` (comms task), never `lean_core.h` directly: `lean.h` includes it for the payload types (`lean_speed_t`, `lean_out_t`).

## lean_core (D-065 item 1)

**Inputs** (per EKF step, passed by the caller):
- `lean_imu_t`: body gyro x and z in rad/s and the sample's timebase stamp. Axes: x forward, y right, z down. Roll is positive leaning right, matching the sign of 0x020 `LEAN_ANGLE`. **Producer contract** (`services/imu`, PR 3; safety-reviewer MAJOR-4): each rate is the mean over (previous stamp, stamp], both axes filtered alike, never a point sample, since the single cylinder's firing vibration (about 25-75 Hz) would alias at the 50 Hz EKF rate. The z gyro offset is removed at standstill.
- `lean_speed_t`: the speed as it crosses tasks (D-065 item 2). It carries the physical m/s, the absolute stamp of the sample (D-051) and a state: an allow-list where only `LEAN_SPEED_VALID` is usable.

**Output:** `lean_out_t` by value. It has the roll in degrees, QUALITY 0..100 %, `LEAN_OUT_ESTIMATED` or `LEAN_OUT_INVALID`, and the reason for INVALID. The glue maps the state to the gen/ 0x020 choices with a switch, never a cast. The core's values (INVALID 0, ESTIMATED 1) differ from the wire values (ESTIMATED 0, INVALID 3). RESERVED (2) is never sent.

**Model** (2-state EKF over x = [roll, roll-rate gyro bias], float32):
- **Predict:** roll += (gyro_x − bias)·dt, with P = F P Fᵀ + diag(q_roll·dt, q_bias·dt).
- **Measure:** the coordinated turn tan(roll) = v·ψ̇/g, where ψ̇ is the earth yaw rate. With pitch at 0, the body z gyro reads r = ψ̇·cos(roll), so z = gyro_z, h(x) = (g/v)·sin(roll) and H = [(g/v)·cos(roll), 0]. Using atan(v·gyro_z/g) instead would estimate atan(sin(roll)), which is 35° for a true lean of 45°.
- **Speed in h:** v is the **upper bound** of the true speed, sample + age × `PLATFORM_LIMIT_VEHICLE_SPEED_ACCEL_MARGIN_MPS2`. A stale sample therefore makes the lean err high, the side on which D-041 warns earlier. This was the user's choice on 2026-10-10 (safety-reviewer MAJOR-3). The cost is an over-reading while braking, up to atan of (1 + margin/v) times the tangent; in the test, a 300 ms old 15 m/s sample reads at least 45° for a true 45°, whether the bike is accelerating or braking.
- **R_eff:** R + (∂h/∂v)²·margin² + (H0·`LEAN_YAW_LAG_S`·roll rate)². The last term is the transient model error. In a turn-in the yaw rate lags the roll by roughly the steering response (0.2 s, provisional), so the steady-turn relation is off by about lag × roll rate (safety-reviewer MAJOR-2).
- **Bias freeze:** while |roll rate| > `LEAN_BIAS_LEARN_RATE_MAX_RPS` (10°/s), and for `LEAN_BIAS_HOLD_MS` (600 ms, three lags) after the last such step, the bias gain is 0. A turn-in and the tail of the lagging yaw rate therefore never leak into the bias (MINOR-1 on the fixes). The Joseph-form covariance update stays exact for that sub-optimal gain.
- **Documented limits:**
  - Pitch ≈ 0. A pitch θ also adds −ψ̇·sin θ to the roll-rate gyro (p = roll rate − ψ̇·sin θ), and the measurement assumes θ = 0.
  - A steady coordinated turn. Transients are carried by the gyro and the gate.
  - The z gyro's own bias is not estimated. An offset b_z gives a roll error of about v·b_z/(g·cos roll), for example 0.9° for 0.5°/s at 15 m/s. `services/imu` (PR 3a) removes the offset at standstill.
  - Below `LEAN_SPEED_MIN_MPS` the lean is INVALID.

**Check order.** Each step runs these checks in order. The first one that fails wins, and every failure gives INVALID with roll 0 and QUALITY 0 (D-065 item 1, safety-reviewer MAJOR-1 on defs v0.9.0):

1. A NULL argument gives `ARG`.
2. A gyro sample that is non-finite or has |rate| ≥ 2000°/s gives `INPUT`. The step is skipped and the state is left untouched. The last stamp is kept, so the next step sees the gap.
3. The gap since the last sample is checked next. The first step, a gap of more than `LEAN_DT_MAX_MS` (40 ms) or a backward stamp gives `DT` and reinitialises the filter. A duplicate stamp (gap 0) gives `DUPLICATE` and is only skipped, with the state kept (safety-reviewer MINOR).
4. Predict.
5. Filter health: the state must be finite, both variances positive, P positive semi-definite and |bias| ≤ 5°/s. A failure gives `FILTER` and reinitialises.
6. With a usable speed, the measurement update runs behind the innovation gate. A gate run that lasts too long gives `GATE` and reinitialises (see below). Filter health is then checked again. Without a usable speed the roll is unobservable: once p00 exceeds the start-up prior P0, roll and p00 go back to it (roll 0, p01 0, p11 capped at its P0) while the bias estimate is kept. This bounds roll and P at standstill (safety-reviewer MAJOR-1, MINOR-2 on the fixes).
7. |roll| ≥ 90° gives `ROLL_RANGE` and restarts the filter, with or without a speed. h = sin is not monotone beyond 90°, so a roll that drifted past it, for example during a long stop on an unlearnt gyro offset, would otherwise converge to the mirror solution (180° − roll) and stay INVALID for the rest of the ride (MAJOR-1).
8. A roll σ above 5° gives `VARIANCE`. This includes a filter that has not converged yet: P0 is (30°)².
9. A missing usable speed gives `SPEED`. Predict still runs.

**Speed rules** (vehicle_speed limits, gen/ `platform_limits.h`, defs v0.8.0). The core computes the age itself at every step, so a stalled producer reads INVALID within the max age (architecture-guard BLOCKER-1 on the plan). The speed is usable only when all of these hold:
- the state is `VALID` and the value is finite;
- the age is ≤ `PLATFORM_LIMIT_VEHICLE_SPEED_MAX_AGE_MS` (300 ms);
- speed − age × 5 m/s² ≥ `LEAN_SPEED_MIN_MPS`.

A stamp up to `LEAN_SPEED_FUTURE_MAX_MS` (40 ms) ahead of the IMU stamp has age 0. The IMU sample can be up to one EKF period older than the newest speed write. A stamp further ahead is unusable. The age is computed with unsigned subtraction in both directions and never with a signed cast, so a future stamp never becomes a huge or negative age (E-17).

**Innovation gate** (D-065 item 1). The gate tests the normalised innovation squared, ν²/S, against 9 (3σ). A rejected update leaves the state on gyro propagation:
- **Within the window:** ESTIMATED stays, with QUALITY capped at `LEAN_QUALITY_REJECT_MAX` (50 %) while the run is open. This was the user's decision on 2026-10-10, after architecture-guard MAJOR-1: a single reject is often a steering transient, while a diverged filter shows up as a sustained run.
- **Run longer than `LEAN_GATE_REJECT_MAX_MS` (100 ms)** after its first reject: the filter counts as diverged. The step gives INVALID (`GATE`) and reinitialises, and ESTIMATED returns only after the variance has converged again.
- **Ending a run:** an accepted update ends it. A step without a usable speed holds the run, and its age keeps counting.

**QUALITY** comes from the roll σ: 100 % up to 1°, then linear (truncated) down to 1 % at 5°. ESTIMATED never sends 0, because 0 means no confidence. `lean_core_quality()` is pure and defined for every input.

**CLAMPED** (D-065 item 3) belongs to the glue (PR 3). It clamps the returned copy to gen/ `PLATFORM_LIMIT_LEAN_ANGLE_CLAMP_MAX_DEG` (55°, defs v0.9.0), and only after the core has already ruled out every INVALID condition. The core estimates leans above 55° unclamped (test: 60°), and the clamp can never reach the EKF state.

**Constants.** All constants are named in `lean_core.h` with their rationale. The noise values (Q, R), P0, the gate window, the minimum speed, the dt and future limits, the bias limit and the QUALITY map are **provisional until the T1/T4 measurements**. They are tuning values, not signals (invariant 2). Physics constants: g = 9.80665 m/s², and 90° as the model bound.

**For the PR 3 glue** (safety-reviewer MINOR; all done in PR 3b, see "Lean glue" below):
- ~~Mark a speed `LEAN_SPEED_VALID` only when gen/ accepts its range~~ (`lean_tx_publish_speed()`, gen/ 0x021 range).
- ~~Add a test that the standstill z-offset removal works~~ (`test_lean`: turns both ways read true after a stop).
- ~~Call the core only when there is a new IMU sample~~ (`IMU_REASON_NO_NEW` skips the step).

**For PR 3b, from the reviews of PR 3a** (architecture-guard on the plan, safety-reviewer on 3a):
- ~~`IMU_REASON_NO_NEW` means "skip this step"~~ (no output, no alive bump; test).
- ~~Standstill for `imu_read()`~~: VALID, below `LEAN_STANDSTILL_MPS`, at most `LEAN_STANDSTILL_AGE_MAX_MS` old, and the previous sample read 0 as well (tests: alternating samples, a zero that stops being refreshed, an UNUSABLE sample just before the learn, the window opening only at the second zero).
- ~~An INVALID IMU sample gives an INVALID output without a core step~~ (tests: 1, 2 and 3 missed periods).
- ~~Report `reject_count` and the time without a learnt offset (UNLEARNED/RANGE), with the snapshot fault latches, so a bad gyro is visible~~ (E-17 (a), D-067: DID 0xFD03 `LEAN_HEALTH`, see "DID 0xFD03" below). The second-reader rule is now in `services/snapshot.h`.
- ~~A small future output stamp is age 0, a large one INVALID~~ (`lean_tx_core`, architecture-guard MAJOR-2).
- ~~MAJOR-3, MAJOR-4, MINOR-4, MINOR-7~~: see "Lean glue" below. MAJOR-4 needs no separate atomic word: the EKF-side consumer fault travels inside the output record. A latched producer fault stops every further write to that snapshot's slots, not only the publish, because the latching publish already handed the write slot to the consumer (safety-reviewer MINOR-1 on 3b). The last output record then ages out to INVALID within `LEAN_TX_AGE_MAX_MS`, and a speed producer fault makes 0x020 INVALID at once (comms-side fault).
- T1: measure the period-mean spread at idle on the CL250; above 1 deg/s the offset is never learnt. Also the raw per-axis LSB spread with the engine off (false FROZEN at standstill), and add per-axis stuck detection on x and z (a single stuck channel is not FROZEN today; safety-reviewer MINOR-D on the 3a fixes).

**Availability cost (fail-safe, to measure on T1/T4).** Each of these makes the lean INVALID until the variance has converged again:
- an IMU gap longer than 40 ms mid-corner;
- a gate run longer than 100 ms;
- an implausible state.

Convergence after a reinit takes a few steps at 20 ms (test: within 1 s). The bias needs tens of seconds of riding (tests: 120 s).

## Lean glue (PR 3b, D-065 item 2)

### Tasks and data

| Part | Task | Does |
|---|---|---|
| `lean_link_t` (`lean.h`) | shared, boot-initialised | speed snapshot (comms → EKF), output snapshot (EKF → comms), alive counter (EKF writes) |
| `lean_ekf_step()` (`lean.c`) | EKF task, every `PLATFORM_EKF_LEAN_CYCLE_TIME_MS` (20 ms) | reads the newest speed, drains the IMU (`services/imu`) with the standstill flag, steps `lean_core` for a new VALID sample only, publishes {output, IMU stamp, EKF-side snapshot faults}, bumps alive last |
| `lean_tx_publish_speed()` (`lean_tx.c`) | comms task, after `uds_client_step()` | hands the 0xF40D sample over as m/s + absolute stamp + state; VALID only inside the gen/ 0x021 range |
| `lean_tx_step()` (`lean_tx.c`) | comms task, after `heartbeat_step()` | builds 0x020 at its gen/ cycle (`lean_tx_core`), E2E on a copy, commits the counter on `CAN_PORT_OK` only (D-056 item 5) |

The EKF task holds no lock, prints nothing and never allocates. It never touches `can_if` (D-065 item 2: no lock on the main-loop-only `can_if`, no priority inversion).

### Task priorities (D-065 item 4, MAJOR-6; checked on HIL with the board, Q-019)

| Priority | Context | Period |
|---|---|---|
| highest | FDCAN ISRs, IMU ISR (data ready or FIFO watermark into the `hal/imu.h` FIFO) | event |
| 1 (highest task) | EKF task: `lean_ekf_step()` | 20 ms |
| 2 | comms task: `comms_pass()` | ≤ `VEHICLE_CL250_CLIENT_STEP_MAX_MS` (10 ms) |
| 3 | anomaly, FFT, logging (later) | — |

The H7 port must static-assert `configTICK_RATE_HZ == 1000` and a 32-bit `TickType_t`, so the ms timebase and its wrap arithmetic hold (E-17). It must also place `lean_link_t` in SRAM with ECC enabled: `services/snapshot` detects only a corrupted index outside the three slots, and a bit flip to another valid index would let both tasks share a slot unnoticed (safety-reviewer, round 2 on 3b).

### Boot order (MINOR-7; `app/host/main.c` follows it)

1. `can_if_init()`, timebase, `diag_init()`, `vehicle_signals_init()`;
2. open the UDS client and server, the republisher and the heartbeat;
3. `lean_link_init()`, then `lean_ekf_init()` (filter, IMU service and standstill reset);
4. `comms_ekf_register(&ekf, lean_link_alive(&link))`;
5. `comms_open_lean()`: opens 0x020 only for exactly this link's counter and registers its dedicated TX buffer (3 of 8);
6. `comms_apply_filters()` (seals the filters and the dedicated buffers);
7. start the scheduler.

If step 4 or 5 fails, 0x020 stays closed and never reaches the bus: receivers see its timeout (D-042 fallback, D-065 amendment).

### WCET budget (MINOR-7; user, 2026-10-14; measured on the board with DWT CYCCNT, Q-019)

`lean_ekf_step()` takes at most **1 ms** per call: 5 % of its 20 ms period. The 0x020 output-age budget allows `LEAN_TX_JITTER_MS` (5 ms) of EKF and comms release jitter; the EKF preempting comms only moves an output stamp ahead of the comms `now`, which the 10 ms future window covers. The budget holds only while `hal_imu_pop()` reads RAM that the IMU ISR filled, never a blocking bus read. The work per call is bounded: at most `IMU_DRAIN_MAX` (64) FIFO samples, one `lean_core_step()` (one `sinf`, one `cosf`, one `sqrtf`, up to four divisions), two snapshot operations and one alive bump. No loop runs longer than the drain bound.

### DID 0xFD03 `LEAN_HEALTH` (E-17 (a), D-067, defs v0.14.0)

Diagnostic only (QM): no safety decision reads it, and `lean_tx_msg()` ignores every new field. A bad gyro (an offset beyond `IMU_OFFSET_MAX_RPS`, a dead IMU under NODE_MODE NORMAL) is now visible in a tester, not only as an INVALID 0x020. The layout, masks, widths, maxima and the 500 ms window are all gen/ (`PLATFORM_UDS_LEAN_HEALTH_*`); the rules are the DID's description and the comment block above its `max_age_ms` in `uds/dids.yaml` (the safety review's race-freedom contract).

| Field | Source | Task |
|---|---|---|
| `OFFSET_LEARNED`, `OFFSET_REJECTED`, `OFFSET_REJECTS`, `UNLEARNED_S`, `SPEED_CONSUMER_FAULT` | the EKF's lean record (`lean_record_t.health` + the existing `fault`), written each step by `lean_ekf_step()` before the publish | EKF (producer), copied by the comms task |
| `RECORD_FRESH` | the record's stamp against the request time, `lean_tx_core`'s age rule with the DID window | comms, at request time |
| `SPEED_PRODUCER_FAULT`, `OUTPUT_CONSUMER_FAULT` | `lean_tx_speed_producer_fault()`, `lean_tx_output_consumer_fault()` | comms, set by `comms_pass()` right before `uds_server_step()` |
| `EKF_ALIVE_OK` | `comms_ekf_registered()` AND NOT the value `comms_pass()` hands `heartbeat_step()` (`ekf_stalled` or an open lean with a foreign counter) | comms |

**Race freedom.** No new shared state exists and no second reader of the output snapshot:
1. The EKF fills the health fields of the slot it already owns (`snapshot_write_index()`) before the one `snapshot_publish()`, never after (a mutant that writes them after the publish is killed by TSan in `test_lean_link_stress`, 3 of 3 runs).
2. `lean_tx_step()` is the only consumer call site. In the same call, from the index that read returned, it copies the record's fields into `services/lean_health` (a comms-task static) and then builds 0x020 from the same slot. No index or pointer outlives the call.
3. The UDS server builds the DID from that comms-local copy plus comms-task flags. It never calls `snapshot_read_index()` and keeps no index, so it cannot take a record from 0x020.
4. The copy is refreshed only when `lean_tx_step()` is due (every 20 ms), which is far inside the 500 ms window.

**EKF side.** `imu_offset_health()` gives learned / rejected / `reject_count`; `reject_count` saturates at the gen/ maximum 0xFFFF (never wraps). `lean_health_unlearned_s()` counts whole seconds of the current run without a learnt offset: it starts at the first published step (boot) and again whenever a learnt offset is lost (a rejected window drops it), is 0 while one is learnt and saturates at 0xFFFF. Everything is computed from `now_ms` in the step, with no extra clock.

**Fail-safe.**
- No record since boot (the boot record's stamp 0 is never read as a record) or one older than the window: `RECORD_FRESH` and `OFFSET_LEARNED` clear, `UNLEARNED_S` = UNKNOWN (0xFFFF), the other EKF fields their last values (0 before the first record). Both saturation and "not fresh" read as not learnt.
- A stamp more than `LEAN_HEALTH_FUTURE_MAX_MS` (= `LEAN_TX_FUTURE_MAX_MS`, static-asserted in `lean_tx.c`) ahead of now is not fresh, never a wrapped age.
- A latched output-snapshot producer fault stops the records for good, so it reads `RECORD_FRESH` clear while `EKF_ALIVE_OK` stays set (the EKF task still runs); a stalled EKF clears `EKF_ALIVE_OK`.
- `RECORD_FRESH` set does not mean 0x020 is valid (that needs the 35 ms `LEAN_TX_AGE_MAX_MS`, the state and the range).
- RESERVED is never set. A DID length that differs from gen/ is refused (NRC 0x22), not guessed.

**Tests.** `test_lean_health_core` (pure, 100 % line and branch): flags at their gen/ masks, window boundary 500 vs 501 ms, future stamp +10/+11 ms and +2^31, timer wrap, saturation, the unlearned run (anchor, learnt, lost, saturation at 49.7 days, wrap), NULL and wrong length, the service. `test_imu`: offset health follows learn, reject, relearn. `test_lean`: the record carries the fields, a rejected window, EKF-side saturation, the copy in `lean_tx_step()`, the boot record is no record, a record consumed by another reader still counts, latches. `test_comms_lean` (SIL, real `comms_pass()`): running, never-run, stopped, unregistered and foreign supervision EKF, producer fault, each latch, an unlearnable gyro, and that building the DID never costs 0x020 a frame. `test_uds_server`: the DID end to end over ISO-TP. `test_lean_link_stress`: `record_consistent()` checks the new fields on every record (the offset flags agree with the state and the stamp, a learnt offset never goes back, nothing rejected), and the comms thread builds the DID every pass from the production copy.

**Memory (M7 release).** `lean_record_t` 16 -> 20 B (`lean_link_t` 104 -> 116 B), `lean_ekf_t` 156 -> 164 B (the 8 B unlearned-run tracker), `lean_tx_t` stays 32 B, `services/lean_health` 19 B bss. Text: `lean_health_core.c.obj` 206 B, `lean_health.c.obj` 100 B, `lean.c.obj` 454 -> 518 B, `lean_tx.c.obj` 366 -> 502 B.

### 0x020 rules (`lean_tx_core.h`)

The rules apply in order. The first that fails sends INVALID with LEAN 0 and QUALITY 0 (never RESERVED):
1. a snapshot fault on either side;
2. the output age: a stamp up to `LEAN_TX_FUTURE_MAX_MS` (10 ms) ahead of now is age 0. Further ahead, or older than `LEAN_TX_AGE_MAX_MS` (20 + 10 + 5 = 35 ms, static-asserted below the receivers' 60 ms E2E timeout), it is INVALID, since a replayed stale output would still pass E2E;
3. the core state, through a switch: only ESTIMATED goes on;
4. a finite roll strictly inside 90°, and a QUALITY within `LEAN_QUALITY_MIN` and the gen/ range;
5. the clamp: |roll| above gen/ `PLATFORM_LIMIT_LEAN_ANGLE_CLAMP_MAX_DEG` (55°) goes out as ±55° with state CLAMPED. Only the copy is clamped.

### Memory (M7 release, `arm-none-eabi-size`, 2026-10-14)

- **RAM** (static, no heap): `lean_link_t` 104 B, `lean_ekf_t` 156 B (includes `lean_core_t` and `imu_t`), `lean_tx_t` 32 B.
- **Flash** (text): `lean.c.obj` 454 B, `lean_tx.c.obj` 366 B, `lean_tx_core.c.obj` 246 B. Total `moto_rtcore_logic` + `moto_rtcore_fw` text: 16108 B.

### Tests

- `test_lean_tx_core` (L0, pure):
  - clamp boundaries 54.99/55.00/55.01/−55.01 → ±5500 CLAMPED with the sign kept;
  - 90°, 180°, NaN, ±Inf → INVALID;
  - the state switch never maps to RESERVED (sweep);
  - QUALITY 0, 101 and 255 rejected, 1 and 100 accepted;
  - age 35 vs 36 ms, and across the timebase wrap;
  - future stamps +10/+11 ms and +2³¹;
  - snapshot faults and NULL;
  - packing and E2E on a copy of the committed state;
  - clamping a real core output leaves the filter byte-identical.
- `test_lean` (L0, the glue on the host IMU fake):
  - boot slots INVALID, and INVALID without a learnt offset;
  - after a stop, turns both ways read true;
  - one alive bump per complete cycle, none on NO_NEW;
  - 1, 2 and 3 missed IMU periods;
  - a stalled comms task → INVALID within 300 ms;
  - speed hand-over only inside the gen/ range;
  - the standstill rules (above), and snapshot faults in the record;
  - a corrupted `middle` word on either snapshot (a real RAM fault, not a set flag): the producer never writes that slot again (MINOR-1);
  - 0x020 opens only for this link's counter (MAJOR-3) and stays closed when its dedicated buffer is refused;
  - NULL and unbound calls.
- `test_comms_lean` (SIL: real `comms_pass()`, UDS client, simulated ECU, heartbeat, a platform receiver checking E2E):
  - 0x020 every 20 ms with E2E;
  - an EKF that never runs → NODE_MODE DEGRADED and 0x020 INVALID;
  - an EKF at a third of its rate → 0x020 INVALID while NODE_MODE stays NORMAL (accepted MINOR-5);
  - a stalled comms task;
  - no registered EKF, or a foreign counter → no 0x020 at all;
  - a refused frame is retried with no counter gap and a fresh output;
  - a bus-off drops cycles without advancing the counter, and the receiver resyncs (INITIAL);
  - comms-side snapshot faults → INVALID;
  - a corrupted output word → INVALID once the last record is older than its budget; a corrupted speed word → INVALID at once (MINOR-1);
  - a comms task given another (zeroed) `comms_ekf_t` → no 0x020 and NODE_MODE DEGRADED (MINOR-2);
  - a real 62° turn both ways → 0x020 CLAMPED ±5500 with the sign kept, while the EKF itself reads above 55°.
- `test_lean_link_stress` (two threads on one `lean_link_t`, under ASan + UBSan and ThreadSanitizer; E-17 (b), D-067):
  - the real `lean_ekf_step()` in one thread; the real `lean_tx_publish_speed()` + `lean_tx_step()` on an in-process platform bus in the other, one pass per ms;
  - a shared 1 ms virtual clock that the EKF may lead by at most 5 ms (a tolerated preemption, ≤ `LEAN_TX_FUTURE_MAX_MS`). The handshake orders everything else for TSan: EKF step S races only the comms passes S-4 .. S. So the test runs five scenarios, with the 0x020 send grid at S - k for k = 0..4, and every send pass of the production reader can race an EKF step (safety-reviewer MAJOR-1). 3000 EKF cycles each;
  - TSan reports an access the snapshots and the alive counter do not order, within that window. The workers check: every speed belongs to its stamp (the speed alternates with the stamp's parity); speed and output stamps never go backwards; every pass reads the alive count and then the record, and the record is never older than the count says; the alive count never decreases; the record fields agree with its state; one E2E-clean 0x020 frame per cycle; no frame turns INVALID once 0x020 is valid; the alive total equals the cycle count; no snapshot fault latches;
  - mutation check, 10 runs each: the EKF writing its output slot after the publish and `lean_tx_step()` reading the two slots it does not own are killed by TSan only (the values do not change); the comms task rewriting a published speed stamp and the alive bump moved before the publish are killed in both builds;
  - limits: the EKF thread feeds the IMU fake (`test_imu` has a feeder thread); no EKF overrun, WCET or preemption depth (HIL, Q-019); not the H7 `hal_atomic` port; the lean side only, not the whole `comms_pass()`; the test's per-ms record read leaves "publish replaces an unread record" to `test_snapshot_stress`.

**Coverage:** `lean.c` has 100 % lines and branches; `lean_tx.c` and `lean_tx_core.c` have 100 % lines. The two branches left are a packing/E2E failure that a valid message cannot produce.

**Mutation check** (24 mutants on the glue, the sender and `app/comms`): 23 are killed. The survivor, `comms_open_lean()` skipping its own registration check, is equivalent: `lean_tx_open()` refuses the same NULL counter.

## Follow-ups (E-17 after (a))

(a) is done in this PR. What remains of E-17, none of it blocking the lean path:
- **(c) H7 port:** ECC on the SRAM holding `lean_link_t` (the snapshot detects only out-of-range indices), `configTICK_RATE_HZ == 1000` and 32-bit `TickType_t` static asserts, and `hal_imu_pop()` RAM-only (WCET budget).
- **(d) HIL scenarios** (list above, including the ±30° sign check, `lean_imu_dead` with the 0xFD03 report, `lean_task_priorities`). Add: read 0xFD03 while `lean_imu_dead`: expected `RECORD_FRESH` set, `OFFSET_LEARNED` clear and `UNLEARNED_S` growing after a power-up, `EKF_ALIVE_OK` set.
- **(e) T1/T4:** tune the provisional constants (`lean_core` noise and gate, standstill 0.1 m/s / 200 ms, the offset window) and measure the reinit availability cost.
- **0xFD03 only:** `UNLEARNED_S` starts at the EKF's first step; if the H7 EKF task starts noticeably after boot, that is the intended anchor, so state it in the bring-up notes.

## Memory and timing

- **RAM:** `lean_core_t` is 36 B, owned by the caller (static in the PR 3 glue). There is no heap and no loop. Each step does a fixed number of float operations: one `sinf`, one `cosf` and one `sqrtf`, plus up to four divisions, each by a value checked to be positive (MISRA Dir 4.1, Dir 4.11).
- **Flash** (M7 release, `arm-none-eabi-size`, 2026-10-10): `lean_core.c.obj` has text 1372 B and bss 0. Total `moto_rtcore_logic` + `moto_rtcore_fw` text: 12370 → 13742 B. The libm functions (`sinf`, `cosf`; `sqrtf` becomes VSQRT with `-fno-math-errno`) are linked with the firmware in PR 3.
- **Build:** `-fno-math-errno` is set on `moto_rtcore_logic`. `lean_core.c` refuses to build with `-ffast-math` (`__FAST_MATH__`) or without `-fno-math-errno` (`__NO_MATH_ERRNO__`), because `isfinite()` and the NaN-false comparisons need IEEE semantics.

## Coverage (D-046)

`lean_core.c` has 100 % line and branch coverage (gcovr, host-coverage preset). The health check runs in the order predict propagates a non-finite value, so each condition can be reached and is tested.

## Tests (`tests/host/test_lean_core.c`)

- **Start-up:** the first step gives DT, then VARIANCE until converged. A zeroed object never reads P = 0 as certainty. ESTIMATED is reached within 1 s.
- **Nominal:** straight riding (0°, QUALITY 100); steady turns right and left (30° at 15 m/s, −45° at 25 m/s); a 0→40° ramp in 1 s, tracked within 1° without leaving ESTIMATED; gyro bias convergence on the straight (1°/s) and in a turn (−1.5°/s).
- **Speed:**
  - 0 m/s, negative, 4.99 m/s and exactly 5 m/s;
  - UNUSABLE, an unknown state value, NaN and ±Inf;
  - age 300 vs 301 ms;
  - the age-grown margin against the minimum;
  - a stalled producer reads INVALID within 300 ms + one step;
  - future stamps of +1 and +40 ms (age 0), +41 ms and +2³¹ (unusable);
  - timebase wrap for both dt and speed age.
- **Gyro:** NaN and ±Inf on either axis give INPUT with the state byte-identical. Exactly 2000°/s is rejected, the next float below is accepted. Bad samples spanning more than 40 ms lead to DT and a reinit.
- **dt:** 40 ms predicts; 41 ms and a backward stamp each give DT and a reinit; 0 ms gives DUPLICATE with the state byte-identical, and the next step is ESTIMATED.
- **Gate:**
  - a diverged filter (40° while riding straight) stays ESTIMATED with QUALITY ≤ 50 up to 100 ms after the first reject, gives GATE at the next reject, reinitialises and reconverges;
  - a single outlier caps QUALITY at 50, and the next accepted update ends the run;
  - a reject leaves a QUALITY already below the cap unchanged;
  - steps without a speed hold the run.
- **Filter health:** NaN or Inf in roll, bias, p00, p01 or p11; negative p00 or p11; P not positive semi-definite; |bias| above the limit; an update that pushes the bias out of range. Each gives FILTER and a reinit.
- **Variance and range:** without a speed the reason moves from SPEED to VARIANCE once σ > 5°. 90° and −120° (the latter without a speed) give ROLL_RANGE and a restart, 80° is estimated. 60° is estimated unclamped, and changing the output copy leaves the filter unchanged.
- **safety-reviewer scenarios:**
  - a 120 s stop with an unlearnt 1°/s roll-gyro offset: roll and p00 stay bounded and the roll restarts, then straight riding reads ESTIMATED within 1 s at the real lean, never the mirror;
  - a turn-in from 0 to 40° in 0.5 s with the yaw rate lagging 200 ms (first order): within 2° from 1 s and within 1° from 2 s after the turn-in began, never INVALID, the bias bit-exact during the turn-in and its 600 ms hold, and within 0.2°/s afterwards;
  - a long idle: the prior reset caps p11 at its P0 and keeps the bias;
  - a 300 ms old speed while the true speed differs by the full margin, accelerating and braking: the lean reads at least 45° for a true 45°;
  - ten minutes of a weaving ride (±25°, 8 s) with seeded Gaussian gyro noise at the model's own Q and R and a 0.5°/s bias: never GATE or FILTER, ESTIMATED throughout after convergence, RMS error ≤ 1.5°, mean NEES within [0.2, 3].
  
  Each fix was also checked by mutation: reverting it makes the matching test fail.
- **QUALITY map:** the boundaries (1°, just above 1°, 3°, 5°, just above 5°), negative values, NaN and Inf, and monotonicity.
- **NULL arguments.**

## HIL scenarios (with the board, moto-hil-bench)

- `lean_steady_turn`: replay a coordinated turn (IMU and speed restbus) at +30° and at −30°, 15 m/s. Expected: 0x020 ESTIMATED within 1 s with the right sign, |error| ≤ 1°, QUALITY ≥ 90. The sign check catches an IMU mounted wrong (x and z both inverted would read a consistent but mirrored lean); the H7 `hal/imu` port's mounting matrix is also checked on the bench.
- `lean_imu_dead`: unpower the IMU while the EKF runs. Expected: 0x020 INVALID for good and safety-node's D-042 fallback active; NODE_MODE stays NORMAL. After E-17 (a): the 0xFD03 `LEAN_HEALTH` report (D-067) shows the cause.
- `platform_comms_starved_e2e` extended to 0x020: a pending 0x020 sent after a comms stall longer than the timeout reads INITIAL at the receiver, never OK.
- `lean_speed_stall`: freeze the restbus speed during a turn. Expected: 0x020 INVALID with LEAN 0 and QUALITY 0 within 300 ms + one 0x020 cycle. NODE_MODE is unchanged, because the EKF still runs.
- `lean_imu_gap`: drop IMU samples for 60 ms. Expected: INVALID at once, then ESTIMATED again after reconvergence (≤ 1 s).
- `heartbeat_ekf_stall`: stop the EKF task on the target. Expected: NODE_MODE DEGRADED within the alive window, and 0x020 INVALID within `LEAN_TX_AGE_MAX_MS`.
- `lean_task_priorities`: the EKF task at its 20 ms period while the comms task runs at full load and ISRs jitter its release. Expected: `lean_ekf_step()` within its 1 ms WCET budget (DWT CYCCNT), 0x020 never INVALID through output age, and no snapshot fault latched (target snapshot stress with ISR-jittered preemption).
- `lean_turn_in_lag`: replay a real CL250 IMU and speed log of turn-ins. Expected: |error| against the reference ≤ 2° from 1 s after each turn-in began, and no low reading at QUALITY ≥ 90.
