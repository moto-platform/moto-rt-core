# features/cornering

rt-core's cornering **Layer 2 only**: estimates that feed safety-node's cornering decision (hardware-architecture §5b.2, D-041, D-065). rt-core never makes the cornering warning/LED decision (CLAUDE.md safety rule 1). Today the module has one part, the lean (roll) core. µ, mass and the centre of gravity follow later.

## Status (D-065 item 4, PR order)

| PR | Content | State |
|---|---|---|
| (0) | defs `lean_angle_clamp_max_deg` (v0.9.0) | done (defs#52) |
| (1) | `services/snapshot`, `services/alive`, heartbeat `ekf_stalled` | done (rt-core#27) |
| (2) | `lean_core`: pure 2-state roll EKF + host tests (this module) | done (rt-core#28) |
| (3a) | `hal/imu.h`, `services/imu` + `imu_core` (interval-mean rates, z offset at standstill), host IMU fake (user split PR 3, 2026-10-10) | done |
| (3b) | glue `lean.c/.h` (EKF task, alive bump, snapshots), 0x020 send driven by `app/comms` with the output clamp, defs bump to v0.9.0, SIL IMU feed | open |

Nothing calls the core yet. PR (3) adds `lean.h`, the module's public header for other code. It includes `lean_core.h` for the payload types (`lean_speed_t`, `lean_out_t`), so other code sees `lean_core.h` only through `lean.h`.

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

**For the PR 3 glue** (safety-reviewer MINOR):
- Mark a speed `LEAN_SPEED_VALID` only when gen/ accepts its range: the core has no upper plausibility bound, so it needs no hand-written limit.
- Add a test that the standstill z-offset removal works; an unremoved z offset makes one turn direction read low.
- Call the core only when there is a new IMU sample. A duplicate stamp is skipped, but it still returns INVALID, which would cost one 0x020 frame.

**For PR 3b, from the reviews of PR 3a** (architecture-guard on the plan, safety-reviewer on 3a):
- `IMU_REASON_NO_NEW` means "skip this step": no core step and no new output (test a second call in the same ms).
- Standstill for `imu_read()`: a VALID speed of 0 (with a tolerance, never float equality), within a shorter age than the 300 ms max, and also 0 in the previous speed snapshot; never from a STALE speed.
- An INVALID IMU sample gives an INVALID output without a core step; lean_core's 40 ms gap check then reinitialises it (tests: 1, 2 and 3 missed periods).
- Report `reject_count` and the time without a learnt offset (UNLEARNED/RANGE) on 0xFD02 or as a DTC, so a bad gyro is visible.
- The output stamp may be newer than the comms `now` (the EKF preempts comms): a small future stamp is age 0, a large one INVALID (architecture-guard MAJOR-2).
- `comms_open_lean` checks that the registered counter is the lean link's own (MAJOR-3); snapshot fault latches cross to comms through an EKF-written atomic word and read as INVALID (MAJOR-4); EKF-side and comms-side functions are split and documented per task (MINOR-4); boot order and an EKF WCET budget in the README (MINOR-7).
- T1: measure the period-mean spread at idle on the CL250; above 1 deg/s the offset is never learnt.

**Availability cost (fail-safe, to measure on T1/T4).** Each of these makes the lean INVALID until the variance has converged again:
- an IMU gap longer than 40 ms mid-corner;
- a gate run longer than 100 ms;
- an implausible state.

Convergence after a reinit takes a few steps at 20 ms (test: within 1 s). The bias needs tens of seconds of riding (tests: 120 s).

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

## HIL scenarios (with the board and PR 3, moto-hil-bench)

- `lean_steady_turn`: replay a coordinated turn (IMU and speed restbus) at 30° and 15 m/s. Expected: 0x020 ESTIMATED within 1 s, |error| ≤ 1°, QUALITY ≥ 90.
- `lean_speed_stall`: freeze the restbus speed during a turn. Expected: 0x020 INVALID with LEAN 0 and QUALITY 0 within 300 ms + one 0x020 cycle. NODE_MODE is unchanged, because the EKF still runs.
- `lean_imu_gap`: drop IMU samples for 60 ms. Expected: INVALID at once, then ESTIMATED again after reconvergence (≤ 1 s).
- `lean_turn_in_lag`: replay a real CL250 IMU and speed log of turn-ins. Expected: |error| against the reference ≤ 2° from 1 s after each turn-in began, and no low reading at QUALITY ≥ 90.
