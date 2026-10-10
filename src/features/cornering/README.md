# features/cornering

rt-core's cornering **Layer 2 only**: estimates that feed safety-node's cornering decision (hardware-architecture §5b.2, D-041, D-065). rt-core never makes the cornering warning/LED decision (CLAUDE.md safety rule 1). Today the module has one part, the lean (roll) core. µ, mass and the centre of gravity follow later.

## Status (D-065 item 4, PR order)

| PR | Content | State |
|---|---|---|
| (0) | defs `lean_angle_clamp_max_deg` (v0.9.0) | done (defs#52) |
| (1) | `services/snapshot`, `services/alive`, heartbeat `ekf_stalled` | done (rt-core#27) |
| **(2)** | **`lean_core`: pure 2-state roll EKF + host tests (this module)** | this PR |
| (3) | `services/imu`, glue `lean.c/.h` (EKF task, alive bump, snapshots), 0x020 send in `app/comms` with the output clamp, defs bump to v0.9.0, SIL IMU feed | open |

Nothing calls the core yet. PR (3) adds `lean.h`, the module's public header for other code. It includes `lean_core.h` for the payload types (`lean_speed_t`, `lean_out_t`), so other code sees `lean_core.h` only through `lean.h`.

## lean_core (D-065 item 1)

**Inputs** (per EKF step, passed by the caller):
- `lean_imu_t`: body gyro x and z in rad/s and the sample's timebase stamp. Axes: x forward, y right, z down. Roll is positive leaning right, matching the sign of 0x020 `LEAN_ANGLE`.
- `lean_speed_t`: the speed as it crosses tasks (D-065 item 2). It carries the physical m/s, the absolute stamp of the sample (D-051) and a state: an allow-list where only `LEAN_SPEED_VALID` is usable.

**Output:** `lean_out_t` by value. It has the roll in degrees, QUALITY 0..100 %, `LEAN_OUT_ESTIMATED` or `LEAN_OUT_INVALID`, and the reason for INVALID. The glue maps the state to the gen/ 0x020 choices with a switch, never a cast. The core's values (INVALID 0, ESTIMATED 1) differ from the wire values (ESTIMATED 0, INVALID 3). RESERVED (2) is never sent.

**Model** (2-state EKF over x = [roll, roll-rate gyro bias], float32):
- **Predict:** roll += (gyro_x − bias)·dt, with P = F P Fᵀ + diag(q_roll·dt, q_bias·dt).
- **Measure:** the coordinated turn tan(roll) = v·ψ̇/g, where ψ̇ is the earth yaw rate. With pitch at 0, the body z gyro reads r = ψ̇·cos(roll), so z = gyro_z, h(x) = (g/v)·sin(roll) and H = [(g/v)·cos(roll), 0]. Using atan(v·gyro_z/g) instead would estimate atan(sin(roll)), which is 35° for a true lean of 45°. The speed's own uncertainty enters as R_eff = R + (∂h/∂v)²·σ_v², with σ_v = age × `PLATFORM_LIMIT_VEHICLE_SPEED_ACCEL_MARGIN_MPS2`. The covariance update uses the Joseph form.
- **Documented limits:**
  - Pitch ≈ 0.
  - A steady coordinated turn. Transients are carried by the gyro and the gate.
  - The z gyro's own bias is not estimated. An offset b_z gives a roll error of about v·b_z/(g·cos roll), for example 0.9° for 0.5°/s at 15 m/s. `services/imu` (PR 3) removes the offset at standstill.
  - Below `LEAN_SPEED_MIN_MPS` the lean is INVALID.

**Check order.** Each step runs these checks in order. The first one that fails wins, and every failure gives INVALID with roll 0 and QUALITY 0 (D-065 item 1, safety-reviewer MAJOR-1 on defs v0.9.0):

1. A NULL argument gives `ARG`.
2. A gyro sample that is non-finite or has |rate| ≥ 2000°/s gives `INPUT`. The step is skipped and the state is left untouched. The last stamp is kept, so the next step sees the gap.
3. The gap since the last sample is checked next. The first step, a gap of 0 or more than `LEAN_DT_MAX_MS` (40 ms), or a backward stamp gives `DT` and reinitialises the filter.
4. Predict.
5. Filter health: the state must be finite, both variances positive, P positive semi-definite and |bias| ≤ 5°/s. A failure gives `FILTER` and reinitialises.
6. With a usable speed, the measurement update runs behind the innovation gate. A gate run that lasts too long gives `GATE` and reinitialises (see below). Filter health is then checked again.
7. A roll σ above 5° gives `VARIANCE`. This includes a filter that has not converged yet: P0 is (30°)².
8. A missing usable speed gives `SPEED`. Predict still runs, so the variance keeps growing.
9. |roll| ≥ 90° gives `ROLL_RANGE`, because h = sin is not monotone beyond that.

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

**Availability cost (fail-safe, to measure on T1/T4).** Each of these makes the lean INVALID until the variance has converged again:
- an IMU gap longer than 40 ms mid-corner;
- a gate run longer than 100 ms;
- an implausible state.

Convergence after a reinit takes a few steps at 20 ms (test: within 1 s). The bias needs tens of seconds of riding (tests: 120 s).

## Memory and timing

- **RAM:** `lean_core_t` is 32 B, owned by the caller (static in the PR 3 glue). There is no heap and no loop. Each step does a fixed number of float operations: one `sinf`, one `cosf` and one `sqrtf`, plus two divisions, each by a value checked to be positive (MISRA Dir 4.1, Dir 4.11).
- **Flash** (M7 release, `arm-none-eabi-size`, 2026-10-10): `lean_core.c.obj` has text 1176 B and bss 0. Total `moto_rtcore_logic` + `moto_rtcore_fw` text: 12370 → 13546 B. The libm functions (`sinf`, `cosf`; `sqrtf` becomes VSQRT with `-fno-math-errno`) are linked with the firmware in PR 3.
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
- **dt:** 40 ms predicts; 0 ms, 41 ms and a backward stamp each give DT and a reinit.
- **Gate:**
  - a diverged filter (40° while riding straight) stays ESTIMATED with QUALITY ≤ 50 up to 100 ms after the first reject, gives GATE at the next reject, reinitialises and reconverges;
  - a single outlier caps QUALITY at 50, and the next accepted update ends the run;
  - a reject leaves a QUALITY already below the cap unchanged;
  - steps without a speed hold the run.
- **Filter health:** NaN or Inf in roll, bias, p00, p01 or p11; negative p00 or p11; P not positive semi-definite; |bias| above the limit; an update that pushes the bias out of range. Each gives FILTER and a reinit.
- **Variance and range:** without a speed the reason moves from SPEED to VARIANCE once σ > 5°. 90° and −120° give ROLL_RANGE, 80° is estimated. 60° is estimated unclamped, and changing the output copy leaves the filter unchanged.
- **QUALITY map:** the boundaries (1°, just above 1°, 3°, 5°, just above 5°), negative values, NaN and Inf, and monotonicity.
- **NULL arguments.**

## HIL scenarios (with the board and PR 3, moto-hil-bench)

- `lean_steady_turn`: replay a coordinated turn (IMU and speed restbus) at 30° and 15 m/s. Expected: 0x020 ESTIMATED within 1 s, |error| ≤ 1°, QUALITY ≥ 90.
- `lean_speed_stall`: freeze the restbus speed during a turn. Expected: 0x020 INVALID with LEAN 0 and QUALITY 0 within 300 ms + one 0x020 cycle. NODE_MODE is unchanged, because the EKF still runs.
- `lean_imu_gap`: drop IMU samples for 60 ms. Expected: INVALID at once, then ESTIMATED again after reconvergence (≤ 1 s).
