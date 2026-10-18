#ifndef FEATURES_CORNERING_LEAN_H
#define FEATURES_CORNERING_LEAN_H

/*
 * Lean (roll) estimation glue, EKF-task side (D-065 item 2, PR 3b): the module's public
 * header together with lean_tx.h (comms-task side). Other code sees lean_core.h only
 * through these.
 *
 * Tasks and data (architecture-guard MINOR-4 on the PR 3 plan):
 *   lean_link_t   shared, boot-initialised before the scheduler; carries
 *                   speed   comms task -> EKF task  (services/snapshot, lean_speed_t)
 *                   output  EKF task -> comms task  (services/snapshot, lean_record_t)
 *                   alive   bumped by the EKF task, supervised by app/comms
 *   lean_ekf_*    EKF task only (this header)
 *   lean_tx_*     comms task only (lean_tx.h)
 *
 * The EKF task runs lean_ekf_step() every PLATFORM_EKF_LEAN_CYCLE_TIME_MS (20 ms) at the
 * highest task priority (README, task priority table). It holds no lock, prints nothing
 * and never allocates; it touches only services/imu, its own snapshot ends, the alive
 * counter and the timebase passed in.
 */

#include "features/cornering/lean_core.h"
#include "features/cornering/lean_tx_core.h"
#include "services/alive.h"
#include "services/imu.h"
#include "services/lean_health_core.h"
#include "services/snapshot.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Standstill for the z offset learning (services/imu; architecture-guard MINOR-6,
 * safety-reviewer carry-over on PR 3a). Provisional until T1:
 * - below half a km/h step of the 0xF40D speed (0.139 m/s), i.e. a reading of 0 km/h,
 *   compared with a tolerance, never float equality;
 * - a sample at most LEAN_STANDSTILL_AGE_MAX_MS old, shorter than the 300 ms usability
 *   limit, so the window ends soon after the bike moves off;
 * - the previous speed sample read 0 as well, so one stale or glitched 0 is not enough. */
#define LEAN_STANDSTILL_MPS (0.1f)
#define LEAN_STANDSTILL_AGE_MAX_MS (200u)

typedef struct {
    snapshot_t speed_snap;                     /* producer: comms task; consumer: EKF task */
    lean_speed_t speed_slots[SNAPSHOT_SLOTS];
    snapshot_t out_snap;                       /* producer: EKF task; consumer: comms task */
    lean_record_t out_slots[SNAPSHOT_SLOTS];
    alive_counter_t alive;                     /* writer: EKF task */
} lean_link_t;

/* EKF task state (static, owned by the EKF setup). */
typedef struct {
    lean_core_t core;
    imu_t imu;
    lean_link_t* link;
    lean_speed_t speed;         /* the speed record read in the last step */
    uint32_t sample_stamp_ms;   /* stamp of the newest speed sample seen */
    bool sample_zero;           /* that sample read 0 */
    bool prior_zero;            /* the sample before it read 0 */
    /* EKF task only, like the rest of lean_ekf_t: another task (a UDS DID, a diagnostic)
     * reads the output through the link's snapshot, never these. */
    lean_health_run_t unlearned_run; /* unlearned-run duration for 0xFD03 (D-067) */
    lean_out_t last;            /* the last output (tests, diagnostics) */
    imu_sample_t last_imu;      /* the last IMU sample (tests, diagnostics) */
} lean_ekf_t;

/* Boot only, before either task runs: every slot INVALID (speed UNUSABLE with stamp 0,
 * output zero record), the snapshots and the alive counter initialised. */
void lean_link_init(lean_link_t* link);

/* The link's alive counter, for comms_ekf_register() before the scheduler. */
const alive_counter_t* lean_link_alive(const lean_link_t* link);

/* Boot only: filter, IMU service and standstill state reset; binds the link. */
void lean_ekf_init(lean_ekf_t* e, lean_link_t* link);

/*
 * EKF task, once per cycle. Reads the newest speed, drains the IMU (services/imu, with the
 * standstill above), steps the core only for a new VALID IMU sample (an INVALID one gives
 * an INVALID output with reason INPUT and no core step, so a gap reaches the core's own
 * dt check), publishes {output, IMU stamp, EKF-side snapshot fault, IMU offset health for
 * DID 0xFD03: learned / rejected, reject count, unlearned seconds} (never again once the
 * output snapshot's producer fault is latched: the record then ages out) and, at the very
 * end of the complete cycle, bumps the alive counter. A call in the same ms as the last
 * one (IMU NO_NEW) does nothing: no output, no bump. NULL or an unbound link does nothing.
 */
void lean_ekf_step(lean_ekf_t* e, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* FEATURES_CORNERING_LEAN_H */
