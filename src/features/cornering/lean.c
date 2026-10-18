#include "features/cornering/lean.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the boot setup (host program, H7) and the tests
void lean_link_init(lean_link_t* link)
{
    if (link == NULL) {
        return;
    }
    (void)memset(link, 0, sizeof *link); /* UNUSABLE speeds, INVALID records, stamp 0 */
    snapshot_init(&link->speed_snap);
    snapshot_init(&link->out_snap);
    alive_init(&link->alive);
}

const alive_counter_t* lean_link_alive(const lean_link_t* link)
{
    return (link == NULL) ? NULL : &link->alive;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the boot setup (host program, H7) and the tests
void lean_ekf_init(lean_ekf_t* e, lean_link_t* link)
{
    if (e == NULL) {
        return;
    }
    (void)memset(e, 0, sizeof *e);
    lean_core_init(&e->core);
    imu_init(&e->imu);
    e->link = link;
}

/* A VALID, recent speed that reads 0 km/h. */
static bool reads_zero(const lean_speed_t* s, uint32_t now_ms)
{
    if ((s->state != LEAN_SPEED_VALID) || !(fabsf(s->speed_mps) < LEAN_STANDSTILL_MPS)) {
        return false; /* NaN fails the comparison */
    }
    /* the core judges a speed's age against the IMU stamp; that stamp is this step's now
     * (services/imu), so the same future limit applies here */
    if ((s->stamp_ms - now_ms) <= LEAN_SPEED_FUTURE_MAX_MS) {
        return true; /* slightly ahead of now: age 0, never a wrapped age */
    }
    return (now_ms - s->stamp_ms) <= LEAN_STANDSTILL_AGE_MAX_MS;
}

/* Standstill: this sample and the one before it read 0, and this one is recent. */
static bool standstill(lean_ekf_t* e, uint32_t now_ms)
{
    if (e->speed.stamp_ms != e->sample_stamp_ms) {
        e->prior_zero = e->sample_zero;
        e->sample_zero = (e->speed.state == LEAN_SPEED_VALID) && (fabsf(e->speed.speed_mps) < LEAN_STANDSTILL_MPS);
        e->sample_stamp_ms = e->speed.stamp_ms;
    }
    return e->prior_zero && reads_zero(&e->speed, now_ms);
}

static lean_out_t imu_invalid_out(void)
{
    lean_out_t out;
    out.roll_deg = 0.0f;
    out.quality = 0u;
    out.state = LEAN_OUT_INVALID;
    out.reason = LEAN_REASON_INPUT;
    return out;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the EKF task (host program, H7) and the tests
void lean_ekf_step(lean_ekf_t* e, uint32_t now_ms)
{
    if ((e == NULL) || (e->link == NULL)) {
        return;
    }
    lean_link_t* const link = e->link;

    uint8_t si = 0u;
    (void)snapshot_read_index(&link->speed_snap, &si);
    e->speed = link->speed_slots[si];

    const imu_sample_t imu = imu_read(&e->imu, now_ms, standstill(e, now_ms));
    e->last_imu = imu;
    if (imu.reason == IMU_REASON_NO_NEW) {
        return; /* not a cycle: nothing new to estimate (safety-reviewer carry-over on 3a) */
    }
    lean_out_t out;
    if (imu.state == IMU_SAMPLE_VALID) {
        const lean_imu_t li = {imu.gyro_x_rps, imu.gyro_z_rps, imu.stamp_ms};
        out = lean_core_step(&e->core, &li, &e->speed);
    } else {
        out = imu_invalid_out();
    }
    e->last = out;

    /* After a producer fault the write slot may be the comms task's front (the latching
     * publish already stored it in the corrupted word), so it is never written again: the
     * last record ages out to INVALID on the comms side (safety-reviewer MINOR-1 on 3b). */
    if (!snapshot_producer_fault(&link->out_snap)) {
        const uint8_t wi = snapshot_write_index(&link->out_snap);
        link->out_slots[wi].out = out;
        link->out_slots[wi].stamp_ms = imu.stamp_ms;
        link->out_slots[wi].fault = snapshot_consumer_fault(&link->speed_snap);
        /* the 0xFD03 fields, diagnostic only (D-067); counters saturate, never wrap */
        const imu_offset_health_t ih = imu_offset_health(&e->imu);
        lean_health_ekf_t* const h = &link->out_slots[wi].health;
        h->offset_learned = ih.offset_learned;
        h->offset_rejected = ih.offset_rejected;
        h->reject_count = lean_health_sat16(ih.reject_count, PLATFORM_UDS_LEAN_HEALTH_OFFSET_REJECTS_MAX);
        h->unlearned_s = lean_health_unlearned_s(&e->unlearned_run, ih.offset_learned, now_ms);
        snapshot_publish(&link->out_snap);
    }

    alive_bump(&link->alive); /* last: only a complete cycle counts (E-17) */
}
