#include "features/cornering/lean_tx_core.h"

#include <math.h>
#include <stddef.h>

_Static_assert(LEAN_TX_AGE_MAX_MS < PLATFORM_EKF_LEAN_E2E_TIMEOUT_MS,
               "the 0x020 output age budget must stay below the receivers' timeout");
/* The clamp limit lies inside the model bound: defs codegen checks it (v0.9.0: below the
 * LEAN_ANGLE maximum of 90 deg, D-065 implementation note); a float cannot be a C11
 * constant expression here. */

/* The model bound in degrees: lean_core never estimates at or beyond it (ROLL_RANGE). */
#define LEAN_TX_ROLL_BOUND_DEG (LEAN_ROLL_MAX_RAD * LEAN_DEG_PER_RAD)

static void set_invalid(struct platform_ekf_lean_t* msg)
{
    msg->lean_angle_state = (uint8_t)PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_INVALID_CHOICE;
    msg->lean_angle = 0;
    msg->lean_angle_quality = 0u;
}

static bool age_ok(uint32_t stamp_ms, uint32_t now_ms)
{
    if ((stamp_ms - now_ms) <= LEAN_TX_FUTURE_MAX_MS) {
        return true; /* now or slightly ahead: age 0, never a wrapped age */
    }
    return (now_ms - stamp_ms) <= LEAN_TX_AGE_MAX_MS;
}

/* Rule 3: an allow-list switch, never a cast (the core's values differ from the wire). */
static bool estimated(lean_out_state_t state)
{
    bool ok = false;
    switch (state) {
    case LEAN_OUT_ESTIMATED:
        ok = true;
        break;
    case LEAN_OUT_INVALID:
    default:
        ok = false;
        break;
    }
    return ok;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the comms-side glue (lean_tx.c) and the tests
void lean_tx_msg(const lean_record_t* rec, bool comms_fault, uint32_t now_ms,
                 struct platform_ekf_lean_t* msg)
{
    if (msg == NULL) {
        return;
    }
    (void)platform_ekf_lean_init(msg);
    set_invalid(msg);
    if ((rec == NULL) || comms_fault || rec->fault || !age_ok(rec->stamp_ms, now_ms) ||
        !estimated(rec->out.state)) {
        return;
    }
    float roll = rec->out.roll_deg;
    const uint8_t q = rec->out.quality;
    /* NaN fails the comparison, Inf exceeds the bound (MISRA Dir 4.1) */
    if (!(fabsf(roll) < LEAN_TX_ROLL_BOUND_DEG) || (q < LEAN_QUALITY_MIN) ||
        !platform_ekf_lean_lean_angle_quality_is_in_phys_range((float)q)) {
        return;
    }
    uint8_t state = (uint8_t)PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_ESTIMATED_CHOICE;
    if (fabsf(roll) > PLATFORM_LIMIT_LEAN_ANGLE_CLAMP_MAX_DEG) {
        roll = (roll > 0.0f) ? PLATFORM_LIMIT_LEAN_ANGLE_CLAMP_MAX_DEG
                             : -PLATFORM_LIMIT_LEAN_ANGLE_CLAMP_MAX_DEG;
        state = (uint8_t)PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_CLAMPED_CHOICE;
    }
    msg->lean_angle_state = state;
    msg->lean_angle = platform_ekf_lean_lean_angle_encode(roll);
    msg->lean_angle_quality = platform_ekf_lean_lean_angle_quality_encode((float)q);
}

bool lean_tx_frame(const struct platform_ekf_lean_t* msg, const vehicle_e2e_tx_state_t* committed,
                   vehicle_e2e_tx_state_t* next, uint8_t out[PLATFORM_EKF_LEAN_LENGTH])
{
    if ((msg == NULL) || (committed == NULL) || (next == NULL) || (out == NULL)) {
        return false;
    }
    if (platform_ekf_lean_pack(out, msg, PLATFORM_EKF_LEAN_LENGTH) != (int)PLATFORM_EKF_LEAN_LENGTH) {
        return false;
    }
    *next = *committed;
    return platform_ekf_lean_e2e_protect(out, PLATFORM_EKF_LEAN_LENGTH, next) == VEHICLE_E2E_OK;
}
