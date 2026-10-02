#include "services/can_sm.h"

#include "hal/can_port.h"
#include "services/timebase.h"
#include "vehicle_cl250.h"

#include <stddef.h>

_Static_assert((VEHICLE_CL250_BUS_OFF_BACKOFF_INITIAL_MS > 0u) &&
                   (VEHICLE_CL250_BUS_OFF_BACKOFF_MAX_MS >= VEHICLE_CL250_BUS_OFF_BACKOFF_INITIAL_MS),
               "invalid vehicle bus-off backoff");
_Static_assert((CAN_SM_PLATFORM_BACKOFF_INITIAL_MS > 0u) &&
                   (CAN_SM_PLATFORM_BACKOFF_MAX_MS >= CAN_SM_PLATFORM_BACKOFF_INITIAL_MS),
               "invalid platform bus-off backoff");
_Static_assert((CAN_SM_TX_TIMEOUT_MS > 0u) && (CAN_SM_VEHICLE_BUS_OFF_LATCH > 0u),
               "invalid N_As or latch count");

static can_sm_core_t cores[CAN_PORT_COUNT];
/* The port's controller state could not be read at its last step: TX refused until a
 * good snapshot (fail-closed). */
static bool state_unknown[CAN_PORT_COUNT];
/* The abort for an unknown state was accepted by the port (else it is retried). */
static bool unknown_abort_done[CAN_PORT_COUNT];
static bool initialised;

static bool sm_port_ok(can_port_id_t port)
{
    return (uint32_t)port < (uint32_t)CAN_PORT_COUNT;
}

void can_sm_init(void)
{
    const can_sm_config_t vehicle = {VEHICLE_CL250_BUS_OFF_BACKOFF_INITIAL_MS,
                                     VEHICLE_CL250_BUS_OFF_BACKOFF_MAX_MS, CAN_SM_TX_TIMEOUT_MS,
                                     CAN_SM_VEHICLE_BUS_OFF_LATCH};
    const can_sm_config_t platform = {CAN_SM_PLATFORM_BACKOFF_INITIAL_MS,
                                      CAN_SM_PLATFORM_BACKOFF_MAX_MS, CAN_SM_TX_TIMEOUT_MS, 0u};
    (void)can_sm_core_init(&cores[CAN_PORT_VEHICLE], &vehicle);
    (void)can_sm_core_init(&cores[CAN_PORT_PLATFORM], &platform);
    state_unknown[CAN_PORT_VEHICLE] = false;
    state_unknown[CAN_PORT_PLATFORM] = false;
    unknown_abort_done[CAN_PORT_VEHICLE] = false;
    unknown_abort_done[CAN_PORT_PLATFORM] = false;
    initialised = true;
}

void can_sm_step(can_port_id_t port)
{
    can_port_state_t st;
    if (!initialised || !sm_port_ok(port)) {
        return;
    }
    if (can_port_get_state(port, &st) != CAN_PORT_OK) {
        if (!state_unknown[port]) {
            state_unknown[port] = true; /* no supervision: refuse TX, drop what waits */
            unknown_abort_done[port] = false;
            can_sm_core_force_abort(&cores[port]);
        }
        if (!unknown_abort_done[port]) {
            unknown_abort_done[port] = (can_port_tx_abort(port) == CAN_PORT_OK); /* retried */
        }
        return;
    }
    state_unknown[port] = false;
    const can_sm_input_t in = {st.tec,          st.rec,       st.error_passive, st.bus_off,
                               st.bus_off_events, st.tx_pending, st.tx_done};
    can_sm_actions_t act;
    can_sm_core_step(&cores[port], timebase_now_ms(), &in, &act);
    if (act.abort_tx) {
        (void)can_port_tx_abort(port);
    }
    if (act.recover) {
        (void)can_port_recover(port);
    }
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: diagnostics, host summary and the health DID
can_sm_state_t can_sm_state(can_port_id_t port)
{
    return (initialised && sm_port_ok(port)) ? can_sm_core_state(&cores[port]) : CAN_SM_LATCHED;
}

bool can_sm_tx_allowed(can_port_id_t port)
{
    return initialised && sm_port_ok(port) && !state_unknown[port] &&
           can_sm_core_tx_allowed(&cores[port]); /* fail-closed */
}

uint32_t can_sm_abort_seq(can_port_id_t port)
{
    return sm_port_ok(port) ? can_sm_core_abort_seq(&cores[port]) : 0u;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: diagnostics, host summary and the health DID
const can_sm_stats_t* can_sm_stats(can_port_id_t port)
{
    return sm_port_ok(port) ? can_sm_core_stats(&cores[port]) : NULL;
}
