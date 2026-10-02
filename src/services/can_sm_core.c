#include "services/can_sm_core.h"

#include <stddef.h>

/* ISO 11898-1: a controller with TEC or REC above this is error passive. */
#define ERROR_PASSIVE_LIMIT 127u

static uint32_t sat_add(uint32_t a, uint32_t b)
{
    return (a > (UINT32_MAX - b)) ? UINT32_MAX : (a + b);
}

/* Wrap-safe: true once at least period_ms have passed since start_ms. */
static bool period_over(uint32_t now_ms, uint32_t start_ms, uint32_t period_ms)
{
    return (uint32_t)(now_ms - start_ms) >= period_ms;
}

static void abort_tx(can_sm_core_t* sm, can_sm_actions_t* act)
{
    act->abort_tx = true;
    sm->abort_seq++; /* wraps: readers only compare it for change */
    sm->tx_timer_running = false;
}

static uint32_t next_backoff(const can_sm_core_t* sm)
{
    const uint32_t half_max = sm->cfg.backoff_max_ms / 2u;
    return (sm->backoff_ms > half_max) ? sm->cfg.backoff_max_ms : (sm->backoff_ms * 2u);
}

bool can_sm_core_init(can_sm_core_t* sm, const can_sm_config_t* cfg)
{
    if (sm == NULL) {
        return false;
    }
    const bool ok = (cfg != NULL) && (cfg->backoff_initial_ms > 0u) &&
                    (cfg->backoff_max_ms >= cfg->backoff_initial_ms) && (cfg->tx_timeout_ms > 0u);
    if (ok) {
        sm->cfg = *cfg;
    } else {
        sm->cfg.backoff_initial_ms = 1u;
        sm->cfg.backoff_max_ms = 1u;
        sm->cfg.tx_timeout_ms = 1u;
        sm->cfg.latch_after = 1u;
    }
    sm->state = ok ? CAN_SM_ERROR_ACTIVE : CAN_SM_LATCHED; /* fail-closed */
    sm->primed = false;
    sm->flag_credit = false;
    sm->seen_bus_off_events = 0u;
    sm->seen_tx_done = 0u;
    sm->tx_timer_running = false;
    sm->tx_timer_ms = 0u;
    sm->backoff_ms = sm->cfg.backoff_initial_ms;
    sm->backoff_start_ms = 0u;
    sm->bus_on_since_ms = 0u;
    sm->bus_on_timing = false;
    sm->abort_seq = 0u;
    sm->stats.bus_off_events = 0u;
    sm->stats.recover_attempts = 0u;
    sm->stats.recover_deferred = 0u;
    sm->stats.tx_timeouts = 0u;
    sm->stats.tec_max = 0u;
    sm->stats.rec_max = 0u;
    return ok;
}

/* A new bus-off: abort, count, then latch or wait for the backoff. */
static void on_bus_off(can_sm_core_t* sm, uint32_t now_ms, uint32_t events, can_sm_actions_t* act)
{
    sm->stats.bus_off_events = sat_add(sm->stats.bus_off_events, events);
    abort_tx(sm, act);
    sm->bus_on_timing = false;
    if ((sm->cfg.latch_after > 0u) && (sm->stats.bus_off_events >= sm->cfg.latch_after)) {
        sm->state = CAN_SM_LATCHED; /* D-030, D-054: no recovery until reboot */
    } else if (sm->state != CAN_SM_BUS_OFF) {
        sm->state = CAN_SM_BUS_OFF;
        sm->backoff_start_ms = now_ms;
    } else {
        /* back in bus-off before a step saw the bus on: the attempt's timer runs on */
    }
}

/* Bus-off and the backoff wait. True while the port stays bus-off. */
static bool bus_off_step(can_sm_core_t* sm, uint32_t now_ms, const can_sm_input_t* in,
                         can_sm_actions_t* act)
{
    if (!in->bus_off) {
        sm->bus_on_since_ms = now_ms;
        sm->bus_on_timing = true;
        sm->flag_credit = false; /* the ISR had a whole bus-off to count it */
        return false;
    }
    if (in->tx_pending > 0u) {
        abort_tx(sm, act); /* the last abort did not take: never rejoin with a stale frame */
        if (period_over(now_ms, sm->backoff_start_ms, sm->backoff_ms)) {
            sm->stats.recover_deferred = sat_add(sm->stats.recover_deferred, 1u);
        }
    } else if (period_over(now_ms, sm->backoff_start_ms, sm->backoff_ms)) {
        act->recover = true;
        sm->flag_credit = false; /* after this an extra count is the fail-safe direction */
        sm->stats.recover_attempts = sat_add(sm->stats.recover_attempts, 1u);
        sm->backoff_start_ms = now_ms;
        sm->backoff_ms = next_backoff(sm);
    } else {
        /* waiting for the backoff */
    }
    return true;
}

/* N_As: pending frames and no TX progress for tx_timeout_ms. */
static void tx_timeout_step(can_sm_core_t* sm, uint32_t now_ms, const can_sm_input_t* in,
                            bool progress, can_sm_actions_t* act)
{
    if (in->tx_pending == 0u) {
        sm->tx_timer_running = false;
    } else if (!sm->tx_timer_running || progress) {
        sm->tx_timer_running = true;
        sm->tx_timer_ms = now_ms;
    } else if (period_over(now_ms, sm->tx_timer_ms, sm->cfg.tx_timeout_ms)) {
        sm->stats.tx_timeouts = sat_add(sm->stats.tx_timeouts, 1u);
        abort_tx(sm, act);
    } else {
        /* still waiting for the confirmation */
    }
}

void can_sm_core_step(can_sm_core_t* sm, uint32_t now_ms, const can_sm_input_t* in,
                      can_sm_actions_t* act)
{
    if (act == NULL) {
        return;
    }
    act->abort_tx = false;
    act->recover = false;
    if ((sm == NULL) || (in == NULL)) {
        return;
    }
    if (in->tec > sm->stats.tec_max) {
        sm->stats.tec_max = in->tec;
    }
    if (in->rec > sm->stats.rec_max) {
        sm->stats.rec_max = in->rec;
    }
    if (!sm->primed) {
        sm->primed = true; /* events before the first step are not ours to count */
        sm->seen_bus_off_events = in->bus_off_events;
        sm->seen_tx_done = in->tx_done;
    }
    const uint32_t events = in->bus_off_events - sm->seen_bus_off_events; /* wraps */
    sm->seen_bus_off_events = in->bus_off_events;
    const bool progress = in->tx_done != sm->seen_tx_done;
    sm->seen_tx_done = in->tx_done;

    if (sm->state == CAN_SM_LATCHED) {
        if (in->tx_pending > 0u) {
            abort_tx(sm, act); /* fail-closed: nothing may wait to go out */
        }
        return;
    }
    uint32_t fresh = events;
    if (sm->flag_credit && (fresh > 0u)) {
        sm->flag_credit = false;
        fresh--; /* the ISR counted the bus-off already taken from the flag */
    }
    if (fresh > 0u) {
        on_bus_off(sm, now_ms, fresh, act);
    } else if (in->bus_off && (sm->state != CAN_SM_BUS_OFF)) {
        sm->flag_credit = true;
        on_bus_off(sm, now_ms, 1u, act); /* bus-off flag before the ISR counter moved */
    } else {
        /* no new bus-off */
    }
    if (sm->state == CAN_SM_LATCHED) {
        return;
    }
    if ((sm->state == CAN_SM_BUS_OFF) && bus_off_step(sm, now_ms, in, act)) {
        return;
    }
    const bool passive = in->error_passive || (in->tec > ERROR_PASSIVE_LIMIT) ||
                         (in->rec > ERROR_PASSIVE_LIMIT);
    sm->state = passive ? CAN_SM_ERROR_PASSIVE : CAN_SM_ERROR_ACTIVE;
    if (sm->bus_on_timing && period_over(now_ms, sm->bus_on_since_ms, sm->cfg.backoff_max_ms)) {
        sm->bus_on_timing = false;
        sm->backoff_ms = sm->cfg.backoff_initial_ms; /* stable again: next bus-off starts over */
    }
    tx_timeout_step(sm, now_ms, in, progress, act);
}

void can_sm_core_force_abort(can_sm_core_t* sm)
{
    if (sm != NULL) {
        sm->abort_seq++;
        sm->tx_timer_running = false;
    }
}

can_sm_state_t can_sm_core_state(const can_sm_core_t* sm)
{
    return (sm != NULL) ? sm->state : CAN_SM_LATCHED;
}

bool can_sm_core_tx_allowed(const can_sm_core_t* sm)
{
    return (sm != NULL) &&
           ((sm->state == CAN_SM_ERROR_ACTIVE) || (sm->state == CAN_SM_ERROR_PASSIVE));
}

uint32_t can_sm_core_abort_seq(const can_sm_core_t* sm)
{
    return (sm != NULL) ? sm->abort_seq : 0u;
}

const can_sm_stats_t* can_sm_core_stats(const can_sm_core_t* sm)
{
    return (sm != NULL) ? &sm->stats : NULL;
}
