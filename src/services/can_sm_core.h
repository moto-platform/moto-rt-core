#ifndef SERVICES_CAN_SM_CORE_H
#define SERVICES_CAN_SM_CORE_H

/*
 * CAN state manager core (Ç1, ≈ AUTOSAR CanSM as a concept, D-006): the error state of
 * one CAN controller, bus-off recovery with backoff, the bus-off latch (D-030, D-054) and
 * the TX confirmation timeout (ISO 15765-2 N_As). Pure logic: no HAL, no RTOS, no heap.
 * The glue (services/can_sm) feeds it a controller snapshot every step and carries out
 * the actions it returns.
 *
 * States (ISO 11898-1 fault confinement, plus the latch):
 *   ERROR_ACTIVE   TEC and REC <= 127
 *   ERROR_PASSIVE  TEC or REC > 127 (the controller still sends)
 *   BUS_OFF        TX and RX stopped; recovery is requested after the backoff
 *   LATCHED        cfg.latch_after bus-off events seen: no recovery until reboot
 * TX is allowed in ERROR_ACTIVE and ERROR_PASSIVE only.
 *
 * Bus-off: every new bus-off event (the controller's wrapping counter moved, or the
 * bus-off flag with no event counted yet) aborts the pending TX, so no stale frame goes
 * out after a recovery, and is counted. An event counted from the flag is a credit: when
 * the ISR counter catches up, that one is not counted again (one physical bus-off, one
 * count). A recovery is only started with no frame pending; otherwise the abort is
 * repeated and the recovery waits for the next step. Recovery waits the backoff, starting at
 * cfg.backoff_initial_ms and doubling after every attempt up to cfg.backoff_max_ms; it is
 * back to the initial value once the port stayed out of bus-off for cfg.backoff_max_ms.
 * The count is kept since init (RAM, D-054; surviving a reset needs no-init RAM on the
 * H7, a follow-up once the board exists, Q-019).
 *
 * N_As: while frames are pending and the controller's TX-done counter does not move for
 * cfg.tx_timeout_ms, the pending TX is aborted (ISOTP links then end their message with
 * N_TIMEOUT_A). Every abort, bus-off or N_As, bumps abort_seq.
 *
 * All times are wrap-safe (unsigned subtraction); every counter saturates, except
 * abort_seq, which wraps and is only compared for change.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CAN_SM_ERROR_ACTIVE = 0,
    CAN_SM_ERROR_PASSIVE,
    CAN_SM_BUS_OFF,
    CAN_SM_LATCHED
} can_sm_state_t;

typedef struct {
    uint32_t backoff_initial_ms; /* > 0 */
    uint32_t backoff_max_ms;     /* >= backoff_initial_ms */
    uint32_t tx_timeout_ms;      /* N_As, > 0 */
    uint32_t latch_after;        /* bus-off events that latch the port; 0 = never */
} can_sm_config_t;

/* Controller snapshot (same meaning as hal/can_types.h can_port_state_t). */
typedef struct {
    uint8_t tec;
    uint8_t rec;
    bool error_passive;
    bool bus_off;
    uint32_t bus_off_events; /* wraps */
    uint32_t tx_pending;
    uint32_t tx_done;        /* wraps */
} can_sm_input_t;

/* What the glue must do after a step. */
typedef struct {
    bool abort_tx; /* cancel every pending TX frame */
    bool recover;  /* start the bus-off recovery */
} can_sm_actions_t;

typedef struct {
    uint32_t bus_off_events;   /* since init */
    uint32_t recover_attempts;
    uint32_t recover_deferred; /* steps a due recovery waited for a pending frame's abort */
    uint32_t tx_timeouts;      /* N_As aborts */
    uint8_t tec_max;
    uint8_t rec_max;
} can_sm_stats_t;

/* Treat as opaque. */
typedef struct {
    can_sm_config_t cfg;
    can_sm_state_t state;
    bool primed;              /* first snapshot taken */
    bool flag_credit;         /* a bus-off counted from the flag before the ISR counter moved */
    uint32_t seen_bus_off_events;
    uint32_t seen_tx_done;
    bool tx_timer_running;
    uint32_t tx_timer_ms;     /* start of the running N_As period */
    uint32_t backoff_ms;      /* wait before the next recovery attempt */
    uint32_t backoff_start_ms;
    uint32_t bus_on_since_ms; /* last time the port left bus-off */
    bool bus_on_timing;       /* bus_on_since_ms is valid and the backoff not reset yet */
    uint32_t abort_seq;
    can_sm_stats_t stats;
} can_sm_core_t;

/* False if cfg is invalid (the core then stays LATCHED: fail-closed). */
bool can_sm_core_init(can_sm_core_t* sm, const can_sm_config_t* cfg);

/* One step with the controller snapshot; fills *act. */
void can_sm_core_step(can_sm_core_t* sm, uint32_t now_ms, const can_sm_input_t* in,
                      can_sm_actions_t* act);

can_sm_state_t can_sm_core_state(const can_sm_core_t* sm);

/* Requests a TX abort outside a step (the glue could not read the controller state):
 * bumps abort_seq. */
void can_sm_core_force_abort(can_sm_core_t* sm);

/* True in ERROR_ACTIVE and ERROR_PASSIVE. */
bool can_sm_core_tx_allowed(const can_sm_core_t* sm);

/* Changes on every TX abort (wraps). */
uint32_t can_sm_core_abort_seq(const can_sm_core_t* sm);

const can_sm_stats_t* can_sm_core_stats(const can_sm_core_t* sm);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_CAN_SM_CORE_H */
