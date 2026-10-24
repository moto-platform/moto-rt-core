#ifndef DTC_SNAPSHOT_CORE_H
#define DTC_SNAPSHOT_CORE_H

/*
 * D-080: the DTC freeze-frame reader of the vehicle tester (ReadDTCInformation 0x19,
 * ISO 14229-1 clause 12.3) as a pure state machine. It never transmits: uds_client_core
 * asks it for the next request, sends that through the D-020 gates and feeds the
 * answer back. It is the C port of moto-connectivity-node's DtcSnapshot (D-023), without
 * the text sink: the raw answers go to the caller (uds_client_core_take_dtc_answer()).
 *
 * A run (D-080 item 2): 0x19 0x01 <mask> (count); only if the count is > 0, 0x19 0x02
 * <mask> (list); then per listed DTC whose status matches the mask, at most
 * VEHICLE_CL250_DTC_MAX_PER_RUN, 0x19 0x04 <DTC> <record> and 0x19 0x06 <DTC> <record>.
 * Sub-functions, mask, records and the cap come from gen/ only. At most
 * DTC_SNAPSHOT_MAX_REQUESTS_PER_RUN requests per run (a re-send after a session refusal
 * counts) and DTC_SNAPSHOT_RUNS_PER_EPOCH runs per stop epoch (D-075): the key-on run at the
 * epoch start and one warm-idle run (D-080 item 1).
 *
 * The three DTC bytes of a 0x04/0x06 request are ECU data: a DTC is stored only from a
 * 0x02 answer whose length matched the record layout, so a malformed list never produces
 * a request. The 0x19 sub-functions the tester sends are the four of gen/ (never 0x14).
 *
 * Why 0x19 is off for the rest of the epoch (halt), D-080 items 2 and 3 and the conn
 * implementation note:
 *   NOT_SERVED       NRC 0x11 / 0x12 / 0x31
 *   SEGMENTED        a refused (FF_DL above the cap, wrong echo), withheld or failed
 *                    segmented reception
 *   SESSION_REFUSED  the second NRC 0x7E / 0x7F in the epoch
 *   NO_ANSWER        a timeout (a late answer may still come as a First Frame)
 *   SLOW             a transaction longer than one unverified-read slot
 *   LATE_FF          a First Frame without a pending request after a 0x19 timeout
 * Any other NRC or a malformed answer only ends the run.
 *
 * Pure logic: no HAL, services or RTOS; time is passed in (ms, wrap-safe); static
 * storage, no heap, no recursion, bounded loops.
 */

#include "vehicle_cl250.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DTC_SNAPSHOT_DTC_BYTES 3u /* ISO 14229-1 clause 11.3: DTC high, middle, low byte */
/* 19 04|06 <DTC 3 bytes> <record>: the longest request. */
#define DTC_SNAPSHOT_REQ_MAX 6u
/* The first bytes of a positive answer the FC check looks at: 59 <sub-function> <DTC 3>. */
#define DTC_SNAPSHOT_PREFIX_MAX 5u
#define DTC_SNAPSHOT_RUNS_PER_EPOCH 2u /* key-on and one warm-idle run (D-080 item 1) */
/* Count + list + (snapshot + extended data) per DTC (D-080 item 2). */
#define DTC_SNAPSHOT_MAX_REQUESTS_PER_RUN (2u + (2u * VEHICLE_CL250_DTC_MAX_PER_RUN))
#define DTC_SNAPSHOT_SESSION_REFUSALS_MAX 2u /* the second NRC 0x7E/0x7F halts 0x19 */
/* ISO 14229-1 clause 12.3.2 response layouts: standard values, not vehicle signals. */
#define DTC_SNAPSHOT_COUNT_ANSWER_LEN 6u  /* 59 01 <availability> <format> <count hi> <count lo> */
#define DTC_SNAPSHOT_LIST_HEADER_LEN 3u   /* 59 02 <availability> */
#define DTC_SNAPSHOT_LIST_RECORD_LEN 4u   /* <DTC 3 bytes> <status> */
#define DTC_SNAPSHOT_RECORD_HEADER_LEN 6u /* 59 04|06 <DTC 3 bytes> <status> */

_Static_assert((VEHICLE_CL250_DTC_MAX_PER_RUN >= 1u) && (VEHICLE_CL250_DTC_MAX_PER_RUN <= 8u),
               "D-080: the run state is sized for at most 8 DTCs");
_Static_assert(DTC_SNAPSHOT_MAX_REQUESTS_PER_RUN <= 18u, "D-080 item 2: at most 18 requests per run");
_Static_assert(VEHICLE_CL250_MAX_FF_DL <= 0xFFFFu, "answer lengths are uint16_t");

typedef enum {
    DTC_SNAPSHOT_STEP_NONE = 0,
    DTC_SNAPSHOT_STEP_COUNT,    /* 0x19 0x01 */
    DTC_SNAPSHOT_STEP_LIST,     /* 0x19 0x02 */
    DTC_SNAPSHOT_STEP_SNAPSHOT, /* 0x19 0x04 */
    DTC_SNAPSHOT_STEP_EXT_DATA  /* 0x19 0x06 */
} dtc_snapshot_step_t;

/* Why 0x19 is off for the rest of the stop epoch (NONE: not halted). */
typedef enum {
    DTC_SNAPSHOT_HALT_NONE = 0,
    DTC_SNAPSHOT_HALT_NOT_SERVED,
    DTC_SNAPSHOT_HALT_SEGMENTED,
    DTC_SNAPSHOT_HALT_SESSION_REFUSED,
    DTC_SNAPSHOT_HALT_NO_ANSWER,
    DTC_SNAPSHOT_HALT_SLOW,
    DTC_SNAPSHOT_HALT_LATE_FF
} dtc_snapshot_halt_t;

/* Why the last run ended (NONE: it has not, or none started). */
typedef enum {
    DTC_SNAPSHOT_END_NONE = 0,
    DTC_SNAPSHOT_END_DONE,      /* every listed DTC read */
    DTC_SNAPSHOT_END_NO_DTC,    /* the count was 0 */
    DTC_SNAPSHOT_END_NO_MATCH,  /* no listed DTC matches the status mask */
    DTC_SNAPSHOT_END_MALFORMED, /* an answer with a wrong length */
    DTC_SNAPSHOT_END_REFUSED,   /* any NRC other than the halting ones */
    DTC_SNAPSHOT_END_BUDGET,    /* DTC_SNAPSHOT_MAX_REQUESTS_PER_RUN spent */
    DTC_SNAPSHOT_END_HALTED     /* a halt ended it */
} dtc_snapshot_end_t;

typedef enum {
    DTC_SNAPSHOT_TRIGGER_KEY_ON = 0,
    DTC_SNAPSHOT_TRIGGER_WARM_IDLE
} dtc_snapshot_trigger_t;

/* What an NRC for the request in flight means for the caller. */
typedef enum {
    DTC_SNAPSHOT_NRC_WAIT = 0, /* 0x78: the request stays in flight, no extension (item 4) */
    DTC_SNAPSHOT_NRC_SESSION_LOST, /* 0x7E/0x7F: request the session again (D-075 item 1) */
    DTC_SNAPSHOT_NRC_ENDED         /* the request is resolved */
} dtc_snapshot_nrc_action_t;

/* Diagnostic counters since init (saturating). */
typedef struct {
    uint32_t runs;     /* runs started */
    uint32_t requests; /* 0x19 requests built (not an unsent one) */
    uint32_t answers;  /* complete positive answers accepted */
    uint32_t halts;    /* epochs in which 0x19 was turned off */
} dtc_snapshot_stats_t;

/* State. Treat as opaque: use the functions below. Everything before `stats` is the run
 * and epoch state (D-080 memory note: <= 64 B, asserted below). */
typedef struct {
    uint8_t dtcs[VEHICLE_CL250_DTC_MAX_PER_RUN][DTC_SNAPSHOT_DTC_BYTES]; /* ECU data, from a 0x02 answer */
    uint32_t warm_idle_since_ms; /* start of the continuous warm-idle hold */
    dtc_snapshot_step_t step;
    dtc_snapshot_halt_t halt;
    dtc_snapshot_end_t last_end;
    dtc_snapshot_trigger_t trigger;
    uint8_t runs_started;
    uint8_t requests_in_run;
    uint8_t requests_in_epoch;
    uint8_t session_refusals;
    uint8_t dtc_count;
    uint8_t dtc_index;
    bool epoch_started; /* start_epoch() ran since init */
    bool run_active;
    bool in_flight;
    bool warm_idle_used; /* the warm-idle trigger is used up in this epoch */
    bool warm_idle_timing; /* warm_idle_since_ms is set */
    dtc_snapshot_stats_t stats;
} dtc_snapshot_core_t;

_Static_assert(offsetof(dtc_snapshot_core_t, stats) <= 64u,
               "D-080 memory note: the run state must stay within 64 B");

void dtc_snapshot_core_init(dtc_snapshot_core_t* c);

/* A stop epoch starts: everything of the last epoch is forgotten (halt, run counts, the
 * warm-idle timer) and the key-on run starts. */
void dtc_snapshot_core_start_epoch(dtc_snapshot_core_t* c);

/*
 * D-080 item 1, once per step with the warm-idle condition of the glue (every
 * vehicle_cl250_dtc_warm_idle[] entry met by a valid sample). The conditions must hold
 * without a break for VEHICLE_CL250_DTC_WARM_IDLE_MIN_DURATION_MS; then the second run
 * starts, once per epoch. While the key-on run is still active nothing is used up and the
 * next step asks again; a halted epoch or a spent run budget refuses it and uses it up.
 * Without an epoch (start_epoch() not called) nothing happens. True when the run started.
 */
bool dtc_snapshot_core_warm_idle_step(dtc_snapshot_core_t* c, uint32_t now_ms, bool holds);

/* True if one condition entry is met: value op threshold (VEHICLE_CL250_DTC_OP_*). EQ is
 * exact float equality. An unknown op, a NaN or an infinite value is never met. */
bool dtc_snapshot_core_condition_met(uint8_t op, float value, float threshold);

/* A request is waiting: a run is active, 0x19 is not off for the epoch, nothing is in
 * flight. */
bool dtc_snapshot_core_due(const dtc_snapshot_core_t* c);

/*
 * Builds the next request (SID first, no ISO-TP PCI: room for DTC_SNAPSHOT_REQ_MAX) and
 * marks it in flight. False (*len 0) if none is due; the run ends if its request budget
 * is spent.
 */
bool dtc_snapshot_core_build_request(dtc_snapshot_core_t* c, uint8_t* out, uint16_t* len);

/* The request did not go out (the link was busy): back to "due", budgets restored. */
void dtc_snapshot_core_unsend(dtc_snapshot_core_t* c);

/*
 * msg starts at the response SID. True for the positive SID, the sub-function echo and,
 * for 0x04/0x06, the DTC echo of the request in flight (len may be only the First
 * Frame's bytes).
 */
bool dtc_snapshot_core_match_positive(const dtc_snapshot_core_t* c, const uint8_t* msg,
                                      uint16_t len);

/* The first bytes of the positive answer to the request in flight: 59 <sub-function>
 * and, for 0x04/0x06, the DTC. For the link's Flow Control check. False if none is in
 * flight. out has room for DTC_SNAPSHOT_PREFIX_MAX. */
bool dtc_snapshot_core_expected_prefix(const dtc_snapshot_core_t* c, uint8_t* out, uint8_t* len);

/* The complete positive answer to the request in flight (match_positive() was true;
 * checked again): runs the length and record checks and moves on. False if it was not
 * accepted. */
bool dtc_snapshot_core_on_positive(dtc_snapshot_core_t* c, const uint8_t* msg, uint16_t len);

/* An NRC to the request in flight (any SID-matched 0x7F 0x19 <nrc>). */
dtc_snapshot_nrc_action_t dtc_snapshot_core_on_nrc(dtc_snapshot_core_t* c, uint8_t nrc);

/* The request in flight timed out: halt NO_ANSWER. */
void dtc_snapshot_core_on_timeout(dtc_snapshot_core_t* c);

/* A refused, withheld or failed segmented reception of the answer in flight: halt SEGMENTED. */
void dtc_snapshot_core_on_segmented_failure(dtc_snapshot_core_t* c);

/* Turns 0x19 off for the rest of the epoch. A request in flight resolves normally first
 * (its answer is handed out); the run ends then (conn settle()). The first reason stays. */
void dtc_snapshot_core_halt_after_answer(dtc_snapshot_core_t* c, dtc_snapshot_halt_t reason);

/*
 * Getters. The ones used only by the tests and diagnostics are static inline (internal
 * linkage), so they are not external functions with a single user (MISRA 8.7).
 */
static inline bool dtc_snapshot_core_run_active(const dtc_snapshot_core_t* c)
{
    return (c != NULL) && c->run_active;
}

static inline bool dtc_snapshot_core_in_flight(const dtc_snapshot_core_t* c)
{
    return (c != NULL) && c->in_flight;
}

static inline dtc_snapshot_halt_t dtc_snapshot_core_halt(const dtc_snapshot_core_t* c)
{
    return (c == NULL) ? DTC_SNAPSHOT_HALT_NONE : c->halt;
}

static inline dtc_snapshot_end_t dtc_snapshot_core_last_end(const dtc_snapshot_core_t* c)
{
    return (c == NULL) ? DTC_SNAPSHOT_END_NONE : c->last_end;
}

static inline uint8_t dtc_snapshot_core_requests_in_run(const dtc_snapshot_core_t* c)
{
    return (c == NULL) ? 0u : c->requests_in_run;
}

static inline uint8_t dtc_snapshot_core_requests_in_epoch(const dtc_snapshot_core_t* c)
{
    return (c == NULL) ? 0u : c->requests_in_epoch;
}

static inline uint8_t dtc_snapshot_core_dtc_count(const dtc_snapshot_core_t* c)
{
    return (c == NULL) ? 0u : c->dtc_count;
}

/* Stored DTC i of the running run (DTC_SNAPSHOT_DTC_BYTES bytes), NULL if out of range. */
static inline const uint8_t* dtc_snapshot_core_dtc(const dtc_snapshot_core_t* c, uint8_t i)
{
    return ((c != NULL) && (i < c->dtc_count)) ? c->dtcs[i] : NULL;
}

static inline const dtc_snapshot_stats_t* dtc_snapshot_core_stats(const dtc_snapshot_core_t* c)
{
    return (c == NULL) ? NULL : &c->stats;
}

dtc_snapshot_step_t dtc_snapshot_core_step(const dtc_snapshot_core_t* c);
dtc_snapshot_trigger_t dtc_snapshot_core_trigger(const dtc_snapshot_core_t* c);
uint8_t dtc_snapshot_core_runs_started(const dtc_snapshot_core_t* c);

/* The sub-function of a step (gen/ UDS_READ_DTC_REPORT_*); 0 for NONE. */
uint8_t dtc_snapshot_core_sub_function(dtc_snapshot_step_t step);

#ifdef __cplusplus
}
#endif

#endif /* DTC_SNAPSHOT_CORE_H */
