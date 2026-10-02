#include "features/vehicle_republish/vehicle_republish.h"

#include "services/can_if.h"
#include "services/timebase.h"

#include <stddef.h>
#include <string.h>

_Static_assert((PLATFORM_VEHICLE_SPEED_CYCLE_TIME_MS > 0u) && (PLATFORM_VEHICLE_ENGINE_CYCLE_TIME_MS > 0u),
               "a zero cycle time would send every pass");

static void stat_inc(uint32_t* counter)
{
    if (*counter < UINT32_MAX) {
        (*counter)++;
    }
}

static void build_frame(can_frame_t* frame, uint32_t id, bool extended, const uint8_t* data,
                        uint8_t dlc)
{
    (void)memset(frame, 0, sizeof *frame);
    frame->id = id;
    frame->extended = extended;
    frame->dlc = dlc;
    (void)memcpy(frame->data, data, dlc);
}

/* True when the cycle is finished (sent or given up), false to retry in the next pass. */
static bool handle(can_port_status_t st, vehicle_republish_msg_stats_t* stats)
{
    if (st == CAN_PORT_OK) {
        stat_inc(&stats->sent);
        return true;
    }
    if (st == CAN_PORT_TX_FULL) {
        stat_inc(&stats->retried); /* replace cancel pending or Tx FIFO full */
        return false;
    }
    stat_inc(&stats->dropped); /* bus-off or latched (can_sm), or a frame that could not be built */
    return true;
}

/* A sample the store cannot return stays NONE, so it goes out INVALID. */
static void read_sample(uint32_t idx, uint32_t now, vehicle_signal_sample_t* s)
{
    (void)memset(s, 0, sizeof *s); /* VEHICLE_SIGNAL_NONE */
    (void)vehicle_signals_get(idx, now, s);
}

static void send_speed(vehicle_republish_t* r, uint32_t now)
{
    vehicle_signal_sample_t s;
    struct platform_vehicle_speed_t msg;
    moto_e2e_tx_state_t next;
    uint8_t data[PLATFORM_VEHICLE_SPEED_LENGTH];
    can_frame_t frame;

    read_sample(VEHICLE_CL250_IDX_VEHICLE_SPEED, now, &s);
    vehicle_republish_speed_msg(&s, &msg);
    can_port_status_t st = CAN_PORT_ERR_ARG; /* not built: dropped like a bus-off */
    if (vehicle_republish_speed_frame(&msg, &r->speed_e2e, &next, data)) {
        build_frame(&frame, PLATFORM_VEHICLE_SPEED_FRAME_ID,
                    PLATFORM_VEHICLE_SPEED_IS_EXTENDED != 0, data,
                    (uint8_t)PLATFORM_VEHICLE_SPEED_LENGTH);
        st = can_if_write(CAN_PORT_PLATFORM, &frame);
    }
    if (st == CAN_PORT_OK) {
        r->speed_e2e = next; /* the counter advances with accepted frames only */
    }
    if (handle(st, &r->speed)) {
        vehicle_republish_cycle_done(&r->speed_cycle, now, PLATFORM_VEHICLE_SPEED_CYCLE_TIME_MS);
    }
}

static void send_engine(vehicle_republish_t* r, uint32_t now)
{
    vehicle_signal_sample_t samples[VEHICLE_CL250_DID_COUNT];
    struct platform_vehicle_engine_t msg;
    uint8_t data[PLATFORM_VEHICLE_ENGINE_LENGTH];
    can_frame_t frame;

    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        read_sample(i, now, &samples[i]);
    }
    vehicle_republish_engine_msg(samples, vehicle_signals_ecu_present(), &msg);
    can_port_status_t st = CAN_PORT_ERR_ARG; /* not built: dropped like a bus-off */
    if (vehicle_republish_engine_frame(&msg, data)) {
        build_frame(&frame, PLATFORM_VEHICLE_ENGINE_FRAME_ID,
                    PLATFORM_VEHICLE_ENGINE_IS_EXTENDED != 0, data,
                    (uint8_t)PLATFORM_VEHICLE_ENGINE_LENGTH);
        st = can_if_write(CAN_PORT_PLATFORM, &frame);
    }
    if (handle(st, &r->engine)) {
        vehicle_republish_cycle_done(&r->engine_cycle, now, PLATFORM_VEHICLE_ENGINE_CYCLE_TIME_MS);
    }
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the host program and the H7 setup
bool vehicle_republish_open(vehicle_republish_t* r)
{
    if (r == NULL) {
        return false;
    }
    (void)memset(r, 0, sizeof *r);
    moto_e2e_tx_init(&r->speed_e2e);
    r->opened = (can_if_register_tx_dedicated(CAN_PORT_PLATFORM, PLATFORM_VEHICLE_SPEED_FRAME_ID,
                                              PLATFORM_VEHICLE_SPEED_IS_EXTENDED != 0) == CAN_IF_OK);
    return r->opened;
}

void vehicle_republish_step(vehicle_republish_t* r)
{
    if ((r == NULL) || !r->opened) {
        return; /* never 0x021 without its dedicated buffer */
    }
    const uint32_t now = timebase_now_ms();
    if (vehicle_republish_cycle_due(&r->speed_cycle, now)) {
        send_speed(r, now);
    }
    if (vehicle_republish_cycle_due(&r->engine_cycle, now)) {
        send_engine(r, now);
    }
}
