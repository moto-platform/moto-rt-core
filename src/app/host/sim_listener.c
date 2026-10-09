#include "app/host/sim_listener.h"

#include "platform.h"
#include "platform_e2e.h"

#include <string.h>

typedef enum { HB_E2E_CRC, HB_E2E_COUNTER, HB_NODE_MODE, HB_ERROR_COUNT, HB_UPTIME } hb_field_t;

static void hb_set(struct platform_heartbeat_rt_core_t* m, hb_field_t f, uint32_t v)
{
    switch (f) {
    case HB_E2E_CRC:
        m->e2_e_crc = (uint8_t)v;
        break;
    case HB_E2E_COUNTER:
        m->e2_e_counter = (uint8_t)v;
        break;
    case HB_NODE_MODE:
        m->node_mode = (uint8_t)v;
        break;
    case HB_ERROR_COUNT:
        m->error_count = (uint8_t)v;
        break;
    default:
        m->uptime = (uint16_t)v;
        break;
    }
}

/* Bit k of field f: the bytes gen/ pack sets for a struct holding only that bit. This
 * relies on pack() of a zeroed 0x081 setting no bit (no offset or constant field in the
 * DBC message), which holds for 0x081. */
static uint32_t hb_field(const uint8_t* data, hb_field_t f, uint32_t width)
{
    uint32_t v = 0u;
    for (uint32_t k = 0u; k < width; k++) {
        struct platform_heartbeat_rt_core_t m;
        uint8_t probe[PLATFORM_HEARTBEAT_RT_CORE_LENGTH];
        memset(&m, 0, sizeof m);
        hb_set(&m, f, 1u << k);
        (void)platform_heartbeat_rt_core_pack(probe, &m, sizeof probe);
        for (uint32_t i = 0u; i < PLATFORM_HEARTBEAT_RT_CORE_LENGTH; i++) {
            if ((probe[i] & data[i]) != 0u) {
                v |= 1u << k;
            }
        }
    }
    return v;
}

void sim_listener_heartbeat_decode(const uint8_t* data, struct platform_heartbeat_rt_core_t* msg)
{
    memset(msg, 0, sizeof *msg);
    msg->e2_e_crc = (uint8_t)hb_field(data, HB_E2E_CRC, 8u * sizeof msg->e2_e_crc);
    msg->e2_e_counter = (uint8_t)hb_field(data, HB_E2E_COUNTER, 8u * sizeof msg->e2_e_counter);
    msg->node_mode = (uint8_t)hb_field(data, HB_NODE_MODE, 8u * sizeof msg->node_mode);
    msg->error_count = (uint8_t)hb_field(data, HB_ERROR_COUNT, 8u * sizeof msg->error_count);
    msg->uptime = (uint16_t)hb_field(data, HB_UPTIME, 8u * sizeof msg->uptime);
}

bool sim_listener_init(sim_listener_t* l, vbus_t* bus)
{
    memset(l, 0, sizeof *l);
    l->bus = bus;
    moto_e2e_rx_init(&l->speed_rx);
    moto_e2e_rx_init(&l->heartbeat_rx);
    return vbus_attach(bus, &l->node);
}

static void count(sim_listener_msg_t* m, uint32_t now_ms, bool length_ok)
{
    if (m->seen && ((now_ms - m->last_ms) > m->gap_max_ms)) {
        m->gap_max_ms = now_ms - m->last_ms;
    }
    m->seen = true;
    m->last_ms = now_ms;
    m->frames++;
    m->bad_length += length_ok ? 0u : 1u;
}

/* The receiver side of 0x081, as SAFETY would run it (D-042 item 3). */
static void check_heartbeat(sim_listener_t* l, const can_frame_t* f, uint32_t now_ms)
{
    count(&l->heartbeat, now_ms, f->dlc == PLATFORM_HEARTBEAT_RT_CORE_LENGTH);
    const moto_e2e_status_t st =
        moto_e2e_check(PLATFORM_HEARTBEAT_RT_CORE_E2E_DATA_ID,
                       PLATFORM_HEARTBEAT_RT_CORE_E2E_MAX_DELTA_COUNTER,
                       PLATFORM_HEARTBEAT_RT_CORE_E2E_TIMEOUT_MS, f->data, f->dlc,
                       &l->heartbeat_rx, now_ms);
    const bool first = (l->heartbeat.frames == 1u);
    if ((st == MOTO_E2E_OK) || (first && (st == MOTO_E2E_INITIAL))) {
        l->heartbeat_e2e_ok++;
    } else {
        l->heartbeat_e2e_bad++;
    }
    if (f->dlc != PLATFORM_HEARTBEAT_RT_CORE_LENGTH) {
        return; /* counted as bad_length */
    }
    struct platform_heartbeat_rt_core_t msg;
    sim_listener_heartbeat_decode(f->data, &msg);
    if (msg.node_mode == PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_NORMAL_CHOICE) {
        l->heartbeat_normal++;
    } else if (msg.node_mode == PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_DEGRADED_CHOICE) {
        l->heartbeat_degraded++;
    } else if (!first || (msg.node_mode != PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_INIT_CHOICE)) {
        l->heartbeat_bad_mode++;
    } else {
        /* INIT on the first frame */
    }
    if (!first && (msg.uptime < l->heartbeat_uptime)) {
        l->heartbeat_uptime_back++;
    }
    l->heartbeat_uptime = msg.uptime;
}

void sim_listener_step(sim_listener_t* l, uint32_t now_ms)
{
    can_frame_t f;
    while (vbus_recv(l->bus, l->node, &f) == CAN_PORT_OK) {
        if (f.extended) {
            continue;
        }
        if (f.id == PLATFORM_VEHICLE_SPEED_FRAME_ID) {
            count(&l->speed, now_ms, f.dlc == PLATFORM_VEHICLE_SPEED_LENGTH);
            const moto_e2e_status_t st =
                moto_e2e_check(PLATFORM_VEHICLE_SPEED_E2E_DATA_ID,
                               PLATFORM_VEHICLE_SPEED_E2E_MAX_DELTA_COUNTER,
                               PLATFORM_VEHICLE_SPEED_E2E_TIMEOUT_MS, f.data, f.dlc, &l->speed_rx,
                               now_ms);
            const bool first = (l->speed.frames == 1u) && (st == MOTO_E2E_INITIAL);
            if ((st == MOTO_E2E_OK) || first) {
                l->speed_e2e_ok++;
            } else {
                l->speed_e2e_bad++;
            }
        } else if (f.id == PLATFORM_VEHICLE_ENGINE_FRAME_ID) {
            count(&l->engine, now_ms, f.dlc == PLATFORM_VEHICLE_ENGINE_LENGTH);
        } else if (f.id == PLATFORM_HEARTBEAT_RT_CORE_FRAME_ID) {
            check_heartbeat(l, &f, now_ms);
        } else {
            /* UDS and other traffic: not this listener's business */
        }
    }
}

bool sim_listener_passed(const sim_listener_t* l)
{
    return l->speed.seen && l->engine.seen && (l->speed_e2e_bad == 0u) &&
           (l->speed.bad_length == 0u) && (l->engine.bad_length == 0u) &&
           (l->speed.gap_max_ms < PLATFORM_VEHICLE_SPEED_E2E_TIMEOUT_MS) &&
           (l->engine.gap_max_ms < (3u * PLATFORM_VEHICLE_ENGINE_CYCLE_TIME_MS)) &&
           l->heartbeat.seen && (l->heartbeat_e2e_bad == 0u) && (l->heartbeat.bad_length == 0u) &&
           (l->heartbeat.gap_max_ms < PLATFORM_HEARTBEAT_RT_CORE_E2E_TIMEOUT_MS) &&
           (l->heartbeat_bad_mode == 0u) && (l->heartbeat_uptime_back == 0u);
}
