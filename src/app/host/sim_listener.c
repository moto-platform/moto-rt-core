#include "app/host/sim_listener.h"

#include "platform.h"
#include "platform_e2e.h"

#include <string.h>

bool sim_listener_init(sim_listener_t* l, vbus_t* bus)
{
    memset(l, 0, sizeof *l);
    l->bus = bus;
    moto_e2e_rx_init(&l->speed_rx);
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
           (l->engine.gap_max_ms < (3u * PLATFORM_VEHICLE_ENGINE_CYCLE_TIME_MS));
}
