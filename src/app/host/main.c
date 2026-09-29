/*
 * moto_rtcore_host: rt-core as a host program (SIL, D-034). The same services/ and
 * features/ code as on the H7 runs against the host HAL:
 *   (default)      in-process virtual bus with a simulated CL250 ECU (app/host/sim_ecu)
 *   --vcan IF      vehicle port on a Linux SocketCAN interface (e.g. vcan0); the peer
 *                  is external (moto-hil-bench live model, can-utils isotpsend, ...).
 *                  Interfaces not named vcan* need --allow-real-bus: a second tester
 *                  next to a real rt-core on the bike would break D-021.
 *
 * The loop runs the UDS client (features/uds/uds_client, Ç3), the single read-only
 * vehicle tester: extended session, tester present, and the gen/ DIDs polled
 * round-robin into services/vehicle_signals. Every request passes the generated D-020
 * gates and the can_if guard. Once a second it prints the signal table.
 *
 * Exit code with --duration-ms: 0 if at least --min-responses DID reads were decoded and
 * the client did not latch as failed.
 */
#include "app/host/sim_ecu.h"
#include "features/uds/uds_client.h"
#include "hal/host/can_port_host.h"
#include "hal/host/hal_time_host.h"
#include "services/can_if.h"
#include "services/timebase.h"
#include "services/vehicle_signals.h"
#include "vehicle_cl250.h"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LOOP_PERIOD_MS 1u
#define RX_PER_PASS 32u
#define PRINT_PERIOD_MS 1000u
#define VIRTUAL_IF_PREFIX "vcan"

static volatile sig_atomic_t stop_requested;

static void on_signal(int sig)
{
    (void)sig;
    stop_requested = 1;
}

typedef struct {
    const char* vcan;
    uint32_t duration_ms; /* 0 = until Ctrl-C */
    uint32_t min_responses;
    int quiet;
    int allow_real_bus;
} options_t;

static void usage(void)
{
    fprintf(stderr,
            "usage: moto_rtcore_host [--vcan IF] [--duration-ms N] [--min-responses N]\n"
            "                        [--quiet] [--allow-real-bus]\n");
}

static int parse_u32(const char* s, uint32_t* out)
{
    char* end = NULL;
    unsigned long v = strtoul(s, &end, 10);
    if ((end == s) || (*end != '\0') || (v > 0xFFFFFFFFul)) {
        return 0;
    }
    *out = (uint32_t)v;
    return 1;
}

static int parse_args(int argc, char** argv, options_t* o)
{
    o->vcan = NULL;
    o->duration_ms = 0u;
    o->min_responses = 1u;
    o->quiet = 0;
    o->allow_real_bus = 0;
    for (int i = 1; i < argc; i++) {
        const char* a = argv[i];
        const char* v = (i + 1 < argc) ? argv[i + 1] : NULL;
        if ((strcmp(a, "--vcan") == 0) && (v != NULL)) {
            o->vcan = v;
            i++;
        } else if ((strcmp(a, "--duration-ms") == 0) && (v != NULL) && parse_u32(v, &o->duration_ms)) {
            i++;
        } else if ((strcmp(a, "--min-responses") == 0) && (v != NULL) &&
                   parse_u32(v, &o->min_responses)) {
            i++;
        } else if (strcmp(a, "--quiet") == 0) {
            o->quiet = 1;
        } else if (strcmp(a, "--allow-real-bus") == 0) {
            o->allow_real_bus = 1;
        } else {
            return 0;
        }
    }
    return 1;
}

static vbus_t bus;
static sim_ecu_t ecu;
static uds_client_t client;

static const char* state_name(vehicle_signal_state_t st)
{
    switch (st) {
    case VEHICLE_SIGNAL_VALID:
        return "VALID";
    case VEHICLE_SIGNAL_STALE:
        return "STALE";
    default:
        return "none";
    }
}

static void print_signals(uint32_t now, uint32_t start)
{
    printf("%8u ms  session %s, ECU %s\n", (unsigned)timebase_elapsed_ms(now, start),
           uds_client_session_up(&client) ? "up" : "down",
           vehicle_signals_ecu_present() ? "present" : "absent");
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        vehicle_signal_sample_t s;
        if (vehicle_signals_get(i, now, &s)) {
            printf("            DID 0x%04X  raw %6u  %10.3f  age %5u ms  %s\n",
                   (unsigned)vehicle_cl250_dids[i].did, (unsigned)s.raw, (double)s.physical,
                   (unsigned)s.age_ms, state_name(s.state));
        }
    }
}

int main(int argc, char** argv)
{
    options_t opt;
    if (!parse_args(argc, argv, &opt)) {
        usage();
        return 2;
    }
    (void)signal(SIGINT, on_signal);
    (void)signal(SIGTERM, on_signal);

    const int simulated = (opt.vcan == NULL);
    if (simulated) {
        uint8_t node = 0u;
        vbus_init(&bus);
        if (!vbus_attach(&bus, &node) || !can_port_host_bind_vbus(CAN_PORT_VEHICLE, &bus, node) ||
            !sim_ecu_init(&ecu, &bus)) {
            fprintf(stderr, "virtual bus setup failed\n");
            return 1;
        }
        ecu.require_session = true; /* reads need the extended session, like the CL250 */
    } else if ((strncmp(opt.vcan, VIRTUAL_IF_PREFIX, strlen(VIRTUAL_IF_PREFIX)) != 0) &&
               !opt.allow_real_bus) {
        fprintf(stderr, "%s is not a virtual CAN interface; pass --allow-real-bus if this is intended "
                        "(this program is a full tester: session control, tester present and reads; only one tester may be on the vehicle bus, D-021)\n", opt.vcan);
        return 2;
    } else if (!can_port_host_bind_socketcan(CAN_PORT_VEHICLE, opt.vcan)) {
        fprintf(stderr, "cannot open SocketCAN interface %s%s\n", opt.vcan,
                can_port_host_has_socketcan() ? "" : " (SocketCAN needs Linux)");
        return 1;
    }

    can_if_init();
    vehicle_signals_init();
    if (uds_client_open(&client) != ISOTP_OK) {
        fprintf(stderr, "vehicle ISO-TP link setup failed\n");
        return 1;
    }
    printf("moto_rtcore_host: UDS client 0x%08X -> 0x%08X on %s\n",
           (unsigned)VEHICLE_CL250_REQUEST_ID, (unsigned)VEHICLE_CL250_RESPONSE_ID,
           simulated ? "in-process bus + simulated CL250 ECU" : opt.vcan);

    const uint32_t start = timebase_now_ms();
    uint32_t printed_at = start;

    while (!stop_requested) {
        const uint32_t now = timebase_now_ms();
        if ((opt.duration_ms != 0u) && timebase_expired(now, start, opt.duration_ms)) {
            break;
        }

        (void)can_if_dispatch(CAN_PORT_VEHICLE, RX_PER_PASS);
        if (simulated) {
            sim_ecu_step(&ecu, now);
            (void)can_if_dispatch(CAN_PORT_VEHICLE, RX_PER_PASS);
        }
        uds_client_step(&client);

        if (!opt.quiet && timebase_expired(now, printed_at, PRINT_PERIOD_MS)) {
            print_signals(now, start);
            printed_at = now;
        }
        hal_time_host_sleep_ms(LOOP_PERIOD_MS);
    }

    const uds_client_stats_t* st = uds_client_stats(&client);
    printf("moto_rtcore_host: %u reads, %u timeouts, %u NRC (%u pending), %u unavailable, "
           "%u sessions, %u requests; %u refused, %u TX errors, %u guard refusals, fault %d%s\n",
           (unsigned)st->reads_ok, (unsigned)st->timeouts, (unsigned)st->nrc,
           (unsigned)st->response_pending, (unsigned)st->unavailable, (unsigned)st->session_starts,
           (unsigned)st->requests, (unsigned)isotp_link_tx_refused_count(&client.link),
           (unsigned)isotp_link_tx_error_count(&client.link),
           (unsigned)can_if_tx_refused_count(CAN_PORT_VEHICLE), (int)uds_client_fault(&client),
           uds_client_failed(&client) ? ", CLIENT FAILED" : "");
    can_port_host_unbind_all();
    if (opt.duration_ms == 0u) {
        return 0;
    }
    return ((st->reads_ok >= opt.min_responses) && !uds_client_failed(&client)) ? 0 : 1;
}
