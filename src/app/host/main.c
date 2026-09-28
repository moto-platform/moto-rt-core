/*
 * moto_rtcore_host: rt-core as a host program (SIL, D-034). The same services/ and
 * features/ code as on the H7 runs against the host HAL:
 *   (default)      in-process virtual bus with a simulated CL250 ECU (app/host/sim_ecu)
 *   --vcan IF      vehicle port on a Linux SocketCAN interface (e.g. vcan0); the peer
 *                  is external (moto-hil-bench live model, can-utils isotpsend, ...).
 *                  Interfaces not named vcan* need --allow-real-bus: a second tester
 *                  next to a real rt-core on the bike would break D-021.
 *
 * The loop is a transport smoke demo, not the UDS client (Ç3): it reads the gen/ DIDs
 * round-robin with 0x22 over the vehicle ISO-TP link, one request in flight, and
 * prints the decoded values. Every request passes the generated D-020 gates.
 *
 * Exit code with --duration-ms: 0 if at least --min-responses answers were decoded.
 */
#include "app/host/sim_ecu.h"
#include "features/uds/isotp_link.h"
#include "hal/host/can_port_host.h"
#include "hal/host/hal_time_host.h"
#include "services/can_if.h"
#include "services/timebase.h"
#include "vehicle_cl250.h"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LOOP_PERIOD_MS 1u
#define RX_PER_PASS 32u
#define LINK_BUF 256u
#define UDS_SID_RDBI 0x22u
#define UDS_POSITIVE_OFFSET 0x40u
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
    uint32_t gap_ms;      /* pause between requests */
    int quiet;
    int allow_real_bus;
} options_t;

static void usage(void)
{
    fprintf(stderr,
            "usage: moto_rtcore_host [--vcan IF] [--duration-ms N] [--min-responses N]\n"
            "                        [--gap-ms N] [--quiet] [--allow-real-bus]\n");
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
    o->gap_ms = 100u;
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
        } else if ((strcmp(a, "--gap-ms") == 0) && (v != NULL) && parse_u32(v, &o->gap_ms)) {
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
static isotp_can_link_t vehicle;
static uint8_t vehicle_rx[LINK_BUF];
static uint8_t vehicle_tx[LINK_BUF];

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
    } else if ((strncmp(opt.vcan, VIRTUAL_IF_PREFIX, strlen(VIRTUAL_IF_PREFIX)) != 0) &&
               !opt.allow_real_bus) {
        fprintf(stderr, "%s is not a virtual CAN interface; pass --allow-real-bus if this is intended "
                        "(only one tester may be on the vehicle bus, D-021)\n", opt.vcan);
        return 2;
    } else if (!can_port_host_bind_socketcan(CAN_PORT_VEHICLE, opt.vcan)) {
        fprintf(stderr, "cannot open SocketCAN interface %s%s\n", opt.vcan,
                can_port_host_has_socketcan() ? "" : " (SocketCAN needs Linux)");
        return 1;
    }

    can_if_init();
    if (isotp_link_open_vehicle_cl250(&vehicle, vehicle_rx, LINK_BUF, vehicle_tx, LINK_BUF) != ISOTP_OK) {
        fprintf(stderr, "vehicle ISO-TP link setup failed\n");
        return 1;
    }
    printf("moto_rtcore_host: vehicle link 0x%08X -> 0x%08X on %s\n",
           (unsigned)VEHICLE_CL250_REQUEST_ID, (unsigned)VEHICLE_CL250_RESPONSE_ID,
           simulated ? "in-process bus + simulated CL250 ECU" : opt.vcan);

    const uint32_t start = timebase_now_ms();
    uint32_t responses = 0u;
    uint32_t failures = 0u;
    uint32_t did_idx = 0u;
    int in_flight = 0;
    uint32_t sent_at = start;
    uint32_t idle_since = start;

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

        isotp_n_result_t res;
        uint16_t len = 0u;
        if (isotp_link_take_rx(&vehicle, &res, &len)) {
            const vehicle_cl250_did_t* e = &vehicle_cl250_dids[did_idx];
            const uint8_t* d = isotp_link_rx_data(&vehicle);
            float value = 0.0f;
            if ((res == ISOTP_N_OK) && (len >= 3u) &&
                (d[0] == (uint8_t)(UDS_SID_RDBI + UDS_POSITIVE_OFFSET)) &&
                (d[1] == (uint8_t)(e->did >> 8u)) && (d[2] == (uint8_t)(e->did & 0xFFu)) &&
                vehicle_cl250_decode(e, &d[3], (size_t)len - 3u, &value)) {
                responses++;
                if (!opt.quiet) {
                    printf("%8u ms  DID 0x%04X = %.2f\n", (unsigned)timebase_elapsed_ms(now, start),
                           (unsigned)e->did, (double)value);
                }
            } else {
                failures++;
                printf("%8u ms  DID 0x%04X: no valid answer (N_Result %d, %u bytes)\n",
                       (unsigned)timebase_elapsed_ms(now, start), (unsigned)e->did, (int)res,
                       (unsigned)len);
            }
            if (res == ISOTP_N_OK) {
                isotp_link_rx_release(&vehicle);
            }
            in_flight = 0;
            idle_since = now;
            did_idx = (did_idx + 1u) % VEHICLE_CL250_DID_COUNT;
        }
        if (isotp_link_take_tx_confirm(&vehicle, &res) && (res != ISOTP_N_OK)) {
            printf("request TX failed: N_Result %d\n", (int)res);
        }

        if (in_flight && timebase_expired(now, sent_at, VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS)) {
            failures++;
            printf("%8u ms  DID 0x%04X: response timeout\n", (unsigned)timebase_elapsed_ms(now, start),
                   (unsigned)vehicle_cl250_dids[did_idx].did);
            in_flight = 0;
            idle_since = now;
            did_idx = (did_idx + 1u) % VEHICLE_CL250_DID_COUNT;
        }
        if (!in_flight && timebase_expired(now, idle_since, opt.gap_ms)) {
            const uint16_t did = vehicle_cl250_dids[did_idx].did;
            const uint8_t req[3] = {UDS_SID_RDBI, (uint8_t)(did >> 8u), (uint8_t)(did & 0xFFu)};
            if (isotp_link_send(&vehicle, req, (uint16_t)sizeof req) == ISOTP_OK) {
                in_flight = 1;
                sent_at = now;
            }
        }

        isotp_link_step(&vehicle);
        hal_time_host_sleep_ms(LOOP_PERIOD_MS);
    }

    printf("moto_rtcore_host: %u responses, %u failures, %u refused, %u TX errors, %u guard refusals\n",
           (unsigned)responses, (unsigned)failures, (unsigned)isotp_link_tx_refused_count(&vehicle),
           (unsigned)isotp_link_tx_error_count(&vehicle),
           (unsigned)can_if_tx_refused_count(CAN_PORT_VEHICLE));
    can_port_host_unbind_all();
    if (opt.duration_ms == 0u) {
        return 0;
    }
    return (responses >= opt.min_responses) ? 0 : 1;
}
