#ifndef APP_HOST_SIM_ECU_H
#define APP_HOST_SIM_ECU_H

/*
 * Minimal simulated CL250 engine ECU for the host SIL demo and tests: a UDS
 * ReadDataByIdentifier (0x22) responder on the in-process bus, using the DID table
 * from gen/vehicle_cl250.h. Values sweep slowly across each DID's [min, max].
 * Anything else gets a negative response (0x7F). It is a stand-in until the live
 * vehicle model of moto-hil-bench (D-035) exists; not firmware.
 */

#include "features/uds/isotp_core.h"
#include "hal/host/vbus.h"

#include <stdbool.h>
#include <stdint.h>

#define SIM_ECU_BUF 64u

typedef struct {
    vbus_t* bus;
    uint8_t node;
    isotp_link_t iso;
    uint8_t rx_buf[SIM_ECU_BUF];
    uint8_t tx_buf[SIM_ECU_BUF];
    uint32_t requests;
    bool silent; /* test hook: ignore requests */
} sim_ecu_t;

bool sim_ecu_init(sim_ecu_t* ecu, vbus_t* bus);
void sim_ecu_step(sim_ecu_t* ecu, uint32_t now_ms);

#endif /* APP_HOST_SIM_ECU_H */
