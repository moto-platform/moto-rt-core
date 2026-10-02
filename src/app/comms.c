#include "app/comms.h"

#include "services/can_if.h"
#include "services/can_sm.h"

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the host program and the H7 task
bool comms_apply_filters(void)
{
    const can_port_status_t vehicle = can_if_apply_filters(CAN_PORT_VEHICLE);
    const can_port_status_t platform = can_if_apply_filters(CAN_PORT_PLATFORM);
    return (vehicle == CAN_PORT_OK) && (platform == CAN_PORT_OK);
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the host program and the H7 task
void comms_pass(uds_client_t* client, uds_server_t* server, vehicle_republish_t* republisher)
{
    can_sm_step(CAN_PORT_VEHICLE);
    (void)can_if_dispatch(CAN_PORT_VEHICLE, COMMS_RX_PER_PASS);
    uds_client_step(client);

    can_sm_step(CAN_PORT_PLATFORM);
    (void)can_if_dispatch(CAN_PORT_PLATFORM, COMMS_RX_PER_PASS);
    vehicle_republish_step(republisher);
    uds_server_step(server);
}
