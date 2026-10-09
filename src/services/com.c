#include "services/com.h"

#include "services/can_if.h"

#include <stddef.h>
#include <string.h>

static void stat_inc(uint32_t* counter)
{
    if (*counter < UINT32_MAX) {
        (*counter)++;
    }
}

bool com_send(uint32_t id, bool extended, const uint8_t* data, uint8_t dlc,
              com_msg_stats_t* stats, can_port_status_t* status)
{
    can_port_status_t st = CAN_PORT_ERR_ARG; /* not built: dropped like a bus-off */
    if ((data != NULL) && (dlc <= CAN_PORT_MAX_DLC)) {
        can_frame_t frame;
        (void)memset(&frame, 0, sizeof frame);
        frame.id = id;
        frame.extended = extended;
        frame.dlc = dlc;
        (void)memcpy(frame.data, data, dlc);
        st = can_if_write(CAN_PORT_PLATFORM, &frame);
    }
    *status = st;
    bool finished = true;
    if (st == CAN_PORT_OK) {
        stat_inc(&stats->sent);
    } else if (st == CAN_PORT_TX_FULL) {
        stat_inc(&stats->retried); /* replace cancel pending or Tx FIFO full */
        finished = false;
    } else {
        stat_inc(&stats->dropped); /* bus-off or latched (can_sm), or a frame not built */
    }
    return finished;
}
