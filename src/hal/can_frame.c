#include "hal/can_types.h"

#include <stddef.h>

bool can_id_valid(uint32_t id, bool extended)
{
    return id <= (extended ? CAN_PORT_EXT_ID_MAX : CAN_PORT_STD_ID_MAX);
}

bool can_frame_valid(const can_frame_t* frame)
{
    return (frame != NULL) && (frame->dlc <= CAN_PORT_MAX_DLC) &&
           can_id_valid(frame->id, frame->extended);
}
