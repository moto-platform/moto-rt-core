#include "services/lean_health.h"

#include <stddef.h>

static lean_health_rec_t record;
static lean_health_comms_t comms_flags;

void lean_health_init(void)
{
    const lean_health_rec_t none_rec = {false, 0u, {false, false, 0u, 0u}, false};
    const lean_health_comms_t none_comms = {false, false, false};
    record = none_rec;
    comms_flags = none_comms;
}

void lean_health_set_record(const lean_health_rec_t* rec)
{
    if (rec != NULL) {
        record = *rec;
    }
}

void lean_health_set_comms(const lean_health_comms_t* comms)
{
    if (comms != NULL) {
        comms_flags = *comms;
    }
}

bool lean_health_read(uint32_t now_ms, uint8_t* out, uint16_t len)
{
    return lean_health_build(&record, &comms_flags, now_ms, out, len);
}
