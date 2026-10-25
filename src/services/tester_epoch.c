#include "services/tester_epoch.h"

static tester_epoch_t state; /* closed, counter 0 */

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the UDS client glue and the tests
void tester_epoch_set(bool open, uint16_t counter)
{
    state.open = open;
    state.counter = counter;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by services/derived_inputs and the tests
tester_epoch_t tester_epoch_get(void)
{
    return state;
}
