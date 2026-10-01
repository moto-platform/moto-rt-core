/* MISRA gate canary. CI requires the cppcheck MISRA addon to report 10.8, 15.6 and 17.7
 * here, so a missing or broken addon cannot pass as a clean src/. Outside src/: never
 * built, never scanned with the firmware; do not fix it. */
#include <stdint.h>
#include <string.h>

uint16_t canary_unpack(uint8_t value, uint8_t mask);
int canary_init(uint8_t *msg_p);

uint16_t canary_unpack(uint8_t value, uint8_t mask)
{
    return (uint16_t)((uint16_t)(value & mask) << 8u); /* 10.8 */
}

int canary_init(uint8_t *msg_p)
{
    if (msg_p == NULL) return -1; /* 15.6 */

    memset(msg_p, 0, 8u); /* 17.7 */

    return 0;
}
