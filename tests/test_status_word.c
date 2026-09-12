#include <stdint.h>
#include <stdio.h>

#include "../firmware/status_word.h"

static int failures = 0;

static void expect_status(uint8_t attempts, uint8_t config, uint8_t poll,
                          uint16_t expected)
{
    uint16_t actual = gps_status_pack(attempts, config, poll);
    if (actual != expected) {
        fprintf(stderr, "status: got 0x%04X, expected 0x%04X\n", actual, expected);
        failures++;
    }
}

int main(void)
{
    expect_status(0, OP_STATUS_OK, OP_STATUS_OK, 0x0000);
    expect_status(1, OP_STATUS_TRANSIENT_ERROR, OP_STATUS_RETRY_EXHAUSTED, 0x0016);
    expect_status(15, OP_STATUS_RETRY_EXHAUSTED, OP_STATUS_DEGRADED, 0x00FB);
    expect_status(31, 7, 7, 0x00FF);

    if (failures != 0) return 1;
    puts("status word tests passed");
    return 0;
}
