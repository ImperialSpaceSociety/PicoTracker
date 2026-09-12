#include <stdint.h>
#include <stdio.h>

#include "../firmware/sleep_policy.h"

static int failures = 0;

static void expect_intervals(uint16_t altitude, uint8_t expected)
{
    uint8_t actual = tracker_sleep_intervals_for_altitude(altitude);
    if (actual != expected) {
        fprintf(stderr, "sleep altitude %u: got %u, expected %u\n",
                altitude, actual, expected);
        failures++;
    }
}

int main(void)
{
    expect_intervals(0, 1);
    expect_intervals(500, 1);
    expect_intervals(3000, 1);
    expect_intervals(3001, 2);
    expect_intervals(50000, 2);

    if (failures != 0) return 1;
    puts("sleep policy tests passed");
    return 0;
}
