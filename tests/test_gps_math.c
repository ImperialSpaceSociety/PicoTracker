#include <stdint.h>
#include <stdio.h>

#include "../firmware/gps_math.h"

static int failures = 0;

static void expect_altitude(int32_t mm, uint16_t expected)
{
    uint16_t actual = gps_altitude_from_mm(mm);
    if (actual != expected) {
        fprintf(stderr, "altitude %ld mm: got %u, expected %u\n",
                (long)mm, (unsigned)actual, (unsigned)expected);
        failures++;
    }
}

int main(void)
{
    expect_altitude(-1000, 1U);
    expect_altitude(0, 1U);
    expect_altitude(999, 0U);
    expect_altitude(1000, 1U);
    expect_altitude(12345678, 12345U);
    expect_altitude(49999999, 49999U);
    expect_altitude(50000000, 50000U);
    expect_altitude(60000000, 50000U);

    for (int32_t mm = 1; mm < 50000000L; mm += 1371111L) {
        expect_altitude(mm, (uint16_t)(mm / 1000L));
    }

    if (failures != 0) return 1;
    puts("GPS math tests passed");
    return 0;
}
