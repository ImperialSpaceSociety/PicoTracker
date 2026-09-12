#include <stdint.h>
#include <stdio.h>

#include "../firmware/radio_measurements.h"

static int failures = 0;

static void expect_temperature(uint16_t raw, int16_t expected)
{
    int16_t actual = si_trx_temperature_from_raw(raw);
    if (actual != expected) {
        fprintf(stderr, "temperature raw %u: got %d, expected %d\n",
                raw, actual, expected);
        failures++;
    }
}

int main(void)
{
    expect_temperature(0, -293);
    expect_temperature(1334, -1);
    expect_temperature(1335, 0);
    expect_temperature(2047, 156);

    for (uint16_t raw = 0; raw <= 2047U; raw++) {
        int16_t expected = (int16_t)(((uint32_t)899U * raw) / 4096U) - 293;
        expect_temperature(raw, expected);
    }

    if (failures != 0) return 1;
    puts("radio measurement tests passed");
    return 0;
}
