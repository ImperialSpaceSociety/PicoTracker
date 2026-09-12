#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../firmware/string.h"

static int failures = 0;

static void expect_u16(uint16_t value, uint8_t width, const char *expected)
{
    char output[8] = {0};
    i16toa(value, width, output);
    output[width] = '\0';
    if (strcmp(output, expected) != 0) {
        fprintf(stderr, "i16toa(%u, %u): got %s, expected %s\n",
                (unsigned)value, (unsigned)width, output, expected);
        failures++;
    }
}

int main(void)
{
    expect_u16(0, 2, "00");
    expect_u16(42, 2, "42");
    expect_u16(3175, 4, "3175");
    expect_u16(32767, 5, "32767");
    expect_u16(34464, 5, "34464");
    expect_u16(40000, 5, "40000");
    expect_u16(50000, 5, "50000");
    expect_u16(65535, 5, "65535");

    if (failures != 0) {
        return 1;
    }

    puts("number format tests passed");
    return 0;
}
