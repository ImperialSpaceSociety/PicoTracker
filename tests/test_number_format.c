#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../firmware/number_format.h"

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

static void expect_u16_av(uint16_t value, const char *expected)
{
    char output[8] = {0};
    uint8_t len = i16toav(value, output);
    output[len] = '\0';
    if (strcmp(output, expected) != 0) {
        fprintf(stderr, "i16toav(%u): got %s, expected %s\n",
                (unsigned)value, output, expected);
        failures++;
    }
    size_t expected_len = strlen(expected);
    if (len != expected_len) {
        fprintf(stderr, "i16toav(%u): returned length %u, expected %zu\n",
                (unsigned)value, (unsigned)len, expected_len);
        failures++;
    }
}

static void expect_u32(uint32_t value, uint8_t width, const char *expected)
{
    char output[12] = {0};
    i32toa(value, width, output);
    output[width] = '\0';
    if (strcmp(output, expected) != 0) {
        fprintf(stderr, "i32toa(%lu, %u): got %s, expected %s\n",
                (unsigned long)value, (unsigned)width, output, expected);
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

    expect_u16_av(0, "0");
    expect_u16_av(9, "9");
    expect_u16_av(10, "10");
    expect_u16_av(99, "99");
    expect_u16_av(100, "100");
    expect_u16_av(999, "999");
    expect_u16_av(1000, "1000");
    expect_u16_av(9999, "9999");
    expect_u16_av(10000, "10000");
    expect_u16_av(65535, "65535");

    expect_u32(0, 9, "000000000");
    expect_u32(515362480, 9, "515362480");
    expect_u32(999999999, 10, "0999999999");
    expect_u32(1410065407UL, 10, "1410065407");
    expect_u32(1410065408UL, 10, "1410065408");
    expect_u32(1500000000UL, 10, "1500000000");
    expect_u32(1799999999UL, 10, "1799999999");
    expect_u32(1800000000UL, 10, "1800000000");

    if (failures != 0) {
        return 1;
    }

    puts("number format tests passed");
    return 0;
}
