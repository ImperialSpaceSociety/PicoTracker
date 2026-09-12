#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../firmware/telemetry_crc.h"

static int failures = 0;

static void expect_crc(const char *text, uint16_t expected)
{
    uint16_t actual = telemetry_crc((const uint8_t *)text, (uint16_t)strlen(text));
    if (actual != expected) {
        fprintf(stderr, "crc: got %04X, expected %04X for %s\n", actual, expected, text);
        failures++;
    }
}

int main(void)
{
    expect_crc("123456789", 0x29B1U);
    expect_crc("ICSPACE14,35,224122,+51.536248,-000.207353,31,04,3175,0004,+18", 0xB3A7U);
    expect_crc("", TELEMETRY_CRC_INITIAL);

    if (failures != 0) return 1;
    puts("telemetry CRC tests passed");
    return 0;
}
