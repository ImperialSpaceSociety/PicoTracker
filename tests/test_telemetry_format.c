#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../firmware/telemetry_format.h"

static int failures = 0;

static void expect_field(const char *label, const char *actual,
                         const char *expected, uint8_t length)
{
    if (memcmp(actual, expected, length) != 0) {
        fprintf(stderr, "%s: got %.*s, expected %.*s\n",
                label, length, actual, length, expected);
        failures++;
    }
}

int main(void)
{
    char latitude[10];
    char longitude[11];
    char temperature[3];

    telemetry_format_latitude(515362480, latitude);
    expect_field("latitude positive", latitude, "+51.536248", 10);
    telemetry_format_latitude(-515362480, latitude);
    expect_field("latitude negative", latitude, "-51.536248", 10);
    telemetry_format_latitude(0, latitude);
    expect_field("latitude zero", latitude, "+00.000000", 10);

    telemetry_format_longitude(-2073530, longitude);
    expect_field("longitude negative", longitude, "-000.207353", 11);
    telemetry_format_longitude(1800000000, longitude);
    expect_field("longitude limit", longitude, "+180.000000", 11);

    telemetry_format_temperature(-18, temperature);
    expect_field("temperature negative", temperature, "-18", 3);
    telemetry_format_temperature(7, temperature);
    expect_field("temperature positive", temperature, "+07", 3);

    if (failures != 0) return 1;
    puts("telemetry format tests passed");
    return 0;
}
