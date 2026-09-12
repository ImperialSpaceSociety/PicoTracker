#ifndef TELEMETRY_FORMAT_H_
#define TELEMETRY_FORMAT_H_

#include <stdint.h>

#include "number_format.h"

static uint32_t telemetry_abs_i32(int32_t value)
{
    return (value < 0) ? (uint32_t)(-(value + 1)) + 1U : (uint32_t)value;
}

static void telemetry_format_latitude(int32_t latitude, char *out)
{
    int8_t i;

    out[0] = (latitude < 0) ? '-' : '+';
    i32toa(telemetry_abs_i32(latitude), 9, &out[1]);
    for (i = 8; i >= 3; i--) out[i + 1] = out[i];
    out[3] = '.';
}

static void telemetry_format_longitude(int32_t longitude, char *out)
{
    int8_t i;

    out[0] = (longitude < 0) ? '-' : '+';
    i32toa(telemetry_abs_i32(longitude), 10, &out[1]);
    for (i = 9; i >= 4; i--) out[i + 1] = out[i];
    out[4] = '.';
}

static void telemetry_format_temperature(int16_t temperature, char *out)
{
    uint16_t magnitude = (temperature < 0) ?
                         (uint16_t)(-(temperature + 1)) + 1U :
                         (uint16_t)temperature;

    if (magnitude > 99U) magnitude = 99U;
    out[0] = (temperature < 0) ? '-' : '+';
    i16toa(magnitude, 2, &out[1]);
}

#endif /* TELEMETRY_FORMAT_H_ */
