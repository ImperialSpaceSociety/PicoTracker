#ifndef GPS_MATH_H_
#define GPS_MATH_H_

#include <stdint.h>

static uint16_t gps_altitude_from_mm(int32_t altitude_mm)
{
    uint32_t remaining;
    uint16_t altitude_m = 0U;

    if (altitude_mm <= 0) return 1U;
    if (altitude_mm >= 50000000L) return 50000U;

    remaining = (uint32_t)altitude_mm;
    while (remaining >= 10000000UL) { remaining -= 10000000UL; altitude_m += 10000U; }
    while (remaining >= 1000000UL)  { remaining -= 1000000UL;  altitude_m += 1000U; }
    while (remaining >= 100000UL)   { remaining -= 100000UL;   altitude_m += 100U; }
    while (remaining >= 10000UL)    { remaining -= 10000UL;    altitude_m += 10U; }
    while (remaining >= 1000UL)     { remaining -= 1000UL;     altitude_m += 1U; }
    return altitude_m;
}

#endif /* GPS_MATH_H_ */
