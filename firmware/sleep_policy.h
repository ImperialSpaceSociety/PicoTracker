#ifndef SLEEP_POLICY_H_
#define SLEEP_POLICY_H_

#include <stdint.h>

#define HIGH_ALTITUDE_SLEEP_THRESHOLD_M 3000U

static uint8_t tracker_sleep_intervals_for_altitude(uint16_t altitude_m)
{
    return (altitude_m > HIGH_ALTITUDE_SLEEP_THRESHOLD_M) ? 2U : 1U;
}

#endif /* SLEEP_POLICY_H_ */
