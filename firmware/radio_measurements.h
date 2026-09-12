#ifndef RADIO_MEASUREMENTS_H_
#define RADIO_MEASUREMENTS_H_

#include <stdint.h>

static int16_t si_trx_temperature_from_raw(uint16_t raw_temperature)
{
    uint32_t raw = (uint32_t)raw_temperature;
    uint32_t scaled = (raw << 10) - (raw << 7) + (raw << 2) - raw;
    return (int16_t)((int32_t)(scaled >> 12) - 293);
}

#endif /* RADIO_MEASUREMENTS_H_ */
