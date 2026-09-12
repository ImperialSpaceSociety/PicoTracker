#ifndef RADIO_MEASUREMENTS_H_
#define RADIO_MEASUREMENTS_H_

#include <stdint.h>

static int16_t si_trx_temperature_from_raw(uint16_t raw_temperature)
{
    int32_t scaled_temperature = ((int32_t)899 * raw_temperature) / 4096;
    return (int16_t)(scaled_temperature - 293);
}

#endif /* RADIO_MEASUREMENTS_H_ */
