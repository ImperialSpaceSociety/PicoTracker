#ifndef RADIO_SYNTH_H_
#define RADIO_SYNTH_H_

#include <stdint.h>

#define SI_TRX_FREQ_MIN_HZ 119000000UL
#define SI_TRX_FREQ_MAX_HZ 1050000000UL
#define SI_TRX_SYNTH_Q19    524288UL

struct si_trx_synth_params {
    uint8_t outdiv;
    uint8_t n;
    uint32_t m;
};

static uint32_t si_trx_fraction_q19(uint32_t remainder, uint32_t denominator)
{
    uint8_t i;
    uint32_t result = 0;

    for (i = 0; i < 19U; i++) {
        remainder <<= 1;
        result <<= 1;
        if (remainder >= denominator) {
            remainder -= denominator;
            result |= 1U;
        }
    }
    return result;
}

static uint8_t si_trx_calculate_synth(uint32_t frequency, uint32_t xo_frequency,
                                      struct si_trx_synth_params *params)
{
    uint32_t denominator;
    uint32_t scaled_frequency;
    uint32_t whole;
    uint32_t remainder;

    if (frequency < SI_TRX_FREQ_MIN_HZ || frequency > SI_TRX_FREQ_MAX_HZ ||
        xo_frequency == 0U) return 0U;

    if (frequency >= 705000000UL) params->outdiv = 4U;
    else if (frequency >= 525000000UL) params->outdiv = 6U;
    else if (frequency >= 353000000UL) params->outdiv = 8U;
    else if (frequency >= 239000000UL) params->outdiv = 12U;
    else if (frequency >= 177000000UL) params->outdiv = 16U;
    else params->outdiv = 24U;

    denominator = xo_frequency << 1;
    switch (params->outdiv) {
    case 4U:  scaled_frequency = frequency << 2; break;
    case 6U:  scaled_frequency = (frequency << 2) + (frequency << 1); break;
    case 8U:  scaled_frequency = frequency << 3; break;
    case 12U: scaled_frequency = (frequency << 3) + (frequency << 2); break;
    case 16U: scaled_frequency = frequency << 4; break;
    case 24U: scaled_frequency = (frequency << 4) + (frequency << 3); break;
    default: return 0U;
    }

    remainder = scaled_frequency;
    whole = 0U;
    while (remainder >= denominator) {
        remainder -= denominator;
        whole++;
        if (whole > 128U) return 0U;
    }
    if (whole == 0U) return 0U;

    params->n = (uint8_t)(whole - 1U);
    params->m = SI_TRX_SYNTH_Q19 + si_trx_fraction_q19(remainder, denominator);
    return (uint8_t)(params->m <= 0xFFFFFUL);
}

#endif /* RADIO_SYNTH_H_ */
