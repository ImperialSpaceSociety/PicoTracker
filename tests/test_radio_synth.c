#include <stdint.h>
#include <stdio.h>

#include "../firmware/radio_synth.h"

static int failures = 0;

static void expect_params(uint32_t frequency, uint32_t xo, uint8_t outdiv,
                          uint8_t n, uint32_t m)
{
    struct si_trx_synth_params params;
    if (!si_trx_calculate_synth(frequency, xo, &params) ||
        params.outdiv != outdiv || params.n != n || params.m != m) {
        fprintf(stderr, "%lu Hz: got div=%u n=%u m=%lu, expected div=%u n=%u m=%lu\n",
                (unsigned long)frequency, (unsigned)params.outdiv,
                (unsigned)params.n, (unsigned long)params.m,
                (unsigned)outdiv, (unsigned)n, (unsigned long)m);
        failures++;
    }
}

int main(void)
{
    struct si_trx_synth_params params;

    expect_params(434570000UL, 32000000UL, 8U, 53U, 692715UL);
    expect_params(119000000UL, 32000000UL, 24U, 43U, 851968UL);
    expect_params(177000000UL, 32000000UL, 16U, 43U, 655360UL);
    expect_params(353000000UL, 32000000UL, 8U, 43U, 589824UL);
    expect_params(525000000UL, 32000000UL, 6U, 48U, 638976UL);
    expect_params(705000000UL, 32000000UL, 4U, 43U, 557056UL);
    expect_params(1050000000UL, 32000000UL, 4U, 64U, 851968UL);

    if (si_trx_calculate_synth(118999999UL, 32000000UL, &params)) failures++;
    if (si_trx_calculate_synth(1050000001UL, 32000000UL, &params)) failures++;
    if (si_trx_calculate_synth(434570000UL, 0U, &params)) failures++;

    if (failures != 0) return 1;
    puts("radio synthesizer tests passed");
    return 0;
}
