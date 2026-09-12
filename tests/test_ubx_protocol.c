#include <stdint.h>
#include <stdio.h>

#include "../firmware/ubx_protocol.h"

static int failures = 0;

static void expect_checksum(const uint8_t *data, uint8_t length,
                            uint8_t expected_a, uint8_t expected_b)
{
    uint16_t checksum = UBX_CHECKSUM_INITIAL;
    uint8_t i;

    for (i = 0; i < length; i++) {
        checksum = ubx_checksum_update(checksum, data[i]);
    }

    if (UBX_CHECKSUM_A(checksum) != expected_a ||
        UBX_CHECKSUM_B(checksum) != expected_b) {
        fprintf(stderr, "checksum: got %02X %02X, expected %02X %02X\n",
                UBX_CHECKSUM_A(checksum), UBX_CHECKSUM_B(checksum),
                expected_a, expected_b);
        failures++;
    }
}

int main(void)
{
    const uint8_t nav_pvt_poll[] = {0x01, 0x07, 0x00, 0x00};
    const uint8_t cfg_rxm_psm[] = {0x06, 0x11, 0x02, 0x00, 0x08, 0x01};
    const uint8_t cfg_rxm_continuous[] = {0x06, 0x11, 0x02, 0x00, 0x08, 0x00};

    expect_checksum(nav_pvt_poll, sizeof(nav_pvt_poll), 0x08, 0x19);
    expect_checksum(cfg_rxm_psm, sizeof(cfg_rxm_psm), 0x22, 0x92);
    expect_checksum(cfg_rxm_continuous, sizeof(cfg_rxm_continuous), 0x21, 0x91);

    if (failures != 0) {
        return 1;
    }

    puts("UBX protocol tests passed");
    return 0;
}
