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

static void expect_fix_usable(uint8_t fix_type, uint8_t flags, uint8_t expected)
{
    uint8_t actual = ubx_nav_pvt_fix_is_usable(fix_type, flags);

    if (actual != expected) {
        fprintf(stderr, "fix usability: type %u flags 0x%02X got %u, expected %u\n",
                (unsigned)fix_type, (unsigned)flags,
                (unsigned)actual, (unsigned)expected);
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

    expect_fix_usable(UBX_NAV_PVT_FIX_TYPE_3D, UBX_NAV_PVT_FLAG_GNSS_FIX_OK, 1);
    expect_fix_usable(UBX_NAV_PVT_FIX_TYPE_3D, 0x00, 0);
    expect_fix_usable(2, UBX_NAV_PVT_FLAG_GNSS_FIX_OK, 0);
    expect_fix_usable(4, UBX_NAV_PVT_FLAG_GNSS_FIX_OK, 0);
    expect_fix_usable(5, UBX_NAV_PVT_FLAG_GNSS_FIX_OK, 0);

    if (failures != 0) {
        return 1;
    }

    puts("UBX protocol tests passed");
    return 0;
}
