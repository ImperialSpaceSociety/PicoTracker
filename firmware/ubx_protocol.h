#ifndef UBX_PROTOCOL_H_
#define UBX_PROTOCOL_H_

#include <stdint.h>

#define UBX_CHECKSUM_INITIAL 0U
#define UBX_CHECKSUM_A(checksum) ((uint8_t)((checksum) & 0xFFU))
#define UBX_CHECKSUM_B(checksum) ((uint8_t)(((checksum) >> 8) & 0xFFU))

#define UBX_NAV_PVT_FIX_TYPE_OFFSET 20U
#define UBX_NAV_PVT_FLAGS_OFFSET 21U
#define UBX_NAV_PVT_FIX_TYPE_3D 3U
#define UBX_NAV_PVT_FLAG_GNSS_FIX_OK 0x01U

static uint16_t ubx_checksum_update(uint16_t checksum, uint8_t byte)
{
    uint8_t ck_a = UBX_CHECKSUM_A(checksum);
    uint8_t ck_b = UBX_CHECKSUM_B(checksum);

    ck_a = (uint8_t)(ck_a + byte);
    ck_b = (uint8_t)(ck_b + ck_a);

    return (uint16_t)(((uint16_t)ck_b << 8) | (uint16_t)ck_a);
}

static uint8_t ubx_nav_pvt_fix_is_usable(uint8_t fix_type, uint8_t flags)
{
    return (uint8_t)((fix_type == UBX_NAV_PVT_FIX_TYPE_3D) &&
                     ((flags & UBX_NAV_PVT_FLAG_GNSS_FIX_OK) != 0U));
}

#endif /* UBX_PROTOCOL_H_ */
