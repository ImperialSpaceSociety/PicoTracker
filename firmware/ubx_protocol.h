#ifndef UBX_PROTOCOL_H_
#define UBX_PROTOCOL_H_

#include <stdint.h>

#define UBX_CHECKSUM_INITIAL 0U
#define UBX_CHECKSUM_A(checksum) ((uint8_t)((checksum) & 0xFFU))
#define UBX_CHECKSUM_B(checksum) ((uint8_t)(((checksum) >> 8) & 0xFFU))

static uint16_t ubx_checksum_update(uint16_t checksum, uint8_t byte)
{
    uint8_t ck_a = UBX_CHECKSUM_A(checksum);
    uint8_t ck_b = UBX_CHECKSUM_B(checksum);

    ck_a = (uint8_t)(ck_a + byte);
    ck_b = (uint8_t)(ck_b + ck_a);

    return ((uint16_t)ck_b << 8) | ck_a;
}

#endif /* UBX_PROTOCOL_H_ */
