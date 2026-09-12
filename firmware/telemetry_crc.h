#ifndef TELEMETRY_CRC_H_
#define TELEMETRY_CRC_H_

#include <stdint.h>

#define TELEMETRY_CRC_INITIAL 0xFFFFU
#define TELEMETRY_CRC_POLYNOMIAL 0x1021U

static uint16_t telemetry_crc_update(uint16_t crc, uint8_t data)
{
    uint8_t i;

    crc ^= (uint16_t)data << 8;
    for (i = 0; i < 8; i++) {
        crc = (crc & 0x8000U) ?
              (uint16_t)((crc << 1) ^ TELEMETRY_CRC_POLYNOMIAL) :
              (uint16_t)(crc << 1);
    }

    return crc;
}

static uint16_t telemetry_crc(const uint8_t *data, uint16_t length)
{
    uint16_t crc = TELEMETRY_CRC_INITIAL;
    uint16_t i;

    for (i = 0; i < length; i++) {
        crc = telemetry_crc_update(crc, data[i]);
    }

    return crc;
}

#endif /* TELEMETRY_CRC_H_ */
