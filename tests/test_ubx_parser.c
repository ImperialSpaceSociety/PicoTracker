#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../firmware/ubx_parser.h"

static int failures = 0;

static uint16_t build_packet(uint8_t class_id, uint8_t msg_id,
                             const uint8_t *payload, uint16_t payload_length,
                             uint8_t *packet)
{
    uint16_t checksum = UBX_CHECKSUM_INITIAL;
    uint16_t i;

    packet[0] = 0xB5U;
    packet[1] = 0x62U;
    packet[2] = class_id;
    packet[3] = msg_id;
    packet[4] = (uint8_t)(payload_length & 0xFFU);
    packet[5] = (uint8_t)(payload_length >> 8);
    memcpy(&packet[6], payload, payload_length);

    for (i = 2; i < (uint16_t)(6U + payload_length); i++)
        checksum = ubx_checksum_update(checksum, packet[i]);

    packet[6 + payload_length] = UBX_CHECKSUM_A(checksum);
    packet[7 + payload_length] = UBX_CHECKSUM_B(checksum);
    return (uint16_t)(payload_length + 8U);
}

static uint8_t feed_packet(struct ubx_parser *parser,
                           const uint8_t *packet, uint16_t length)
{
    uint16_t i;
    uint8_t result = UBX_PARSER_CONTINUE;

    for (i = 0; i < length; i++) {
        result = ubx_parser_push(parser, packet[i]);
        if (result != UBX_PARSER_CONTINUE) break;
    }
    return result;
}

int main(void)
{
    uint8_t payload[92] = {0};
    uint8_t decoded[92] = {0};
    uint8_t packet[100];
    uint16_t packet_length;
    struct ubx_parser parser;

    payload[UBX_NAV_PVT_FIX_TYPE_OFFSET] = UBX_NAV_PVT_FIX_TYPE_3D;
    payload[UBX_NAV_PVT_FLAGS_OFFSET] = UBX_NAV_PVT_FLAG_GNSS_FIX_OK;
    packet_length = build_packet(0x01U, 0x07U, payload, sizeof(payload), packet);

    ubx_parser_init(&parser, 0x01U, 0x07U, decoded, sizeof(decoded));
    if (feed_packet(&parser, packet, packet_length) != UBX_PARSER_COMPLETE ||
        parser.payload_length != sizeof(payload) || memcmp(payload, decoded, sizeof(payload)) != 0 ||
        !ubx_nav_pvt_fix_is_usable(decoded[UBX_NAV_PVT_FIX_TYPE_OFFSET],
                                   decoded[UBX_NAV_PVT_FLAGS_OFFSET]))
        failures++;

    packet[packet_length - 1U] ^= 0x01U;
    ubx_parser_init(&parser, 0x01U, 0x07U, decoded, sizeof(decoded));
    if (feed_packet(&parser, packet, packet_length) != UBX_PARSER_ERROR) failures++;
    packet[packet_length - 1U] ^= 0x01U;

    packet[4] = 93U;
    packet[5] = 0U;
    ubx_parser_init(&parser, 0x01U, 0x07U, decoded, sizeof(decoded));
    if (feed_packet(&parser, packet, 6U) != UBX_PARSER_ERROR) failures++;

    if (failures != 0) {
        fprintf(stderr, "UBX parser failures: %d\n", failures);
        return 1;
    }

    puts("UBX parser tests passed");
    return 0;
}
