#include <stdint.h>
#include <stdio.h>

#include "../firmware/ubx_ack_parser.h"

static int failures = 0;

static uint8_t feed(const uint8_t *packet, uint8_t length)
{
    struct ubx_ack_parser parser;
    uint8_t result = UBX_ACK_CONTINUE;
    uint8_t i;

    ubx_ack_parser_init(&parser, 0x06U, 0x00U);
    for (i = 0; i < length && result == UBX_ACK_CONTINUE; i++) {
        result = ubx_ack_parser_push(&parser, packet[i]);
    }
    return result;
}

static void expect(const char *name, const uint8_t *packet, uint8_t length,
                   uint8_t expected)
{
    uint8_t actual = feed(packet, length);
    if (actual != expected) {
        fprintf(stderr, "%s: got %u expected %u\n", name, actual, expected);
        failures++;
    }
}

int main(void)
{
    const uint8_t ack[] = {0xB5,0x62,0x05,0x01,0x02,0x00,0x06,0x00,0x0E,0x37};
    const uint8_t nak[] = {0xB5,0x62,0x05,0x00,0x02,0x00,0x06,0x00,0x0D,0x32};
    const uint8_t hybrid[] = {0xB5,0x62,0x05,0x01,0x02,0x00,0x06,0x00,0x0D,0x32};
    const uint8_t wrong_payload[] = {0xB5,0x62,0x05,0x01,0x02,0x00,0x06,0x01,0x0F,0x38};

    expect("ACK", ack, sizeof(ack), UBX_ACK_ACCEPTED);
    expect("NAK", nak, sizeof(nak), UBX_NAK_ACCEPTED);
    expect("hybrid checksum", hybrid, sizeof(hybrid), UBX_ACK_ERROR);
    expect("wrong payload", wrong_payload, sizeof(wrong_payload), UBX_ACK_ERROR);

    if (!ubx_nav_pvt_fix_is_usable(UBX_NAV_PVT_FIX_TYPE_3D,
                                   UBX_NAV_PVT_FLAG_GNSS_FIX_OK)) {
        failures++;
    }

    if (failures != 0) return 1;
    puts("UBX ACK parser tests passed");
    return 0;
}
