#ifndef UBX_ACK_PARSER_H_
#define UBX_ACK_PARSER_H_

#include <stdint.h>

#include "ubx_protocol.h"

#define UBX_ACK_CONTINUE 0U
#define UBX_ACK_ACCEPTED 1U
#define UBX_NAK_ACCEPTED 2U
#define UBX_ACK_ERROR    3U

struct ubx_ack_parser {
    uint8_t expected_class;
    uint8_t expected_id;
    uint8_t message_id;
    uint16_t checksum;
    uint8_t state;
};

enum ubx_ack_state {
    UBX_ACK_SYNC_A,
    UBX_ACK_SYNC_B,
    UBX_ACK_CLASS,
    UBX_ACK_ID,
    UBX_ACK_LENGTH_A,
    UBX_ACK_LENGTH_B,
    UBX_ACK_PAYLOAD_CLASS,
    UBX_ACK_PAYLOAD_ID,
    UBX_ACK_CHECKSUM_A,
    UBX_ACK_CHECKSUM_B
};

static void ubx_ack_parser_init(struct ubx_ack_parser *parser,
                                uint8_t expected_class, uint8_t expected_id)
{
    parser->expected_class = expected_class;
    parser->expected_id = expected_id;
    parser->message_id = 0;
    parser->checksum = UBX_CHECKSUM_INITIAL;
    parser->state = UBX_ACK_SYNC_A;
}

static uint8_t ubx_ack_parser_push(struct ubx_ack_parser *parser, uint8_t byte)
{
    switch (parser->state) {
    case UBX_ACK_SYNC_A:
        if (byte == 0xB5U) parser->state = UBX_ACK_SYNC_B;
        break;
    case UBX_ACK_SYNC_B:
        parser->state = (byte == 0x62U) ? UBX_ACK_CLASS : UBX_ACK_SYNC_A;
        break;
    case UBX_ACK_CLASS:
        if (byte != 0x05U) { parser->state = UBX_ACK_SYNC_A; break; }
        parser->checksum = ubx_checksum_update(UBX_CHECKSUM_INITIAL, byte);
        parser->state = UBX_ACK_ID;
        break;
    case UBX_ACK_ID:
        if (byte != 0x00U && byte != 0x01U) return UBX_ACK_ERROR;
        parser->message_id = byte;
        parser->checksum = ubx_checksum_update(parser->checksum, byte);
        parser->state = UBX_ACK_LENGTH_A;
        break;
    case UBX_ACK_LENGTH_A:
        if (byte != 0x02U) return UBX_ACK_ERROR;
        parser->checksum = ubx_checksum_update(parser->checksum, byte);
        parser->state = UBX_ACK_LENGTH_B;
        break;
    case UBX_ACK_LENGTH_B:
        if (byte != 0x00U) return UBX_ACK_ERROR;
        parser->checksum = ubx_checksum_update(parser->checksum, byte);
        parser->state = UBX_ACK_PAYLOAD_CLASS;
        break;
    case UBX_ACK_PAYLOAD_CLASS:
        if (byte != parser->expected_class) return UBX_ACK_ERROR;
        parser->checksum = ubx_checksum_update(parser->checksum, byte);
        parser->state = UBX_ACK_PAYLOAD_ID;
        break;
    case UBX_ACK_PAYLOAD_ID:
        if (byte != parser->expected_id) return UBX_ACK_ERROR;
        parser->checksum = ubx_checksum_update(parser->checksum, byte);
        parser->state = UBX_ACK_CHECKSUM_A;
        break;
    case UBX_ACK_CHECKSUM_A:
        if (byte != UBX_CHECKSUM_A(parser->checksum)) return UBX_ACK_ERROR;
        parser->state = UBX_ACK_CHECKSUM_B;
        break;
    case UBX_ACK_CHECKSUM_B:
        if (byte != UBX_CHECKSUM_B(parser->checksum)) return UBX_ACK_ERROR;
        return (parser->message_id == 0x01U) ? UBX_ACK_ACCEPTED : UBX_NAK_ACCEPTED;
    default:
        return UBX_ACK_ERROR;
    }
    return UBX_ACK_CONTINUE;
}

#endif /* UBX_ACK_PARSER_H_ */
