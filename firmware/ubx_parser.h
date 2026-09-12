#ifndef UBX_PARSER_H_
#define UBX_PARSER_H_

#include <stdint.h>

#include "ubx_protocol.h"

#define UBX_PARSER_CONTINUE 0U
#define UBX_PARSER_COMPLETE 1U
#define UBX_PARSER_ERROR    2U

struct ubx_parser {
    uint8_t expected_class;
    uint8_t expected_id;
    uint8_t *payload;
    uint16_t payload_capacity;
    uint16_t payload_length;
    uint16_t payload_count;
    uint16_t checksum;
    uint8_t received_checksum_a;
    uint8_t state;
};

enum ubx_parser_state {
    UBX_PARSE_SYNC_A,
    UBX_PARSE_SYNC_B,
    UBX_PARSE_CLASS,
    UBX_PARSE_ID,
    UBX_PARSE_LENGTH_A,
    UBX_PARSE_LENGTH_B,
    UBX_PARSE_PAYLOAD,
    UBX_PARSE_CHECKSUM_A,
    UBX_PARSE_CHECKSUM_B,
    UBX_PARSE_FAILED
};

static void ubx_parser_init(struct ubx_parser *parser, uint8_t expected_class,
                            uint8_t expected_id, uint8_t *payload,
                            uint16_t payload_capacity)
{
    parser->expected_class = expected_class;
    parser->expected_id = expected_id;
    parser->payload = payload;
    parser->payload_capacity = payload_capacity;
    parser->payload_length = 0;
    parser->payload_count = 0;
    parser->checksum = UBX_CHECKSUM_INITIAL;
    parser->received_checksum_a = 0;
    parser->state = UBX_PARSE_SYNC_A;
}

static uint8_t ubx_parser_push(struct ubx_parser *parser, uint8_t byte)
{
    switch (parser->state) {
    case UBX_PARSE_SYNC_A:
        if (byte == 0xB5U) parser->state = UBX_PARSE_SYNC_B;
        break;
    case UBX_PARSE_SYNC_B:
        parser->state = (byte == 0x62U) ? UBX_PARSE_CLASS : UBX_PARSE_SYNC_A;
        break;
    case UBX_PARSE_CLASS:
        if (byte != parser->expected_class) {
            parser->state = UBX_PARSE_SYNC_A;
            break;
        }
        parser->checksum = ubx_checksum_update(UBX_CHECKSUM_INITIAL, byte);
        parser->state = UBX_PARSE_ID;
        break;
    case UBX_PARSE_ID:
        if (byte != parser->expected_id) {
            parser->state = UBX_PARSE_SYNC_A;
            break;
        }
        parser->checksum = ubx_checksum_update(parser->checksum, byte);
        parser->state = UBX_PARSE_LENGTH_A;
        break;
    case UBX_PARSE_LENGTH_A:
        parser->payload_length = byte;
        parser->checksum = ubx_checksum_update(parser->checksum, byte);
        parser->state = UBX_PARSE_LENGTH_B;
        break;
    case UBX_PARSE_LENGTH_B:
        parser->payload_length |= (uint16_t)byte << 8;
        parser->checksum = ubx_checksum_update(parser->checksum, byte);
        if (parser->payload_length > parser->payload_capacity) {
            parser->state = UBX_PARSE_FAILED;
            return UBX_PARSER_ERROR;
        }
        parser->state = (parser->payload_length == 0U) ?
                        UBX_PARSE_CHECKSUM_A : UBX_PARSE_PAYLOAD;
        break;
    case UBX_PARSE_PAYLOAD:
        parser->payload[parser->payload_count++] = byte;
        parser->checksum = ubx_checksum_update(parser->checksum, byte);
        if (parser->payload_count == parser->payload_length)
            parser->state = UBX_PARSE_CHECKSUM_A;
        break;
    case UBX_PARSE_CHECKSUM_A:
        parser->received_checksum_a = byte;
        parser->state = UBX_PARSE_CHECKSUM_B;
        break;
    case UBX_PARSE_CHECKSUM_B:
        if (parser->received_checksum_a != UBX_CHECKSUM_A(parser->checksum) ||
            byte != UBX_CHECKSUM_B(parser->checksum)) {
            parser->state = UBX_PARSE_FAILED;
            return UBX_PARSER_ERROR;
        }
        return UBX_PARSER_COMPLETE;
    default:
        return UBX_PARSER_ERROR;
    }

    return UBX_PARSER_CONTINUE;
}

#endif /* UBX_PARSER_H_ */
