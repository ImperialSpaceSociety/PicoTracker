#ifndef STATUS_WORD_H_
#define STATUS_WORD_H_

#include <stdint.h>

#define GPS_FIX_ATTEMPTS_MAX       0x0FU
#define OP_STATUS_ERROR_MASK       0x03U
#define OP_STATUS_CFG_SHIFT        2U
#define OP_STATUS_FIX_SHIFT        4U
#define OP_STATUS_OK               0U
#define OP_STATUS_TRANSIENT_ERROR  1U
#define OP_STATUS_RETRY_EXHAUSTED  2U
#define OP_STATUS_DEGRADED         3U
#define OP_STATUS_MEASUREMENT_ERROR 0x0100U

static uint16_t gps_status_pack(uint8_t fix_attempts, uint8_t config_status,
                                uint8_t poll_status)
{
    return (uint16_t)(((uint16_t)(fix_attempts & GPS_FIX_ATTEMPTS_MAX) << OP_STATUS_FIX_SHIFT) |
                      ((uint16_t)(config_status & OP_STATUS_ERROR_MASK) << OP_STATUS_CFG_SHIFT) |
                      (uint16_t)(poll_status & OP_STATUS_ERROR_MASK));
}

#endif /* STATUS_WORD_H_ */
