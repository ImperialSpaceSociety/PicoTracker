#include <stdint.h>
#include <stdio.h>

uint16_t tlm_sent_id_length;
uint16_t tlm_alt_length;

#include "../firmware/main.h"

int main(void)
{
    tlm_sent_id_length = SENT_ID_LENGTH_MAX;
    tlm_alt_length = ALT_LENGTH_MAX;

    if (TX_BUF_FRAME_END > TX_BUF_MAX_LENGTH) {
        fprintf(stderr, "maximum telemetry frame exceeds buffer: %lu > %lu\n",
                (unsigned long)TX_BUF_FRAME_END,
                (unsigned long)TX_BUF_MAX_LENGTH);
        return 1;
    }

    if (TX_BUF_FRAME_END != TX_BUF_MAX_LENGTH) {
        fprintf(stderr, "maximum telemetry frame does not fill expected capacity\n");
        return 1;
    }

    puts("telemetry layout tests passed");
    return 0;
}
