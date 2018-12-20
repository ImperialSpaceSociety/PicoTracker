
#include <inttypes.h>
#include "main.h"
#include "tlm.h"
#include "string.h"
#include "fix.h"

/* calculated sentence ID length, used for variable length buffer */
//uint16_t tlm_sent_id_length;
//uint16_t tlm_alt_length;

extern volatile uint16_t tlm_tick;
extern uint16_t tx_buf_rdy;
extern uint16_t tx_buf_length;
extern char tx_buf[TX_BUF_MAX_LENGTH];

extern struct gps_fix current_fix;



