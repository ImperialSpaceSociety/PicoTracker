/*
 * main tracker software
 *
 * Stefan Biereigel
 *
 */



#ifndef MAIN_H_
#define MAIN_H_


////////////////////////////////////////////////////////////////////////////////
/* Here is the most important configuration section                           */
////////////////////////////////////////////////////////////////////////////////

/* payload name */
#define PAYLOAD_NAME "ICSPACE13"

/* radio definitions */
#define RADIO_FREQUENCY	434600000
#define RADIO_POWER	0x10
#define RTTY_CHANNEL_SPACING	50         // Corresponds to 745Hz spacing

/* setting up baud rates */

/* 50 baud RTTY */
#define RTTY_BIT_MS		19         // Miliseconds - 1 of bit period
/* 200 baud RTTY */
//#define RTTY_BIT_MS		4         // Miliseconds - 1 of bit period
////////////////////////////////////////////////////////////////////////////////

/* telemetry string prefix for RX syncronisation */
#define SYNC_PREFIX		"  $$"
/* telemetry string postfix for tlm parser */
#define TX_BUF_POSTFIX		"\n\n"

/* number of idle bits to transmit before beginning tlm string tx */
#define NUM_IDLE_BITS	32

/* length of non-GPS-telemetry fields */
#define LAT_LENGTH		9
#define LON_LENGTH		10
#define TIME_LENGTH		6
#define SAT_LENGTH		2
#define ALT_LENGTH_MAX		5
#define VOLT_LENGTH		4
#define OP_STAT_LENGTH		4
#define TEMP_LENGTH		2
#define CHECKSUM_LENGTH		4
/* sentence id is variable length */
#define SENT_ID_LENGTH_MAX	5

#define TX_BUF_POSTFIX_LENGTH	sizeof(TX_BUF_POSTFIX) - 1
/* offset from buffer start to telemetry data */
#define TX_BUF_START_OFFSET 	sizeof(SYNC_PREFIX "$$" PAYLOAD_NAME ",") - 1
#define TX_BUF_SENT_ID_START	TX_BUF_START_OFFSET
#define TX_BUF_TIME_START	TX_BUF_SENT_ID_START + tlm_sent_id_length + 1
#define TX_BUF_LAT_START	TX_BUF_TIME_START + TIME_LENGTH + 1
/* lat and lon fields are one char longer for +/- */
#define TX_BUF_LON_START	TX_BUF_LAT_START + LAT_LENGTH + 1 + 1
#define TX_BUF_ALT_START	TX_BUF_LON_START + LON_LENGTH + 1 + 1
#define TX_BUF_SAT_START	TX_BUF_ALT_START + tlm_alt_length + 1
#define TX_BUF_VOLT_START	TX_BUF_SAT_START + SAT_LENGTH + 1
#define TX_BUF_OP_STAT_START	TX_BUF_VOLT_START + VOLT_LENGTH + 1
/* temperature field is one char longer for +/- */
#define TX_BUF_TEMP_START	TX_BUF_OP_STAT_START + OP_STAT_LENGTH + 1
#define TX_BUF_CHECKSUM_START	TX_BUF_TEMP_START + TEMP_LENGTH + 1 + 1
#define TX_BUF_POSTFIX_START	TX_BUF_CHECKSUM_START + CHECKSUM_LENGTH
#define TX_BUF_FRAME_END	TX_BUF_POSTFIX_START + sizeof(TX_BUF_POSTFIX) - 1;
/* checksum parameters */
#define TX_BUF_CHECKSUM_BEGIN	sizeof(SYNC_PREFIX "$$") - 1
#define TX_BUF_CHECKSUM_END	TX_BUF_CHECKSUM_START - 1

#define TX_BUF_MAX_LENGTH	sizeof(SYNC_PREFIX "$$" PAYLOAD_NAME) + 1 + \
				SENT_ID_LENGTH_MAX + 1 + TIME_LENGTH + 1 + LAT_LENGTH + 1 + LON_LENGTH + 1 + \
				ALT_LENGTH_MAX + 1 + SAT_LENGTH + 1 + VOLT_LENGTH + 1 + OP_STAT_LENGTH + 1 + TEMP_LENGTH + \
				sizeof("*") + CHECKSUM_LENGTH + TX_BUF_POSTFIX_LENGTH





#endif
