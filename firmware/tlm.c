#include <msp430.h>
#include <inttypes.h>
#include "main.h"
#include "tlm.h"
#include "si4060.h"
#include "hw.h"
#include "string.h"
#include "fix.h"

/* calculated sentence ID length, used for variable length buffer */
uint16_t tlm_sent_id_length;
uint16_t tlm_alt_length;

extern volatile uint16_t tlm_tick;
extern uint16_t tx_buf_rdy;
extern uint16_t tx_buf_length;
extern char tx_buf[TX_BUF_MAX_LENGTH];

extern struct gps_fix current_fix;

/*
 * tx_blips
 *
 * when called periodically (fast enough), transmits blips with ratio 1:5
 * checks the timer-tick flag for timing
 *
 */
void tx_blips(uint8_t reset) {
	static uint8_t count = 0;	/* keeps track of blip state */

	if (reset) {
		count = 0;
		return;
	}

	if (!tlm_tick)
		return;

	tlm_tick = 0;
	count++;
	switch (count) {
		case 1:
			P1OUT |= SI_DATA;
			P1OUT |= LED_A;
			break;
		case 1+BLIP_FACTOR:
			P1OUT &= ~SI_DATA;
			P1OUT &= ~LED_A;
			count--;
			break;
	}
}

/*
 * calculate_txbuf_checksum
 *
 * this routine calculates the 16bit checksum value used in the HAB protocol
 * it uses the MSP430 hardware CRC generator
 */
uint16_t calculate_txbuf_checksum(void) {
	int i;
	CRCINIRES = 0xffff;
	for (i = TX_BUF_CHECKSUM_BEGIN; i < TX_BUF_CHECKSUM_END; i++) {
		CRCDIRB_L = tx_buf[i];
	}
	return CRCINIRES;
}

/*
 * prepare_tx_buffer
 *
 * fills tx_buf with telemetry values. this depends on the
 * GPS having a fix and telemetry being extracted before
 *
 * telemetry format:
 * - callsign
 * - sentence id
 * - time
 * - latitude
 * - longitude
 * - altitude
 * - available satellites
 * - voltage of the AAA cell
 * - MSP430 temperature
 */
void prepare_tx_buffer(void) {
	static uint16_t sent_id = 0;
	int i;
	uint16_t crc;

	sent_id++;
	tlm_sent_id_length = i16toav(sent_id, &tx_buf[TX_BUF_SENT_ID_START]);
	tx_buf[TX_BUF_SENT_ID_START + tlm_sent_id_length] = ',';

	i16toa(current_fix.hour, 2, &tx_buf[TX_BUF_TIME_START]);
	i16toa(current_fix.min, 2, &tx_buf[TX_BUF_TIME_START + 2]);
	i16toa(current_fix.sec, 2, &tx_buf[TX_BUF_TIME_START + 4]);
	tx_buf[TX_BUF_TIME_START + TIME_LENGTH] = ',';
	
	if (current_fix.lat > 0) {
		tx_buf[TX_BUF_LAT_START] = '+';
		i32toa(current_fix.lat, 9, &tx_buf[TX_BUF_LAT_START + 1]);
	} else {
		tx_buf[TX_BUF_LAT_START] = '-';
		i32toa(0 - current_fix.lat, 9, &tx_buf[TX_BUF_LAT_START + 1]);
	}
	/* copy fraction of degrees one character towards the end, insert dot */
	/* 012XXXXXX -> 012 XXXXXX */
	for (i = 8; i >= 3; i--) {
		tx_buf[TX_BUF_LAT_START + i + 1] = tx_buf[TX_BUF_LAT_START + i];	
	}
	tx_buf[TX_BUF_LAT_START + 3] = '.';
	tx_buf[TX_BUF_LAT_START + LAT_LENGTH + 1] = ',';

	if (current_fix.lon > 0) {
		tx_buf[TX_BUF_LON_START] = '+';
		i32toa(current_fix.lon, 10, &tx_buf[TX_BUF_LON_START + 1]);
	} else {
		tx_buf[TX_BUF_LON_START] = '-';
		i32toa(0 - current_fix.lon, 10, &tx_buf[TX_BUF_LON_START + 1]);
	}
	/* copy fraction of degrees one character towards the end, insert dot */
	/* 51XXXXXX -> 51 XXXXXX */
	for (i = 9; i >= 4; i--) {
		tx_buf[TX_BUF_LON_START + i + 1] = tx_buf[TX_BUF_LON_START + i];	
	}
	tx_buf[TX_BUF_LON_START + 4] = '.';
	tx_buf[TX_BUF_LON_START + LON_LENGTH + 1] = ',';
	
	tlm_alt_length = i16toav(current_fix.alt, &tx_buf[TX_BUF_ALT_START]);
	tx_buf[TX_BUF_ALT_START + tlm_alt_length] = ',';
	
	i16toa(current_fix.num_svs, SAT_LENGTH, &tx_buf[TX_BUF_SAT_START]);
	tx_buf[TX_BUF_SAT_START + SAT_LENGTH] = ',';
	
	i16toa(current_fix.voltage_bat, VOLT_LENGTH, &tx_buf[TX_BUF_VOLT_START]);
	tx_buf[TX_BUF_VOLT_START + VOLT_LENGTH] = ',';
	
	i16toa(current_fix.voltage_sol, VSOL_LENGTH, &tx_buf[TX_BUF_VSOL_START]);
	tx_buf[TX_BUF_VSOL_START + VSOL_LENGTH] = ',';

	if (current_fix.temperature_int < 0) {
		tx_buf[TX_BUF_TEMP_START] = '-';
		i16toa(0 - current_fix.temperature_int, TEMP_LENGTH, &tx_buf[TX_BUF_TEMP_START + 1]);
	} else {
		tx_buf[TX_BUF_TEMP_START] = '+';
		i16toa(current_fix.temperature_int, TEMP_LENGTH, &tx_buf[TX_BUF_TEMP_START + 1]);
	}
	
	tx_buf[TX_BUF_TEMP_START + TEMP_LENGTH + 1] = '*';

	crc = calculate_txbuf_checksum();
	i16tox(crc, &tx_buf[TX_BUF_CHECKSUM_START]);
	for (i = 0; i < TX_BUF_POSTFIX_LENGTH; i++)
		tx_buf[TX_BUF_POSTFIX_START + i] = TX_BUF_POSTFIX[i];

	tx_buf_length = TX_BUF_FRAME_END;
	/* trigger transmission */
	tx_buf_rdy = 1;
}

void tlm_init(void) {
	tx_buf_rdy = 1;
}

/*
 * init_tx_buffer
 *
 * helper routine to fill the TX buffer with "x"es - if any of those get transmitted,
 * the field handling is not correct
 */
void init_tx_buffer(void) {
	uint16_t i;

	for (i = TX_BUF_START_OFFSET; i < TX_BUF_MAX_LENGTH; i++) {
		tx_buf[i] = 'x';
	}
}
