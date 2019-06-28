/*
* Telemetry strings and formatting
* 
* for Pico Balloon Tracker using HC12 radio module and GPS
* HC12 Module with STM8S003F3 processor and silabs Si4463 Radio
*  
* Derived Work Copyright (c) 2018 Imperial College Space Society
* From original work Copyright (C) 2014  Richard Meadows <richardeoin>
*
*
* Permission is hereby granted, free of charge, to any person obtaining
* a copy of this software and associated documentation files (the
* "Software"), to deal in the Software without restriction, including
* without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to
* permit persons to whom the Software is furnished to do so, subject to
* the following conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
* LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
* OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
* WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <string.h>
#include <iostm8s003f3.h>
#include <stdint.h>
#include <stdbool.h>
#include <intrinsics.h>
#include "HC12Board.h"
#include "telemetry.h"
#include "rtty.h"
#include "pips.h"
#include "si_trx.h"
#include "si_trx_defs.h"
#include "main.h"
#include "string.h"
#include "fix.h"

#define TIMER1_PRESCALE  (HSCLK_FREQUENCY/1000)

/* A lot of work for the telemetry and gps communication is taken from 
* https://github.com/thasti/utrak
*/



extern volatile uint16_t tlm_tick;
extern uint16_t tx_buf_rdy;
extern uint16_t tx_buf_length;
extern char tx_buf[TX_BUF_MAX_LENGTH];
extern char telemetry_string[TX_BUF_MAX_LENGTH];
extern struct gps_fix current_fix;

/* calculated sentence ID length, used for variable length buffer */
uint16_t tlm_sent_id_length;
uint16_t tlm_alt_length;

/**
* TELEMETRY OUTPUT
* =============================================================================
*/

/**
* The type of telemetry we're currently outputting
*/
enum telemetry_t telemetry_type;
/**
* Current output
*/
int8_t telemetry_string_length = 0;
/**
* Where we are in the current output
*/
int8_t telemetry_index;
/**
* Is the radio currently on?
*/
uint8_t radio_on = 0;

/**
* Returns 1 if we're currently outputting.
*/
int telemetry_active(void) {
	return (telemetry_string_length > 0);
}


/**
* Starts telemetry output
*
* Returns 0 on success, 1 if already active
*/
int telemetry_start(enum telemetry_t type, int8_t length) {
	if (!telemetry_active()) {
		
		/* Initialise */
		telemetry_type = type;
		telemetry_index = 0;
		telemetry_string_length = length;
		
		/* Setup timer tick */
		switch(telemetry_type) {
		case TELEMETRY_RTTY:
			timer1_tick_init(RTTY_BIT_MS);
			break;
		case TELEMETRY_PIPS:
			timer1_tick_init(PIPS_RATE_MS);
			break;
		}
		return 0; /* Success */
	} else {
		return 1; /* Already active */
	}
}


uint8_t is_telemetry_finished(void) {
	if (telemetry_index >= telemetry_string_length) {
		/* All done, deactivate */
		telemetry_string_length = 0;
		
		/* Turn radio off */
		if (radio_on) {
			si_trx_off(); radio_on = 0;
		}
		
		/* De-init timer */
		timer1_tick_deinit();
		
		return 1;
	}
	return 0;
}



/**
* Called at the telemetry mode's baud rate
*/
void telemetry_tick(void) {
	if (telemetry_active()) {
		switch (telemetry_type) {
			
		case TELEMETRY_RTTY: /* ---- ---- A character mode */
			if (!radio_on) {
				/* RTTY: We use the modem offset to modulate */
				si_trx_on(SI_MODEM_MOD_TYPE_CW, 1);
				radio_on = 1;
				rtty_preamble();
			}
			
			if (!rtty_tick()) {
				/* Transmission Finished */
				if (is_telemetry_finished()) return;
				
				/* Let's start again */
				uint8_t data = tx_buf[telemetry_index]; 
				
				telemetry_index++;
				rtty_start(data);
			}
			
			break;
			
		case TELEMETRY_PIPS: /* ---- ---- A pips mode! */
			
			if (!radio_on) { /* Turn on */
				/* Pips: Cw */
				si_trx_on(SI_MODEM_MOD_TYPE_CW, 1);
				radio_on = 1;
				timer1_tick_time(PIPS_LENGTH_MS);
				
			} else { /* Turn off */
				si_trx_off();
				radio_on = 0;
				timer1_tick_time(PIPS_RATE_MS);
				
				telemetry_index++;
				if (is_telemetry_finished())
					
					return;
			}
		}
	}
}


/**
* CYCLIC REDUNDANCY CHECK (CRC)
* =============================================================================
*/

/**
* CRC Function for the XMODEM protocol.
* http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html#gaca726c22a1900f9bad52594c8846115f
*/
uint16_t crc_xmodem_update(uint16_t crc, uint8_t data)
{
	int i;
	
	crc = crc ^ ((uint16_t)data << 8);
	for (i = 0; i < 8; i++) {
		if (crc & 0x8000) {
			crc = (crc << 1) ^ 0x1021;
		} else {
			crc <<= 1;
		}
	}
	
	return crc;
}

/**
* Calcuates the CRC checksum for a communications string
* See http://ukhas.org.uk/communication:protocol
*/
uint16_t calculate_txbuf_checksum(void)
{
	size_t i;
	uint16_t crc;
	crc = 0xFFFF;
	
	for (i = TX_BUF_CHECKSUM_BEGIN; i < TX_BUF_CHECKSUM_END; i++) {
		crc = crc_xmodem_update(crc, tx_buf[i]);
	}
	
	return crc;
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
* - voltage of the AAA cell(after boosting)
* - op status
* - temperature of radio

*/
void prepare_tx_buffer(void) {
	//static uint16_t sent_id = 0;
        static uint16_t sent_id = 1000;

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
	
	i16toa(current_fix.voltage_radio, VOLT_LENGTH, &tx_buf[TX_BUF_VOLT_START]);
	tx_buf[TX_BUF_VOLT_START + VOLT_LENGTH] = ',';
	
	i16toa(current_fix.op_status, OP_STAT_LENGTH, &tx_buf[TX_BUF_OP_STAT_START]);
	tx_buf[TX_BUF_OP_STAT_START + OP_STAT_LENGTH] = ',';
	
	if (current_fix.temp_radio < 0) {
		tx_buf[TX_BUF_TEMP_START] = '-';
		i16toa(0 - current_fix.temp_radio, TEMP_LENGTH, &tx_buf[TX_BUF_TEMP_START + 1]);
	} else {
		tx_buf[TX_BUF_TEMP_START] = '+';
		i16toa(current_fix.temp_radio, TEMP_LENGTH, &tx_buf[TX_BUF_TEMP_START + 1]);
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


/**
* CLOCKING
* =============================================================================
*/

/**
* Initialises a timer interupt at the time period in ms
*
*/
void timer1_tick_init(uint16_t millisecs)
{
	
	__disable_interrupt();
	/* Configure Timer 1 */
	
	TIM1_PSCRH = TIMER1_PRESCALE >> 8 ;       //  Prescaler 
	TIM1_PSCRL = TIMER1_PRESCALE & 0xff ;       //  
	
	TIM1_ARRH =millisecs >> 8 ;       //  Count Register
	TIM1_ARRL =millisecs & 0xff ;       //  
	
	
	/* Enable Interrupt */
	TIM1_IER_UIE = 1;       //  Enable the update interrupts.
	
	/* Enable Timer */
	TIM1_CR1_CEN = 1;       //  Enable the timer.
	__enable_interrupt();
}



/**
* Changes the timer1 time
*/
void timer1_tick_time(uint16_t millisecs)
{
		
	TIM1_ARRH =millisecs >> 8 ;       //  Count Register
	TIM1_ARRL =millisecs & 0xff ;       //  
	
}


/* Disables the timer
*/
void timer1_tick_deinit()
{
	__disable_interrupt();
	TIM1_CR1_CEN = 0;       //  Disable the timer.
	TIM1_IER_UIE = 0;       //  Disable the update interrupts.
	__enable_interrupt();
}
/**
* Timer 1 Interrupt called at the symbol rate
*/
#pragma vector = TIM1_OVR_UIF_vector
__interrupt void TIM1_UPD_OVF_IRQHandler(void)
{
	telemetry_tick();
	TIM1_SR1_UIF = 0;               //  Reset the interrupt otherwise it will fire again   
	
}


