/*
 * Functions for the UBLOX 8 GPS
 * for Pico Balloon Tracker using HC12 radio module and GPS
 * HC12 Module with STM8S003F3 processor and silabs Si4463 Radio
 *  
 * Derived Work Copyright (c) 2018 Imperial College Space Society
 * From original work Copyright (C) 2014  Richard Meadows <richardeoin>
 * Also derived from https://github.com/thasti/utrak
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


#include <iostm8s003f3.h>
#include "gps.h"
#include <inttypes.h>
#include "fix.h"
#include <intrinsics.h>




/**
 Wonderful summary reference taken from: https://github.com/zoomx/stm8-samples/blob/master/blinky/blinky.c
 ********************* UART ********************
 * baud rate: regs UART_BRR1/2  !!!VERY STUPID!!!
 * f_{UART} = f_{master} / UART_DIV
 * if UART_DIV = 0xABCD then
 * 		UART_BRR1 = UART_DIV[11:4] = 0xBC;
 * 		UART_BRR2 = UART_DIV[15:12|3:0] = 0xAD
 *           registers
 * UART_SR:  | TXE | TC | RXNE | IDLE | OR/LHE | NF | FE | PE |
 * 		TXE: Transmit data register empty
 * 		TC: Transmission complete
 * 		RXNE: Read data register not empty
 * 		IDLE: IDLE line detected
 * 		OR: Overrun error / LHE: LIN Header Error (LIN slave mode)
 * 		NF: Noise flag
 * 		FE: Framing error
 * 		PE: Parity error
 * UART_DR: data register (when readed returns coming byte, when writed fills output shift register)
 * UART_BRR1 / UART_BRR2 - see upper
 * UART_CR1: | R8 | T8 | UARTD | M | WAKE | PCEN | PS | PIEN |
 * 		R8, T8 - ninth bit (in 9-bit mode)
 * 		UARTD: UART Disable (for low power consumption)
 * 		M: word length (0 - 8bits, 1 - 9bits)
 * 		WAKE: Wakeup method
 * 		PCEN: Parity control enable
 * 		PS: Parity selection (0 - even)
 * 		PIEN: Parity interrupt enable
 * UART_CR2: | TIEN | TCEN | RIEN | ILIEN | TEN | REN | RWU | SBK |
 * 		TIEN: Transmitter interrupt enable
 * 		TCIEN: Transmission complete interrupt enable
 * 		RIEN: Receiver interrupt enable
 * 		ILIEN: IDLE Line interrupt enable
 * 		TEN: Transmitter enable   <----------------------------------------
 * 		REN: Receiver enable      <----------------------------------------
 * 		RWU: Receiver wakeup
 * 		SBK: Send break
 * UART_CR3: | - | LINEN | STOP[1:0] | CLCEN | CPOL | CPHA | LBCL |
 *		LINEN: LIN mode enable
 * 		STOP: STOP bits
 * 		CLKEN: Clock enable (CLC pin)
 * 		CPOL: Clock polarity
 * 		CPHA: Clock phase
 * 		LBCL: Last bit clock pulse
 */
   
   
   
/* millisec delay at @16MHz
 * the 960 comes from the number of instructions to perform the do/while loop in 1 ms
 */
void delay_ms(unsigned long ms) {

	unsigned long cycles = 960 * ms;
	do
	{
		cycles--;
	}
	while(cycles > 0);
}

/**
 * UART Serial Port Functions
 * Setup the UART to run at 9600 baud, no parity, one stop bit, 8 data bits.
 * Important: This relies upon the system clock being set to run at 16 MHz.
 */


void InitialiseUART(void)
{
    //
    //  Clear the Idle Line Detected bit in the status rerister by a read
    //  to the UART1_SR register followed by a Read to the UART1_DR register.
    //
    unsigned char tmp = UART1_SR;
    unsigned char tmp1 = UART1_DR;
    //
    //  Reset the UART registers to the reset values.
    //
    UART1_CR1 = 0;
    UART1_CR2 = 0;
    UART1_CR3 = 0;
    UART1_CR4 = 0;
    UART1_CR5 = 0;
    UART1_GTR = 0;
    UART1_PSCR = 0;
    //
    //  Now setup the port to 9600,n,8,1.
    //
    UART1_CR1_M = 0;        //  8 Data bits.
    UART1_CR1_PCEN = 0;     //  Disable parity.
    UART1_CR3_STOP = 0;     //  1 stop bit.
    UART1_BRR2 = 0x02;      //  Set the baud rate registers to 9600 baud 
    UART1_BRR1 = 0x68;      //  based upon a 16 MHz system clock.
    //
    //  Disable the transmitter and receiver.
    //
    UART1_CR2_TEN = 0;      //  Disable transmit.
    UART1_CR2_REN = 0;      //  Disable receive.
    //
    //  Set the clock polarity, lock phase and last bit clock pulse.
    //
    UART1_CR3_CPOL = 1;
    UART1_CR3_CPHA = 1;
    UART1_CR3_LBCL = 1;
    //
    // Setup the Receive Interrupt
    //
    UART1_CR2_TIEN  = 0;     // Transmitter interrupt enable
    UART1_CR2_TCIEN = 0;     // Transmission complete interrupt enable
    UART1_CR2_RIEN  = 0;     //  Receiver interrupt disable
    UART1_CR2_ILIEN = 0;     //  IDLE Line interrupt enable
    //
    //  Turn on the UART transmit, receive and the UART clock.
    //
    UART1_CR2_TEN = 1;
    UART1_CR2_REN = 1;
   
}




/* 
 * gps_transmit_string
 *
 * transmits a command to the GPS
 */
void UART_send_buffer(char *cmd, uint8_t length) {
	uint8_t i;

	for (i = 0; i < length; i++) {
            while (!UART1_SR_TXE);   //  Wait for transmission to complete.
		UART1_DR = cmd[i];
	}
}

/* 
 * gps_receive_ack
 *
 * waits for transmission of an ACK/NAK message from the GPS.
 *
 * returns 1 if ACK was received, 0 if NAK was received or a timeout occured
 *
 */
uint8_t gps_receive_ack(uint8_t class_id, uint8_t msg_id) {    
        

	int match_count = 0;
	int msg_ack = 0;
	char rx_byte;
	char ack[] = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x00, 0x00};
	char nak[] = {0xB5, 0x62, 0x05, 0x00, 0x02, 0x00, 0x00, 0x00};
	ack[6] = class_id;
	nak[6] = class_id;
	ack[7] = msg_id;
	nak[7] = msg_id;
    uint16_t timeout;
	

	/* runs until ACK/NAK packet is received, or a timeout.*/

        
	while(1) {
		timeout = 0;
		while(!UART1_SR_RXNE){ // check if there is any data to be read.
		  if(timeout++ > UBX_CFG_TIMEOUT) return 0; // return no ack if timeout
		}
		rx_byte = UART1_DR;

		if (rx_byte == ack[match_count] || rx_byte == nak[match_count]) {
			  if (match_count == 3) {	/* test ACK/NAK byte */
					if (rx_byte == ack[match_count]) {
						msg_ack = 1;
					} 
					else {
						msg_ack = 0; //nak
					}
			  }
			  if (match_count == 7) { 
					return msg_ack;
			  }
		match_count++;
		} 
		else {
			  match_count = 0;
		}
	}
}

/* 
 * gps_disable_nmea_output
 *
 * disables all NMEA messages to be output from the GPS.
 * even though the parser can cope with NMEA messages and ignores them, it 
 * may save power to disable them completely.
 *
 * returns if ACKed by GPS
 *
 */
uint8_t gps_disable_nmea_output(void) {
	char nonmea[] = {
		0xB5, 0x62, 0x06, 0x00, 20, 0x00,	/* UBX-CFG-PRT */
		0x01, 0x00, 0x00, 0x00, 			/* UART1, reserved, no TX ready */
		0xe0, 0x08, 0x00, 0x00,				/* UART mode (8N1) */
		0x80, 0x25, 0x00, 0x00,				/* UART baud rate (9600) */
		0x01, 0x00,							/* input protocols (uBx only) */
		0x01, 0x00,							/* output protocols (uBx only) */
		0x00, 0x00,							/* flags */
		0x00, 0x00,							/* reserved */
		0xaa, 0x79							/* checksum */
	};

	UART_send_buffer(nonmea, sizeof(nonmea));
	return gps_receive_ack(0x06, 0x00);
}

/*
 * gps_receive_payload
 *
 * retrieves the payload of a packet with a given class and message-id with the retrieved length.
 * the caller has to ensure suitable buffer length!
 *
 * returns the length of the payload
 *
 */
uint16_t gps_receive_payload(uint8_t class_id, uint8_t msg_id, unsigned char *payload) {
	uint8_t rx_byte;
	enum {UBX_A, UBX_B, CLASSID, MSGID, LEN_A, LEN_B, PAYLOAD} state = UBX_A;
	uint16_t payload_cnt = 0;
	uint16_t payload_len = 0;
        uint32_t timeout = 0;
	while(1) {
		
		while(!UART1_SR_RXNE){ // wait for rx character
                      if(timeout++ > UBX_POLL_TIMEOUT) return 0;
                }
                
		rx_byte = UART1_DR; // get byte by byte and see what they are.
		switch (state) {
			case UBX_A:
				if (rx_byte == 0xB5)	state = UBX_B;
				else 			state = UBX_A;
				break;
			case UBX_B:
				if (rx_byte == 0x62)	state = CLASSID;
				else			state = UBX_A;
				break;
			case CLASSID:
				if (rx_byte == class_id)state = MSGID;
				else			state = UBX_A;
				break;
			case MSGID:
				if (rx_byte == msg_id)	state = LEN_A;
				else			state = UBX_A;
				break;
			case LEN_A:
				payload_len = rx_byte;
				state = LEN_B;
				break;
			case LEN_B:
				payload_len |= ((uint16_t)rx_byte << 8);
				state = PAYLOAD;
				break;
			case PAYLOAD:
				payload[payload_cnt] = rx_byte;
				payload_cnt++;
				if (payload_cnt == payload_len)
					return payload_len;
				break;
			default:
				state = UBX_A;
		}
	}
}


/* 
 * gps_get_fix
 *
 * retrieves a GPS fix from the module. if validity flag is not set, date/time and position/altitude are 
 * assumed not to be reliable!
 *
 * argument is call by reference to avoid large stack allocations
 *
 */
uint8_t gps_get_fix(struct gps_fix *fix) {
	static uint8_t response[92];	/* PVT response length is 92 bytes */
        /* UBX-NAV-PVT
         * Section 33.17.14 in the Ublox M8Q reference manual
         */
	char pvt[] = {0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19};
	int32_t alt_tmp;
		
	/* wake up from sleep */
	while(!UART1_SR_TXE); 
	UART1_DR = 0xFF;
	while(!UART1_SR_TXE); 
	gps_startup_delay();
        

	/* request position */
	UART_send_buffer(pvt, sizeof(pvt));
	if(gps_receive_payload(0x01, 0x07, response) == 0) return 0;
    
    // the mapping is found in the reference manual for M8 series gps modules. Section for UBX-NAV-PVT (0x01 0x07)
	fix->num_svs = response[23];
	fix->type = response[20];
	fix->year = response[4] + (response[5] << 8);
	fix->month = response[6];
	fix->day = response[7];
	fix->hour = response[8];
	fix->min = response[9];
	fix->sec = response[10];
	fix->lat = (int32_t) (
			(uint32_t)(response[28]) + ((uint32_t)(response[29]) << 8) + ((uint32_t)(response[30]) << 16) + ((uint32_t)(response[31]) << 24)
			);
	fix->lon = (int32_t) (
			(uint32_t)(response[24]) + ((uint32_t)(response[25]) << 8) + ((uint32_t)(response[26]) << 16) + ((uint32_t)(response[27]) << 24)
			);
	alt_tmp = (((int32_t) 
			((uint32_t)(response[36]) + ((uint32_t)(response[37]) << 8) + ((uint32_t)(response[38]) << 16) + ((uint32_t)(response[39]) << 24))
			) / 1000);
	if (alt_tmp <= 0) {
		fix->alt = 1;
	} else if (alt_tmp > 50000) {
		fix->alt = 50000;
	} else {
		fix->alt = (uint16_t) alt_tmp;
	}
        return 1;
			
}

/*
 * gps_set_gps_only
 *
 * tells the uBlox to only use the GPS satellites
 *
 * returns if ACKed by GPS
 *
 */
uint8_t gps_set_gps_only(void) {
        
	char gpsonly[] = {
		0xB5,0x62,0x06,0x3E,0x3C,0x00,				/* UBX-CFG-GNSS */
		0x00,0x00,0x20,0x07,						/* use 32 channels, 7 configs following */
		0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x01,	/* GPS enable */
		0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x01,	/* SBAS disable */
		0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x01,	/* Galileo disable */
		0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x01,	/* Beidou disable */
		0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x01,	/* IMES disable */
		0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x01,	/* QZSS	disable */
		0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x01,	/* GLONASS disable */
		0xC4,0xBC									/* checksum */
	};
       
	UART_send_buffer(gpsonly, sizeof(gpsonly));
	return gps_receive_ack(0x06, 0x3E);
}

/*
 * gps_set_airborne_model
 *
 * tells the GPS to use the airborne positioning model. Should be used to
 * get stable lock up to 50km altitude
 *
 * working uBlox MAX-M8Q
 *
 * returns if ACKed by GPS
 *
 */
uint8_t gps_set_airborne_model(void) {
	char model6[] = {
		0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 	/* UBX-CFG-NAV5 */
		0xFF, 0xFF, 							/* parameter bitmask */
		0x06, 									/* dynamic model */
		0x03, 									/* fix mode */
		0x00, 0x00, 0x00, 0x00, 				/* 2D fix altitude */
		0x10, 0x27, 0x00, 0x00,					/* 2D fix altitude variance */
		0x05, 									/* minimum elevation */
		0x00, 									/* reserved */
		0xFA, 0x00, 							/* position DOP */
		0xFA, 0x00, 							/* time DOP */
		0x64, 0x00, 							/* position accuracy */
		0x2C, 0x01, 							/* time accuracy */
		0x00,									/* static hold threshold */ 
		0x3C, 									/* DGPS timeout */
		0x00, 									/* min. SVs above C/No thresh */
		0x00, 									/* C/No threshold */
		0x00, 0x00, 							/* reserved */
		0xc8, 0x00,								/* static hold max. distance */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 	/* reserved */
		0x1a, 0x28								/* checksum */
	};

	UART_send_buffer(model6, sizeof(model6));
	return gps_receive_ack(0x06, 0x24);
}


/*
 * gps_save_settings
 *
 * saves the GPS settings to flash. should be done when power save is disabled and all
 * settings are configured. 
 */
uint8_t gps_save_settings(void) {
	char cfg[] = {
		0xB5, 0x62, 0x06, 0x09, 12, 0,	/* UBX-CFG-CFG */
		0x00, 0x00, 0x00, 0x00,		    /* clear no sections */
		0x1f, 0x1e, 0x00, 0x00,		    /* save all sections */
		0x00, 0x00, 0x00, 0x00,		    /* load no sections */
		0x58, 0x59
	};

	UART_send_buffer(cfg, sizeof(cfg));
	return gps_receive_ack(0x06, 0x09);
}


/*
 * Go into software backup mode
 *
 * Puts the GPS in software backup mode and sets it to wakeup on rising
 * edge of UART.
 * sleeps for 100s
 */
uint8_t gps_software_backup(void) {

	
	/* This is the command for a much older protocol. Please uncomment it out 
	 * to see if it has any effect on power consumption. And also comment out
	 * the other version of the command below
	 */
	
	char backup[]={
		0xB5, 0x62, 0x06, 0x57, 0x08, 0x00, 
		0x01, 0x00, 0x00, 0x00,
		0x50, 0x4B, 0x43, 0x42, 
		0x86, 0x46
	};
	
//	/* force backup */
//	char backup[]={
//	0xB5,0x62,0x02,0x41,0x10,0x00,  /* UBX-RXM-PMREQ */
//	0x00,0x00,0x00,0x00,
//	0xA0,0x86,0x01,0x00,  /* automatically wake up after 100 000 ms */
//	0x06,0x00,0x00,0x00,  /* Force to sleep, wake up on rising edge of UART pin */
//	0x08,0x00,0x00,0x00,
//	0x88,0xB7};

	UART_send_buffer(backup, sizeof(backup));
	//return gps_receive_ack(0x02, 0x41);
	return 1;

}




/*
 * gps_startup_delay
 *
 * waits for the GPS to start up. this value is empirical.
 * we could also wait for the startup string
 */
void gps_startup_delay(void) {
	/* wait for the GPS to startup */
        delay_ms(1000);

}


/*
 * UART power save mode turn on or off
 */
void uart_power_save(int on) {
  /* UARTD: UART Disable (for low power consumption).
   * When this bit is set the UART prescaler and outputs are stopped at the end of the current byte
   * transfer in order to reduce power consumption. This bit is set and cleared by software.
   * 0: UART enabled
   * 1: UART prescaler and outputs disabled
   */
    UART1_CR1_UART0 = on ;
}



