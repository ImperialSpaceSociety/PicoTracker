/*
 * Functions for the UBLOX 8 GPS
 * for Pico Balloon Tracker using HC12 radio module and GPS
 * HC12 Module with STM8S003F3 processor and silabs Si4463 Radio
 *  
 * Derived Work Copyright (c) 2018 Imperial College Space Society
 * From original work Copyright (C) 2014  Richard Meadows <richardeoin>
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


#include <stdint.h>
#include <iostm8s003f3.h>
#include "gps.h"
#include <string.h> //for strlen()
#include <stdio.h>


uint8_t UART1_rx_buffer[UART_RX_BUFFER_LENGTH]; // buffer for UART receive characters
uint8_t UART1_buffer_pointer;


/**
 * UART Serial Port Functions
 */
//
//  Setup the UART to run at 115200 baud, no parity, one stop bit, 8 data bits.
//
//  Important: This relies upon the system clock being set to run at 16 MHz.
//

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
    UART1_CR4 = 0;
    UART1_CR3 = 0;
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
    UART1_CR2_RIEN  = 1;     //  Receiver interrupt enable
    UART1_CR2_ILIEN = 0;     //  IDLE Line interrupt enable
    //
    //  Turn on the UART transmit, receive and the UART clock.
    //
    UART1_CR2_TEN = 1;
    UART1_CR2_REN = 1;
   
}


/*
Comments by Medad Newman 17 December 2018
UART_SR,UART_DR  are 8 bit registers that seem very important
They have some important bits in them, namely TXE and RXNE
TODO: read more about what is a shift register

more information on the registrs is found on the Refernce Manual, page 363.

https://github.com/thasti/utrak is the source of the gps code
and : https://github.com/DL7AD/pecanpico9/blob/3008ff27fe6a80bf22438779077a0475d33bd389/tracker/software/drivers/ublox.c
*/

/*
void uart_disable_nema(){
    // Disable ALL automatic NMEA mesages for polling
    uart_write("$PUBX,40,VTG,0,0,0,0*5E");
    uart_write("$PUBX,40,GSV,0,0,0,0*59");
    uart_write("$PUBX,40,GSA,0,0,0,0*4E");
    uart_write("$PUBX,40,RMC,0,0,0,0*47");    
    uart_write("$PUBX,40,GLL,0,0,0,0*5C");
    uart_write("$PUBX,40,GGA,0,0,0,0*5A");
}
*/

/**
* Send the message in the string to UART1.
*/
void gps_transmit_string(uint8_t *tx_data, uint8_t length)
{       
    /*
    *tx_data: using the * gets the value of the variable the pointer is
    pointing to.
    */
   
    uint8_t count =0;
    while (count <= length)
    {
        UART1_DR =  tx_data[count++];     //  Put the next character into the
                                          //data transmission register.
        //TXE stands for "Transmit Data register empty"
        while (UART1_SR_TXE == 0);          //  Wait for transmission to complete.
                                      
    }
}
/**
*int uart_write(const char *str,const char size) {
	char i;
	for(i = 0; i < size; i++) {
		while(!UART1_SR_TXE);
		UART1_DR = str[i];
	}
	return(i); // Bytes sent
}

*/
/**
  * gps_receive_ack
  *
  * waits for transmission of an ACK/NAK message from the GPS.
  *
  * returns 1 if ACK was received, 0 if NAK was received or timeout
  *
  */
uint8_t gps_receive_ack(uint8_t class_id, uint8_t msg_id, uint16_t timeout) {
	int match_count = 0;
	int msg_ack = 0;
	uint8_t rx_byte;
	uint8_t ack[] = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x00, 0x00};
	uint8_t nak[] = {0xB5, 0x62, 0x05, 0x00, 0x02, 0x00, 0x00, 0x00};
	ack[6] = class_id;
	nak[6] = class_id;
	ack[7] = msg_id;
	nak[7] = msg_id;

	// runs until ACK/NAK packet is received
	systime_t sTimeout = chVTGetSystemTimeX() + MS2ST(timeout);
	while(sTimeout >= chVTGetSystemTimeX()) {

		// Receive one byte
		;
		if(!gps_receive_byte(&rx_byte)) {
			chThdSleepMilliseconds(10);
			continue;
		}

		// Process one byte
		if (rx_byte == ack[match_count] || rx_byte == nak[match_count]) {
			if (match_count == 3) {	/* test ACK/NAK byte */
				if (rx_byte == ack[match_count]) {
					msg_ack = 1;
				} else {
					msg_ack = 0;
				}
			}
			if (match_count == 7) { 
				return msg_ack;
			}
			match_count++;
		} else {
			match_count = 0;
		}

	}

	return 0;
}
/**
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
	uint8_t nonmea[] = {
		0xB5, 0x62, 0x06, 0x00, 20, 0x00,	// UBX-CFG-PRT
		0x01, 0x00, 0x00, 0x00, 			// UART1, reserved, no TX ready
		0xe0, 0x08, 0x00, 0x00,				// UART mode (8N1)
		0x80, 0x25, 0x00, 0x00,				// UART baud rate (9600)
              0x01, 0x00,							// input protocols (uBx only)
              0x01, 0x00,							// output protocols (uBx only)
              0x00, 0x00,							// flags
              0x00, 0x00,							// reserved
              0xaa, 0x79							// checksum
	};

	gps_transmit_string(nonmea, sizeof(nonmea));
	return gps_receive_ack(0x06, 0x00, 1000);
}
/**
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
	uint8_t model6[] = {
		0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 	// UBX-CFG-NAV5
		0xFF, 0xFF, 							// parameter bitmask
		0x06, 									// dynamic model
		0x03, 									// fix mode
		0x00, 0x00, 0x00, 0x00, 				// 2D fix altitude
		0x10, 0x27, 0x00, 0x00,					// 2D fix altitude variance
		0x05, 									// minimum elevation
		0x00, 									// reserved
		0xFA, 0x00, 							// position DOP
		0xFA, 0x00, 							// time DOP
		0x64, 0x00, 							// position accuracy
		0x2C, 0x01, 							// time accuracy
		0x00,									// static hold threshold 
		0x3C, 									// DGPS timeout
		0x00, 									// min. SVs above C/No thresh
		0x00, 									// C/No threshold
		0x00, 0x00, 							// reserved
		0xc8, 0x00,								// static hold max. distance
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 	// reserved
		0x1a, 0x28								// checksum
	};

	gps_transmit_string(model6, sizeof(model6));
	return gps_receive_ack(0x06, 0x24, 1000);
}
/**
 * UART Rx Interupt. 
 */


#pragma vector = UART1_R_RXNE_vector //a special instruction to compiler
// RXNE stands for - "Receive Data register not empty"

__interrupt void UART1_IRQHandler(void)
{  
  //unsigned char status = UART1_SR; // try to see the Status Register. IT turns out, the status HAS to be
  //read each time before UART1_DR to prevent it from throwing errors when reading UART1_DR. This is very important.
  // The idea comes from the question: https://electronics.stackexchange.com/questions/222638/clearing-usart-uart-interrupt-flags-in-an-stm32
  // also shown in the code at the bottom of : http://www.micromouseonline.com/2009/12/31/stm32-usart-basics/
  
  while (!UART1_SR_RXNE); // wait until a status has been received
  //while(!UART1_SR); // wait to fully receive the data
  
  unsigned char data = UART1_DR; // this is the data byte that has been received. Reads
                            // the UART1_DR, the data register. Seems to be an error often
  /*
  the value of UART1_SR is 0b11011000
  IT indicates that Over run error detected and LIN error detected
  */

  //if(data == '\n') // Check if the end of nmea line is reached
  // TODO: find the last character of the pubx string
  if(data == 0x0d)// check if we have reached the end of the pubx string. doesnt verify the data at the moment.
    // TODO: make a parser for the data.
  {
     UART1_rx_buffer[UART1_buffer_pointer] = data; // puts the data into the buffer

     //UART_send_buffer(UART1_rx_buffer, UART1_buffer_pointer);
     /*
      see if it is possible to clear some flag such that it can receive data
      again. THe problem appears to be UART1_DR which returns an error some times.
      Maybe disable the receiving when needed and then transmit.
      */
     UART1_buffer_pointer = 0;
  }
  else
  {     
      if(UART1_buffer_pointer < UART_RX_BUFFER_LENGTH -1)
      {
      UART1_rx_buffer[UART1_buffer_pointer++] = data;
      }
  }
}
// find a way to prevent the interrupts to fireing just about any time.




