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
/*
void InitialiseUART(void)
{
    //
    //  Clear the Idle Line Detected bit in the status rerister by a read
    //  to the UART1_SR register followed by a Read to the UART1_DR register.
    //
    unsigned char tmp = UART1_SR;
    tmp = UART1_DR;
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
*/


//
//  Send the message in the string to UART1.
//
void UART_send_buffer(uint8_t *tx_data, uint8_t length)
{
   
    uint8_t count =0;
    while (count <= length)
    {
        UART1_DR =  tx_data[count++];     //  Put the next character into the data transmission register.
        while (UART1_SR_TXE == 0);          //  Wait for transmission to complete.
                                      
    }
}

/*
void delay_ms(unsigned long ms) {
	//The best naive delay @16MHz
	//the 960 comes from the number of instructions to perform the do/while loop
	//to figure it out, have a look at the generated ASM file after compilation
	unsigned long cycles = 960 * ms;
	int i = 0;
	do
	{
		cycles--;
	}
	while(cycles > 0);
}
*/

/**
 * UART Rx Interupt. 
 */

/*
#pragma vector = UART1_R_RXNE_vector //a special instruction to compiler


__interrupt void UART1_IRQHandler(void)
{
  uint8_t data = UART1_DR;
  if(data == '\n')
  {
     UART1_rx_buffer[UART1_buffer_pointer] = data;
     //UART_send_buffer(UART1_rx_buffer, UART1_buffer_pointer);
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
*/


int uart_write(const char *str) {
	char i;
	for(i = 0; i < strlen(str); i++) {
		while(!(UART1_SR & UART_SR_TXE));
		UART1_DR = str[i];
	}
	return(i); // Bytes sent
}




void InitialiseUART() {
	unsigned long i = 0;
	CLK_CKDIVR = 0x00;
	CLK_PCKENR1 = 0xFF; // Enable peripherals

	PC_DDR = 0x08; // Put TX line on
	PC_CR1 = 0x08;

	UART1_CR2 = UART_CR2_TEN; // Allow TX & RX
	UART1_CR3 &= ~(UART_CR3_STOP1 | UART_CR3_STOP2); // 1 stop bit
	UART1_BRR2 = 0x03; UART1_BRR1 = 0x68; // 9600 baud@16MHz CLK
k.

	//main loop
	while(1) {
		uart_write("Visit http://embedonix.com/tag/stm8 for more info!\r\n");
		delay_ms(1000);
	}
}