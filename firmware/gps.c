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
// names of the different types of nmea strings
const char nema_string_names[6][3] ={"VTG","GSV","GSA","RMC","GLL","GGA"};



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
*/



//
//  Send the message in the string to UART1.
//
void UART_send_buffer(uint8_t *tx_data, uint8_t length)
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


int uart_write(const char *str) {
	char i;
	for(i = 0; i < strlen(str); i++) {
		while(!UART1_SR_TXE);
		UART1_DR = str[i];
	}
	return(i); // Bytes sent
}

int calculateChecksum (const char *msg)
{
    int checksum = 0;
    for (int i = 0; msg[i] && i < 32; i++)
        checksum ^= (unsigned char)msg[i];

    return checksum;
}

int nemaMsgSend (const char *msg)
{
    char checksum[8];
    snprintf(checksum, sizeof(checksum)-1, "*%.2X", calculateChecksum(msg));
    uart_write("$");
    uart_write(msg);
    uart_write(checksum);
    
    return 1;
}

int nemaMsgDisable (const char *nema)
{
    if (strlen(nema) != 3) return 0;

    char tmp[32];
    snprintf(tmp, sizeof(tmp)-1, "PUBX,40,%s,0,0,0,0", nema); // see if I can make this a constant in flash
    //snprintf(tmp, sizeof(tmp)-1, F("PUBX,40,%s,0,0,0,0,0,0"), nema);
    nemaMsgSend(tmp);

    return 1;
}

/*
     // Disable ALL automatic NMEA mesages for polling
    uart_write("$PUBX,40,VTG,0,0,0,0*5E\r\n");
    uart_write("$PUBX,40,GSV,0,0,0,0*59\r\n");
    uart_write("$PUBX,40,GSA,0,0,0,0*4E\r\n");
    uart_write("$PUBX,40,RMC,0,0,0,0*47\r\n");    
    uart_write("$PUBX,40,GLL,0,0,0,0*5C\r\n");
    uart_write("$PUBX,40,GGA,0,0,0,0*5A\r\n");
    //TODO: make sure the acknoledgement packet is received. Currently it works
*/



void uart_disable_nema(){
  for(int i = 0; i < 6; i++)
  {
    nemaMsgSend(nema_string_names[i]);
  }
}
/*
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
  if(data == '*') // check if we have reached the checksum in pubx string. doesnt verify the data at the moment.
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




