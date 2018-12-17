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
    Somehow, by transmitting the data back over uart, the uart can 
    restart listening over again. and doesn't return an error

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
 * UART Rx Interupt. 
 */


#pragma vector = UART1_R_RXNE_vector //a special instruction to compiler
// RXNE stands for - "Receive Data register not empty"

__interrupt void UART1_IRQHandler(void)
{  
  unsigned char status = UART1_SR; // try to see the Status Register. IT turns out, the status HAS to be
  //read each time before UART1_DR to prevent it from throwing errors when reading UART1_DR. This is very important.
  // The idea comes from the question: https://electronics.stackexchange.com/questions/222638/clearing-usart-uart-interrupt-flags-in-an-stm32

  unsigned char data = UART1_DR; // this is the data byte that has been received. Reads
                            // the UART1_DR, the data register. Seems to be an error often
  /*
  the value of UART1_SR is 0b11011000
  IT indicates that Over run error detected and LIN error detected
  */

  if(data == '\n') // echo back the data via uart when the end of the line is reached.
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




