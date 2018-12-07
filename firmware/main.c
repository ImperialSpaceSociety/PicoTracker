/*
* main.c  Main Code Module
* 
* Pico Balloon Tracker using HC12 radio module and GPS
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
#include "HC12Board.h"
#include "si_trx.h"
#include "si_trx_defs.h"
#include "telemetry.h"
#include <intrinsics.h>

//
//  Setup the system clock to run at 16MHz using the internal oscillator.
//
void InitialiseSystemClock()
{
    CLK_ICKR = 0;                       //  Reset the Internal Clock Register.
    CLK_ICKR_HSIEN = 1;                 //  Enable the HSI.
    CLK_ECKR = 0;                       //  Disable the external clock.
    while (CLK_ICKR_HSIRDY == 0);       //  Wait for the HSI to be ready for use.
    CLK_CKDIVR = 0;                     //  Ensure the clocks are running at full speed.
    CLK_PCKENR1 = 0xff;                 //  Enable all peripheral clocks.
    CLK_PCKENR2 = 0xff;                 //  Ditto.
    CLK_CCOR = 0;                       //  Turn off CCO.
    CLK_HSITRIMR = 0;                   //  Turn off any HSIU trimming.
    CLK_SWIMCCR = 0;                    //  Set SWIM to run at clock / 2.
    CLK_SWR = 0xe1;                     //  Use HSI as the clock source.
    CLK_SWCR = 0;                       //  Reset the clock switch control register.
    CLK_SWCR_SWEN = 1;                  //  Enable switching.
    while (CLK_SWCR_SWBSY != 0);        //  Pause while the clock switch is busy.
}

//
//  Setup the UART to run at 115200 baud, no parity, one stop bit, 8 data bits.
//
//  Important: This relies upon the systemk clock being set to run at 16 MHz.
//
void InitialiseUART()
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
    //  Now setup the port to 115200,n,8,1.
    //
    UART1_CR1_M = 0;        //  8 Data bits.
    UART1_CR1_PCEN = 0;     //  Disable parity.
    UART1_CR3_STOP = 0;     //  1 stop bit.
    UART1_BRR2 = 0x0a;      //  Set the baud rate registers to 115200 baud // change for 9600 bps for gps 
    UART1_BRR1 = 0x08;      //  based upon a 16 MHz system clock.
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
    //  Turn on the UART transmit, receive and the UART clock.
    //
    UART1_CR2_TEN = 1;
    UART1_CR2_REN = 1;
   
}

void delay_ms(unsigned long ms) {
	//The best naive delay @16MHz
	//the 960 comes from the number of instructions to perform the do/while loop
	//to figure it out, have a look at the generated ASM file after compilation
	unsigned long cycles = 960 * ms;
	do
	{
		cycles--;
	}
	while(cycles > 0);
}


//
//  Send the message in the string to UART1.
//
void UARTPrintF(char *message)
{
    char *ch = message;
    while (*ch)
    {
        UART1_DR = (unsigned char) *ch;     //  Put the next character into the data transmission register.
        while (UART1_SR_TXE == 0);          //  Wait for transmission to complete.
        ch++;                               //  Grab the next character.
    }
}

int main( void )
{
    __disable_interrupt();
    InitialiseSystemClock();
    InitialiseUART();
    __enable_interrupt();
    
// start up the radio    
  
    /* Initialise Si4060 interface */
    si_trx_init();
    
    
    while (1)
    {
   

    telemetry_start(TELEMETRY_PIPS, 5);

    /* Sleep Wait */
    while (telemetry_active());
    
    telemetry_start(TELEMETRY_RTTY, 20);

    /* Sleep Wait */
    while (telemetry_active());
    
    
    }
}
