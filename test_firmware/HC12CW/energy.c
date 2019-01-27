/*
 * Energy Management functions for Clock and Power
 * 
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


#include <iostm8s003f3.h>
#include <stdint.h>
#include "HC12Board.h"

/*
 *  Setup the system clock to run at 16MHz using the internal oscillator.
 */	
void InitialiseSystemClock(void)
{
    CLK_ICKR = 0;                       //  Reset the Internal Clock Register.
    CLK_ICKR_HSIEN = 1;                 //  Enable the HSI.
    CLK_ICKR_REGAH = 1;                 //  MVR regulator can be powered off automatically when the MCU enters Active-halt mode.
    CLK_ECKR = 0;                       //  Disable the external clock.
    while (CLK_ICKR_HSIRDY == 0);       //  Wait for the HSI to be ready for use.
    CLK_CKDIVR = 0;                     //  Ensure the clocks are running at full speed.
	
    CLK_PCKENR1 = 0x8C;                 //  Enable clock to only UART 1/2/3/4, Enable TIM 1, disable other timers, SPI, I2C
    CLK_PCKENR2 = 0x04;                 //  Enable clock to only to AWU register clock, (does not disable counter clock), not to ADC
	
    CLK_CCOR = 0;                       //  Turn off CCO.
    CLK_HSITRIMR = 0;                   //  Turn off any HSIU trimming.
    CLK_SWIMCCR = 0;                    //  Set SWIM to run at clock / 2.
	
    CLK_SWR = 0xe1;                     //  Use HSI as the clock source.
    CLK_SWCR = 0;                       //  Reset the clock switch control register.
    CLK_SWCR_SWEN = 1;                  //  Enable switching.
	CLK_ICKR_FHW = 1;					//  Fast wakeup from Halt/Active-halt modes enabled
    while (CLK_SWCR_SWBSY != 0);        //  Pause while the clock switch is busy.
}


/*
* Switch to the Low Speed Internal Ocillator during the Halt period to save power
* as well as to increase the halt time.
*/
void Switch_to_LSI_clock(void)
{	
	CLK_ICKR_LSIEN = 1;					//  Low speed internal RC oscillator enable
	CLK_SWCR_SWEN = 1;                  //  Enable switching.
	CLK_SWR = 0xD2;                     //  Use LSI as the clock source.
    while (CLK_SWCR_SWBSY != 0);        //  Pause while the clock switch is busy.
}




/*
* Switch to the High Speed Internal Ocillator as soon as wakeup from active halt
*/
void Switch_to_HSI_clock(void)
{	
	CLK_ICKR_LSIEN = 0;					//  Low speed internal RC oscillator Disable

	CLK_SWCR_SWEN = 1;                  //  Enable switching.
	CLK_SWR = 0xE1;                     //  Use HSI as the clock source.
    while (CLK_SWCR_SWBSY != 0);        //  Pause while the clock switch is busy.
}





/*
 * Initialise the Auto Wake-up feature.
 * https://blog.mark-stevens.co.uk/2014/06/auto-wakeup-stm8s/
 * Total delay is 30.720s
 * Ref. Section 12.3 AWU functional description in STM8 ref manual
 * At the moment I think the values of AWUTb and APR are outside the 
 * recommended range in the ref manual. It is running for more than 30 seconds,
 * which is the max in the document
*/		
void InitialiseAWU()
{
    AWU_CSR1_AWUEN = 0;     // Disable the Auto-wakeup feature.
	AWU_APR_APR = 62; 	    // set one of the 2 registers for delay (6 bit)
    AWU_TBR_AWUTB = 15;     // set one of the 2 registers for delay (4 bit)
    AWU_CSR1_AWUEN = 1;     // Enable the Auto-wakeup feature.
}

void DeInitAWU()
{
    AWU_CSR1_AWUEN = 0;     // Disable the Auto-wakeup feature.
    AWU_TBR_AWUTB = 0;	    // needs to be 0 to save power
}

/*  Auto Wakeup Interrupt Service Routine (ISR).
 *  https://blog.mark-stevens.co.uk/2014/06/auto-wakeup-stm8s/
*/ 
#pragma vector = AWU_vector
__interrupt void AWU_IRQHandler(void)
{
    volatile unsigned char reg;

    reg = AWU_CSR1;     // Reading AWU_CSR1 register clears the interrupt flag.
}
