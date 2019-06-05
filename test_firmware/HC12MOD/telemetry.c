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


#include <iostm8s003f3.h>
#include <stdint.h>
#include <intrinsics.h>
#include "HC12Board.h"
#include "telemetry.h"
#include "si_trx.h"
#include "si_trx_defs.h"



#define TIMER1_PRESCALE  (HSCLK_FREQUENCY/1000)
#define MOD_BIT_MS       20
#define TRANSMIT_CYCLES  1000
/**
 * Interface to the physical world.
 */
#define MOD_CHANNEL_SPACING	 XO_745_DEVIATION // 745Hz Deviation
#define MOD_CHANNEL_DEVIATION	(MOD_CHANNEL_SPACING / 2)
#define MOD_CHANNEL(b)		(b ? MOD_CHANNEL_DEVIATION : -MOD_CHANNEL_DEVIATION)
#define MOD_SET(b)		si_trx_switch_channel(MOD_CHANNEL(b))






/**
* TELEMETRY OUTPUT
* =============================================================================
*/


uint8_t mark_space;
uint8_t  telemetry_on;
uint16_t transmit_cycles;

uint8_t telemetry_active(void){
    return telemetry_on;
}

/**
* Starts telemetry output
*
* Returns 0 on success, 1 if already active
*/
void telemetry_start(void) {
	/* Setup timer tick */
        timer1_tick_init(MOD_BIT_MS);
        telemetry_on = 1;
        transmit_cycles = 0;
        mark_space = 0;
	si_trx_on(SI_MODEM_MOD_TYPE_CW, 1);	
        
}





/**
* Called at the telemetry mode's baud rate
*/
void telemetry_tick(void) {
  if(transmit_cycles++ < TRANSMIT_CYCLES){
    mark_space++;
    if(mark_space == 2) mark_space =0;
    MOD_SET(mark_space);
  }
  else {
    /* Turn radio off */
    si_trx_off();
    /* De-init timer */
    timer1_tick_deinit();
    telemetry_on = 0;
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


