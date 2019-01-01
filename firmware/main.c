/*
* main.c  Main Code Module
* 
* Pico Balloon Tracker using HC12 radio module and GPS
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

/*
Things to do 17 December 2018

make the program parse the gps coordinates
form the telemetry string
transmit the telemetry string
Try to reduce memory usage when sending the pubx strings to disable nmea



*/




#include <stdint.h>
#include <iostm8s003f3.h>
#include "HC12Board.h"
#include "si_trx.h"
#include "si_trx_defs.h"
#include "telemetry.h"
#include "energy.h"
#include "gps.h"
#include <intrinsics.h>
#include "main.h"


/* A lot of work for the telemetry and gps communication is taken from 
* https://github.com/thasti/utrak
*/



/*
Strategy:

todo: find out why telemetry from gps fails exactly at message 36. 
Why does it read 2 positions at a time?
*/




/*
* the TX data buffer
* contains ASCII data, which is either transmitted as CW over RTTY
*/
uint16_t tx_buf_rdy = 0;			/* the read-flag (main -> main) */
uint16_t tx_buf_length = 0;			/* how many chars to send */
char tx_buf[TX_BUF_MAX_LENGTH] = {SYNC_PREFIX "$$" PAYLOAD_NAME ","};	/* the telemetry buffer initialised with $$ */

/* Retry counters and Operational Status*/
uint8_t  ubx_cfg_fail = 0;
uint8_t  ubx_retry_count;
uint8_t  ubx_poll_fail = 0;


/* current (latest) GPS fix and measurements */
struct gps_fix current_fix;

void get_fix_and_measurements(void) {
    
    for(ubx_retry_count=0; ubx_retry_count < UBX_POLL_RETRIES ; ubx_retry_count++){ 
      if( gps_get_fix(&current_fix)) break;
      ubx_poll_fail = 1;
      if(ubx_retry_count == (UBX_POLL_RETRIES -1)) ubx_poll_fail = 2;
    } 

    current_fix.temp_radio = si_trx_get_temperature();
    current_fix.op_status = ((ubx_cfg_fail & 0x03) << 2) | ((ubx_poll_fail & 0x03)); //send operational status
    current_fix.voltage_radio =  si_trx_get_voltage();
}



int main( void )
{
    /* get the clock working */
    __disable_interrupt();
    InitialiseSystemClock();    
    __enable_interrupt();
    
    /* Start the UART */
    InitialiseUART(); // set up the uart
    // TODO: put uart in disable mode when not using it to save power
    
    
    /* Initialise Si4060 interface */
    si_trx_init();
    
    
    /* Initialise GPS */   
    gps_startup_delay(); // wait 1 sec for GPS to startup
    
    for(ubx_retry_count=0; ubx_retry_count < UBX_CFG_RETRIES; ubx_retry_count++){ // Setup for no NMEA Messages
      if((gps_disable_nmea_output())) break;
      ubx_cfg_fail = 1;
      if(ubx_retry_count == (UBX_CFG_RETRIES -1)) ubx_cfg_fail = 2;
    }
         
    for(ubx_retry_count=0; ubx_retry_count < UBX_CFG_RETRIES; ubx_retry_count++){ // Setup for only GPS mode 
      if((gps_set_gps_only())) break;
      ubx_cfg_fail = 1;
      if(ubx_retry_count == (UBX_CFG_RETRIES -1)) ubx_cfg_fail = 2;
    } 
         
    for(ubx_retry_count=0; ubx_retry_count < UBX_CFG_RETRIES; ubx_retry_count++){ // Setup for High Altitude 
      if((gps_set_airborne_model())) break;
      ubx_cfg_fail = 1;
      if(ubx_retry_count == (UBX_CFG_RETRIES -1)) ubx_cfg_fail = 2;
    } 
         
    for(ubx_retry_count=0; ubx_retry_count < UBX_CFG_RETRIES; ubx_retry_count++){ // Configure Power Save Mode
      if((gps_set_power_save())) break;
      ubx_cfg_fail = 1;
      if(ubx_retry_count == (UBX_CFG_RETRIES -1)) ubx_cfg_fail = 2;
    } 
         
    for(ubx_retry_count=0; ubx_retry_count < UBX_CFG_RETRIES; ubx_retry_count++){ // Power Save Mode Off
      if((gps_power_save(0))) break;
      ubx_cfg_fail = 1;
      if(ubx_retry_count == (UBX_CFG_RETRIES -1)) ubx_cfg_fail = 2;
    } 
         
    for(ubx_retry_count=0; ubx_retry_count < UBX_CFG_RETRIES; ubx_retry_count++){ // Save setup to gps flash
      if((gps_save_settings())) break;
      ubx_cfg_fail = 1;
      if(ubx_retry_count == (UBX_CFG_RETRIES -1)) ubx_cfg_fail = 2;
    } 
    
    
    //    
    //    
    //    /* the tracker outputs RF blips while waiting for a GPS fix */
    //    while (current_fix.num_svs < 5 && current_fix.type < 3) {
    //        //WDTCTL = WDTPW + WDTCNTCL + WDTIS0; // TODO: work out how to use the watchdog timer
    //        if (seconds > BLIP_FIX_INTERVAL) {
    //                seconds = 0;
    //                gps_get_fix(&current_fix);
    //                telemetry_start(TELEMETRY_PIPS, 1);
    //
    //        } else {
    //                telemetry_start(TELEMETRY_PIPS, 5);
    //
    //        }
    //    }
    //    
    
    // TODO : how to use the watchdog timer here to prevent it from getting stuck
    // one of the reasons for getting stuck is when data does not arrive from the
    // GPS module, leaving the MCU waiting for a resposnse forever.
    while (1)
    {
	
	/* get the gps fix, voltage  and temperature*/

        get_fix_and_measurements();
	
	/* save gps power by putting in power save mode */
	//gps_power_save(1); // arg = 1 to enable power save. Seems to fail to send back data sometimes.
	
	/* save power by putting in power save mode */
	// TODO: work out how to wake up the uart again. It doesn't wake back up
	//uart_power_save(1); // 1 to power save
	
	
	/* fill the zeros with x. For debug. Not sure why. comment on original function states
	* that if there are the field handling errors, we get this.*/
	init_tx_buffer();
	
	/* create the telemetry string */
	prepare_tx_buffer();
	
	
	
	/* start pips */
	telemetry_start(TELEMETRY_PIPS, 5);
	
	/* Sleep Wait */ 
	while (telemetry_active());
	
	/* send telemetry over RTTY */
	telemetry_start(TELEMETRY_RTTY, TX_BUF_MAX_LENGTH);
	
	/* Sleep Wait */ 
	while (telemetry_active());
	
	
    }
    
    
}



