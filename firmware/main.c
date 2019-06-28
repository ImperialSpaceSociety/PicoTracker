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



/*
* the TX data buffer
* contains ASCII data, which is either transmitted as CW over RTTY
*/
uint16_t tx_buf_rdy = 0;			/* the read-flag (main -> main) */
uint16_t tx_buf_length = 0;			/* how many chars to send */
char tx_buf[TX_BUF_MAX_LENGTH] = {SYNC_PREFIX "$$" PAYLOAD_NAME ","};	/* the telemetry buffer initialised with $$ */
extern uint16_t tlm_sent_id_length; 
extern uint16_t tlm_alt_length;    


/* Retry counters and Operational Status*/
uint8_t  ubx_cfg_fail = 0;
uint8_t  ubx_retry_count;
uint8_t  ubx_poll_fail = 0;



									

/* current (latest) GPS fix and measurements */
struct gps_fix current_fix;







void get_fix(void) {
    ubx_poll_fail = 0;
	
	/* 
	 * The tracker outputs Pips while waiting for a good GPS fix.
     */
	
	current_fix.num_svs = 0; 
	current_fix.type = 0;

	while (1) {
		
		/* check if we have a fix*/
		for(ubx_retry_count=0; ubx_retry_count < UBX_POLL_RETRIES ; ubx_retry_count++){ 
                //if( gps_get_fix(&current_fix)) break; // don't forget to remove the fake fix
     		if( gps_get_fake_fix(&current_fix)) break; // don't forget to remove the fake fix
      		ubx_poll_fail = 1;
      		if(ubx_retry_count == (UBX_POLL_RETRIES -1)) ubx_poll_fail = 2;
    	} 
		
		/* check if we have a 3D fix */
		if (current_fix.type == 3){
			break;
		};
		
		/* Pip because we don't have a fix yet*/
		telemetry_start(TELEMETRY_PIPS, 1);
		
		/* Sleep Wait */ 
		while (telemetry_active());
	}   

}

void get_measurements(void){
	current_fix.temp_radio = si_trx_get_temperature();
    current_fix.op_status = ((ubx_cfg_fail & 0x03) << 2) | ((ubx_poll_fail & 0x03)); //send operational status
	// DO we need 4 bytes for op status? it seems to use only one byte at most
    current_fix.voltage_radio =  si_trx_get_voltage();
}


int main( void )
{
    /* get the clock working and initialise the auto wakeup service*/
    __disable_interrupt();
    InitialiseSystemClock();
    InitialiseAWU(); // auto wake up
    __enable_interrupt();
    
    /* Start the UART */
    InitialiseUART(); // set up the uart
    
    
    /* Initialise Si4060 interface */
    si_trx_init();
    
    
    /* Initialise GPS */   
    gps_startup_delay(); // wait 1 sec for GPS to startup
	
	
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
         
         
 	/* Get a single GPS fix from a cold start. Does not carry on until it has a
	 * solid fix
	*/
	get_fix();
    get_measurements();
	
	
	/* activate power save mode as fix is stable. 1 to activate power save.*/
	for(ubx_retry_count=0; ubx_retry_count < UBX_CFG_RETRIES; ubx_retry_count++){ // Power Save Mode ON
      if((gps_power_save(1))) break;
      ubx_cfg_fail = 1;
      if(ubx_retry_count == (UBX_CFG_RETRIES -1)) ubx_cfg_fail = 2;
    } 
	
	
	for(ubx_retry_count=0; ubx_retry_count < UBX_CFG_RETRIES; ubx_retry_count++){ // Save setup to gps flash
      if((gps_save_settings())) break;
      ubx_cfg_fail = 1;
      if(ubx_retry_count == (UBX_CFG_RETRIES -1)) ubx_cfg_fail = 2;
    } 
    
	
    
    while (1)
    {
	/* Turn back on uart. 0 to turn Uart back on*/
	uart_power_save(0); 

	/* now wake up the GPS */
	gps_wake_up();
	
	/* now put the gps in full power mode */
	while(!(gps_power_save(0)));
	
	/* get the gps fix */
        get_fix();
	
	/* put the gps back to power save mode(sleep) */
	while(!(gps_power_save(1)));
	
	
	/* save power by turning off uart on stm8,  1 to turn off UART*/
	uart_power_save(1); 
	
	/* get voltage  and temperature*/
        get_measurements(); 
	
	/* create the telemetry string */
	prepare_tx_buffer();
	
	/* 10 start pips */
	//telemetry_start(TELEMETRY_PIPS, 1);
	
	/* Sleep Wait */ 
	//while (telemetry_active());
	
	
	/* send telemetry over RTTY */
	tx_buf_length  = TX_BUF_FRAME_END;
	telemetry_start(TELEMETRY_RTTY, tx_buf_length);
	
	/* Sleep Wait */ 
	while (telemetry_active());
	

	/* go into active halt for around 30s. This will not be very accurate.
	 * https://blog.mark-stevens.co.uk/2014/06/auto-wakeup-stm8s/ 
	 * The automatic interrupt wakes up the controller.
	 * TODO: how to make it sleep for longer at higher altitudes? call __halt repeatedly?
	 */
	
	//Switch_to_LSI_clock();

	/* reinit AWU_TBR. see ref manual section 12.3.1. Do we have to do this while disabling 
	 * interrupt like in the init function(InitialiseAWU())? */
	//InitialiseAWU(); // Initialise the autowakeup feature 
	//__halt(); // halt until an interrupt wakes things up in 30s
	
	//if (current_fix.alt> 3000){	
		//__halt(); // halt until an interrupt wakes things up in 30s
	//}
	//DeInitAWU(); // set AWU_TBR = 0 for power saving. See ref manual section 12.3.1
	
    } /* while(1)*/
    
} /* main()*/



