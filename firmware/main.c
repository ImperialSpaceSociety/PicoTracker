/*
* main.c  Main Code Module
*
* Pico Balloon Tracker using HC12 radio module and GPS
* HC12 Module with STM8S003F3 processor and silabs Si4463 Radio
*
* This Branch is for build with TCXO Oscillator only
* includes enable of TCXO, 32MHz Clock and config changes to Si4463
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



#include <stdint.h>
#include <iostm8s003f3.h>
#include "HC12Board.h"
#include "si_trx.h"
#include "si_trx_defs.h"
#include "telemetry.h"

#include "energy.h"
#include "gps.h"
#include "ubx_protocol.h"
#include <intrinsics.h>
#include "main.h"


/* A lot of work for the telemetry and GPS communication is taken from
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


/* Retry counters and operational status.
 * Bits 7..4: GPS fix poll attempts, saturated at 15.
 * Bits 3..2: GPS configuration status.
 * Bits 1..0: GPS poll status.
 */
#define GPS_FIX_ATTEMPTS_MAX       0x0F
#define OP_STATUS_ERROR_MASK       0x03
#define OP_STATUS_CFG_SHIFT        2
#define OP_STATUS_FIX_SHIFT        4
#define OP_STATUS_OK               0
#define OP_STATUS_TRANSIENT_ERROR  1
#define OP_STATUS_RETRY_EXHAUSTED  2
#define OP_STATUS_DEGRADED         3

static uint8_t ubx_cfg_fail = 0;
static uint8_t ubx_retry_count;
static uint8_t ubx_poll_fail = OP_STATUS_OK;
static uint8_t gps_fix_attempts = 0;





/* current (latest) GPS fix and measurements */
struct gps_fix current_fix;

uint8_t get_fix(void) {
    ubx_poll_fail = OP_STATUS_OK;
    gps_fix_attempts = 0;

    /*
    * The tracker outputs Pips while waiting for a good GPS fix.
    */

    current_fix.num_svs = 0;
    current_fix.type = 0;
    current_fix.flags = 0;

    while (gps_fix_attempts < GPS_FIX_ATTEMPTS_MAX) {

        /* check if we have a fix*/
        for(ubx_retry_count=0;
            ubx_retry_count < UBX_POLL_RETRIES && gps_fix_attempts < GPS_FIX_ATTEMPTS_MAX;
            ubx_retry_count++){
            gps_fix_attempts++;
            if (gps_get_fix(&current_fix)) break;
            ubx_poll_fail = OP_STATUS_TRANSIENT_ERROR;
            if(ubx_retry_count == (UBX_POLL_RETRIES -1)) ubx_poll_fail = OP_STATUS_RETRY_EXHAUSTED;
        }

        /* accept only a valid 3D navigation solution */
        if (ubx_nav_pvt_fix_is_usable(current_fix.type, current_fix.flags)) {
            return 1;
        }

        /* Pip because we don't have a fix yet*/
        //telemetry_start(TELEMETRY_PIPS, 1);

        /* Sleep Wait */
        //while (telemetry_active());
    }

    ubx_poll_fail = OP_STATUS_DEGRADED;
    return 0;
}

static uint8_t gps_power_mode_with_retries(uint8_t on)
{
    uint8_t retry;

    for (retry = 0; retry < UBX_CFG_RETRIES; retry++) {
        if (gps_power_save(on)) {
            return 1;
        }
        ubx_cfg_fail = OP_STATUS_TRANSIENT_ERROR;
    }

    ubx_cfg_fail = OP_STATUS_RETRY_EXHAUSTED;
    return 0;
}

void get_measurements(void){
    current_fix.temp_radio = si_trx_get_temperature();
    current_fix.op_status = ((uint16_t)gps_fix_attempts << OP_STATUS_FIX_SHIFT) |
                            ((uint16_t)(ubx_cfg_fail & OP_STATUS_ERROR_MASK) << OP_STATUS_CFG_SHIFT) |
                            (ubx_poll_fail & OP_STATUS_ERROR_MASK);
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
    InitialiseUART(); // set up the UART


    /* Initialise Si4060 interface */
    si_trx_init();

    /* indicate that it is alive!*/
    telemetry_start(TELEMETRY_PIPS, 3);


    /* Initialise GPS */
    gps_startup_delay(); // wait 1 sec for GPS to start up




    for(ubx_retry_count=0; ubx_retry_count < UBX_CFG_RETRIES; ubx_retry_count++){ // Configure Power Save Mode
        if((gps_set_power_save())) break;
        ubx_cfg_fail = OP_STATUS_TRANSIENT_ERROR;
        if(ubx_retry_count == (UBX_CFG_RETRIES -1)) ubx_cfg_fail = OP_STATUS_RETRY_EXHAUSTED;
    }

    for(ubx_retry_count=0; ubx_retry_count < UBX_CFG_RETRIES; ubx_retry_count++){ // Power Save Mode Off
        if((gps_power_save(0))) break;
        ubx_cfg_fail = OP_STATUS_TRANSIENT_ERROR;
        if(ubx_retry_count == (UBX_CFG_RETRIES -1)) ubx_cfg_fail = OP_STATUS_RETRY_EXHAUSTED;
    }

    for(ubx_retry_count=0; ubx_retry_count < UBX_CFG_RETRIES; ubx_retry_count++){ // Setup for no NMEA Messages
        if((gps_disable_nmea_output())) break;
        ubx_cfg_fail = OP_STATUS_TRANSIENT_ERROR;
        if(ubx_retry_count == (UBX_CFG_RETRIES -1)) ubx_cfg_fail = OP_STATUS_RETRY_EXHAUSTED;
    }

    for(ubx_retry_count=0; ubx_retry_count < UBX_CFG_RETRIES; ubx_retry_count++){ // Setup for only GPS mode
        if((gps_set_gps_only())) break;
        ubx_cfg_fail = OP_STATUS_TRANSIENT_ERROR;
        if(ubx_retry_count == (UBX_CFG_RETRIES -1)) ubx_cfg_fail = OP_STATUS_RETRY_EXHAUSTED;
    }

    for(ubx_retry_count=0; ubx_retry_count < UBX_CFG_RETRIES; ubx_retry_count++){ // Setup for High Altitude
        if((gps_set_airborne_model())) break;
        ubx_cfg_fail = OP_STATUS_TRANSIENT_ERROR;
        if(ubx_retry_count == (UBX_CFG_RETRIES -1)) ubx_cfg_fail = OP_STATUS_RETRY_EXHAUSTED;
    }


    /* Get a single GPS fix from a cold start. Does not carry on until it has a
    * solid fix
    */
    get_fix();
    get_measurements();


    /* activate power save mode as fix is stable. 1 to activate power save.*/
    for(ubx_retry_count=0; ubx_retry_count < UBX_CFG_RETRIES; ubx_retry_count++){ // Power Save Mode ON
        if((gps_power_save(1))) break;
        ubx_cfg_fail = OP_STATUS_TRANSIENT_ERROR;
        if(ubx_retry_count == (UBX_CFG_RETRIES -1)) ubx_cfg_fail = OP_STATUS_RETRY_EXHAUSTED;
    }


    for(ubx_retry_count=0; ubx_retry_count < UBX_CFG_RETRIES; ubx_retry_count++){ // Save setup to GPS flash
        if((gps_save_settings())) break;
        ubx_cfg_fail = OP_STATUS_TRANSIENT_ERROR;
        if(ubx_retry_count == (UBX_CFG_RETRIES -1)) ubx_cfg_fail = OP_STATUS_RETRY_EXHAUSTED;
    }



    while (1)
    {
	/* Turn the UART back on. 0 enables the UART*/
	uart_power_save(0);

	/* now wake up the GPS */
	gps_wake_up();

	/* put the GPS in full power mode */
	while(!(gps_power_save(0)));

	/* get the GPS fix */
        get_fix();

	/* put the GPS back into power-save mode (sleep) */
	while(!(gps_power_save(1)));


	/* save power by turning off UART on STM8,  1 to turn off UART*/
	uart_power_save(1);

	/* get voltage  and temperature*/
        get_measurements();

	/* create the telemetry string */
	prepare_tx_buffer();

	/* 10 start pips */
	telemetry_start(TELEMETRY_PIPS, 10);

	/* Sleep Wait */
	while (telemetry_active());


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

	Switch_to_LSI_clock();

	/* reinit AWU_TBR. see ref manual section 12.3.1. Do we have to do this while disabling
        * interrupt like in the init function(InitialiseAWU())? */
	InitialiseAWU(); // Initialise the autowakeup feature

        if (current_fix.alt> 500){
            __halt(); // halt until an interrupt wakes things up in 30s
        }

	if (current_fix.alt> 3000){
           __halt(); // halt until an interrupt wakes things up in 30s
	}
	DeInitAWU(); // set AWU_TBR = 0 for power saving. See ref manual section 12.3.1

    } /* while(1)*/

} /* main()*/



