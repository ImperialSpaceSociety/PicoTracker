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
#include "energy.h"
#include "gps.h"
#include <intrinsics.h>


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




int main( void )
{
    __disable_interrupt();
    InitialiseSystemClock();
    __enable_interrupt();

    // Start the UART
    InitialiseUART(); // set up the uart
    
     // Disable ALL automatic NMEA mesages for polling
    __disable_interrupt();
    uart_write("$PUBX,40,VTG,0,0,0,0*5E\r\n");
    uart_write("$PUBX,40,GSV,0,0,0,0*59\r\n");
    uart_write("$PUBX,40,GSA,0,0,0,0*4E\r\n");
    uart_write("$PUBX,40,RMC,0,0,0,0*47\r\n");    
    uart_write("$PUBX,40,GLL,0,0,0,0*5C\r\n");
    uart_write("$PUBX,40,GGA,0,0,0,0*5A\r\n");
    //TODO: make sure the acknoledgement packet is received. Currently it works
    __enable_interrupt();

    //poll the gps
    uart_write("$PUBX,00*33\r\n");
    // should expect to receive::
    //$PUBX,00,hhmmss.ss,Latitude,N,Longitude,E,AltRef,NavStat,Hacc,Vacc,SOG,COG,Vvel,ageC,HDOP,VDOP,TDOP,GU,RU,DR,*cs<CR><LF>


    
    // start up the radio    
    //Initialise Si4060 interface 
    si_trx_init();
    
    // maybe try to disable the UART interrupt after it is interrupted once.
    /*
    Strategy:
    get gps position and disable gps interrupts
    create telemetry string
    transmit telemetry string
    loop.
    */



    //Serial.print("$PUBX,40,GSA,0,0,0,0*4E\r\n");
    //Serial.print("$PUBX,40,RMC,0,0,0,0*47\r\n");
    while (1);


    /*
    while (1)
    {
   

    telemetry_start(TELEMETRY_PIPS, 5);

    // Sleep Wait 
    while (telemetry_active());
    
    telemetry_start(TELEMETRY_RTTY, 20);

    // Sleep Wait 
    while (telemetry_active());
    
    
    }
    */
}
          


