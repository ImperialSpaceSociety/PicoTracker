/* HC12 Si4463 Radio board definitions file
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
* */


/*
* STM8 Processor IO
* 
* Pin1  PD4             Si4463 p1 - SDN
* Pin2  PD5/UART1_TX    TXD Header Pin via level shifter
* Pin3  PD6/UART1_Rx    RXD Header Pin via level shifter
* Pin4  nRST            RESET test point near RXD pad
* Pin5  OSCIN
* Pin6  OSCOUT
* Pin7  VSS
* Pin8  VCAP
* Pin9  VDD
* Pin10 PA3             TCXO Enable Output
* Pin11 PB5             SET Header Pin via level shifter
* Pin12 PB4             Si4463 p9 - GPIO0
* Pin13 PC3             Si4463 p10 - GPIO1
* Pin14 PC4             Si4463 p11 - nIRQ
* Pin15 PC5/SPI_SCK     Si4463 p12 - SCLK
* Pin16 PC6/SPI_MOSI    Si4463 p14 - SDI
* Pin17 PC7/SPI_MISO    Si4463 p13 - SDO
* Pin18 PD1/SWIM        SWIM test point near TXD pad
* Pin19 PD2             Si4463 p15 - nSEL
* Pin20 PD3
*
*
* Si4463 GPIO
* GPIO0 Transmit state
* GPIO1 CTS clear to send next command
* GPIO2 Antenna switch 2
* GPIO3 Antenna switch 1
* nIRQ  Receive detected
*/



#define SI406X_TCXO_FREQUENCY	30000000L  // si4463 clock frequency Crystal
//#define SI406X_TCXO_FREQUENCY	32000000L  // si4463 clock frequency TYCXO

#define HSCLK_FREQUENCY         16000000L  // Processor internal clock frequency

#define GPS_UART        UART1