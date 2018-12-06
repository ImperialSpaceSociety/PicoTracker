/* HC12 Si4463 Radio board file
*  Silabs si4463 radio with STM8S003F3 microcontroller
*
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
* Pin10 PA3
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
#define SI406X_TCXO_FREQUENCY	30000000L
#define HSCLK_FREQUENCY         16000000L

