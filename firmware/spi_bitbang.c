/*
 * SPI bit-banging!
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
#include <stdint.h>
#include <iostm8s003f3.h>




void spi_bitbang_init()
{

  /* Configure the output pins */
  
  // Configure SCLK Pin  

    PC_DDR_DDR5 = 1;        //  Port C, bit 5 is output.
    PC_CR1_C15 = 1;         //  Pin is set to Push-Pull mode.
    PC_CR2_C25 = 1;         //  Pin can run upto 10 MHz. 
    
  // Configure MOSI Pin  

    PC_DDR_DDR6 = 1;        //  Port C, bit 6 is output.
    PC_CR1_C16 = 1;         //  Pin is set to Push-Pull mode.
    PC_CR2_C26 = 1;         //  Pin can run upto 10 MHz. 
    
  // Configure MISO Pin
    
    PC_DDR_DDR7 = 0;        //  Port C, bit & is input.
    PC_CR1_C17 = 1;         //  Pin is set to Pullup mode.
    PC_CR2_C27 = 0;         //  Pin is interrupt disabled. 
    
  
  /* Set output pins to default values */
    
    PC_ODR_ODR5 = 0;  //SCLK is low
    PC_ODR_ODR6 = 1;  //MOSI is high
 
}

uint8_t spi_bitbang_transfer(uint8_t byte)
{
  for (uint8_t counter = 0; counter < 8; counter++) {
    /* Set output data */
    if (byte & 0x80) {
      PC_ODR_ODR6 = 1;  //MOSI is high
    } else {
      PC_ODR_ODR6 = 0;  //MOSI is low
    }
    byte <<= 1;
    
    /* Latch Data into Slave */
    PC_ODR_ODR5 = 1; // SCK high
   

    /* Read Data */
    uint8_t temp = PC_IDR;
    if (temp & MASK_PC_IDR_IDR7) {
      byte |= 0x1;
    }

    /* Slave shifts out next data bit */
   PC_ODR_ODR5 = 0; // SCK low
  }

  return byte;
}
