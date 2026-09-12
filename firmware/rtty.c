/*
 * Bit-bangs RTTY
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

#include "rtty.h"
#include "si_trx.h"
#include <iostm8s003f3.h>
#include <stdint.h>


/**
 * Interface to the physical world.
 */
#define RTTY_CHANNEL_DEVIATION	(RTTY_CHANNEL_SPACING / 2)
#define RTTY_CHANNEL(b)		(b ? RTTY_CHANNEL_DEVIATION : -RTTY_CHANNEL_DEVIATION)
#define RTTY_SET(b)		si_trx_switch_channel(RTTY_CHANNEL(b))

//#define RTTY_SET(b)		port_pin_set_output_level(SI406X_GPIO1_PIN, b);

/**
 * Formatting 8N2
 */
#define ASCII_BITS	8
#define BITS_PER_CHAR	11
#define PREAMBLE_LENGTH  50

/**
 * Current output data
 */
static uint8_t rtty_data;
/**
 * Where we currently are in the rtty output byte
 *
 * 0 = Start Bit
 * 1-8 = Data Bit
 * 10 = Stop Bit
 * 11 = Stop Bit
 */
static uint8_t rtty_phase = 0xFE;
static uint8_t rtty_preamble_count = 0;

void rtty_start(uint8_t data) {
  /* Start transmission */
  rtty_phase = 0;
  rtty_data = data;
}
void rtty_preamble(void) {
  rtty_preamble_count = PREAMBLE_LENGTH;
}

/**
 * Called at the baud rate, outputs bits of rtty
 */
uint8_t rtty_tick(void) {
  uint8_t bit;

  if (rtty_preamble_count > 0U) {
    rtty_preamble_count--;
    bit = 1U;
  } else {
    if (rtty_phase >= BITS_PER_CHAR) return RTTY_COMPLETE;

    if (rtty_phase == 0U) {
      bit = 0U;
    } else if (rtty_phase <= ASCII_BITS) {
      bit = (uint8_t)((rtty_data >> (rtty_phase - 1U)) & 1U);
    } else {
      bit = 1U;
    }

    rtty_phase++;
  }

  if (RTTY_SET(bit) != SI_TRX_OK) return RTTY_ERROR;
  return (rtty_preamble_count > 0U || rtty_phase < BITS_PER_CHAR) ?
         RTTY_ACTIVE : RTTY_COMPLETE;
}
