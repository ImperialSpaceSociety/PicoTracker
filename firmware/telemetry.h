/*
 * Telemetry strings and formatting
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

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <iostm8s003f3.h>
#include <stdint.h>
#include <inttypes.h>


enum telemetry_t {
  TELEMETRY_RTTY,
  TELEMETRY_PIPS,
};

/**
 * Output String
 */
#define TELEMETRY_STRING_MAX	0x100


int telemetry_active(void);
int telemetry_start(enum telemetry_t type, int8_t length);
void timer1_tick_init(uint16_t millisecs);
void timer1_tick_time(uint16_t millisecs);
void timer1_tick_deinit();
void init_tx_buffer(void);
void prepare_tx_buffer(void);


#endif /* TELEMETRY_H */
