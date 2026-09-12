/*
* Functions for controlling Si Labs Transceivers
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
#include <iostm8s003f3.h>
#include <stdint.h>
#include "HC12Board.h"
#include "si_trx.h"
#include "spi_bitbang.h"
#include "si_trx_defs.h"
#include "main.h"
#include "radio_measurements.h"


#define RF_DEVIATION	500



static uint8_t radio_select_pin = 3;

static void si_trx_delay_cycles(uint16_t cycles)
{
    volatile uint16_t remaining = cycles;
    while (remaining > 0U) remaining--;
}

static void si_trx_xo_power_init(void)
{
#ifdef XO_TCXO
    PA_DDR_DDR3 = 1;
    PA_CR1_C13 = 1;
    PA_CR2_C23 = 1;
    PA_ODR_ODR3 = 0;
#endif
}

static void si_trx_xo_power_on(void)
{
#ifdef XO_TCXO
    PA_ODR_ODR3 = 1;
    si_trx_delay_cycles(5000U);
#endif
}

static void si_trx_xo_power_off(void)
{
#ifdef XO_TCXO
    PA_ODR_ODR3 = 0;
#endif
}

/**
* Generic SPI Send / Receive
*/
uint8_t _si_trx_transfer(int tx_count, int rx_count, uint8_t *data)
{
	uint8_t response;
	uint16_t cts_poll_count = 0;
	
	/* Send command */
        
        /* Enable select */
        if (radio_select_pin == 3) //QFN radio
          PD_ODR_ODR3 = 0;
        else
          PD_ODR_ODR2 = 0;
	
	for (int i = 0; i < tx_count; i++) {
		spi_bitbang_transfer(data[i]);
	}
	
	/* Disable select */
        if (radio_select_pin == 3)
          PD_ODR_ODR3 = 1;
        else
          PD_ODR_ODR2 = 1;
        
	
	/**
	* Poll CTS. From the docs:
	*
	* READ_CMD_BUFF is used to poll the CTS signal via the SPI bus. The
	* NSEL line should be pulled low, followed by sending the
	* READ_CMD_BUFF command on SDI. While NSEL remains asserted low, an
	* additional eight clock pulses are sent on SCLK and the CTS
	* response byte is read on SDO. If the CTS response byte is not
	* 0xFF, the host MCU should pull NSEL high and repeat the polling
	* procedure.
	*/
	
	do {
		si_trx_delay_cycles(200U); /* Approx. 20 us */
		
                /* Enable select */
                if (radio_select_pin == 3)
                    PD_ODR_ODR3 = 0;
                else
                    PD_ODR_ODR2 = 0;
		
		/* Issue READ_CMD_BUFF */
		spi_bitbang_transfer(SI_CMD_READ_CMD_BUFF);
		response = spi_bitbang_transfer(0xFF);
		
		/* If the reply is 0xFF, read the response */
		if (response == 0xFF) break;
		
		/* Otherwise repeat the procedure */

		/* Disable select */
                if (radio_select_pin == 3)
                    PD_ODR_ODR3 = 1;
                else
                    PD_ODR_ODR2 = 1;

		cts_poll_count++;
		if (cts_poll_count >= SI_TRX_CTS_POLL_LIMIT) {
			return SI_TRX_ERROR;
		}
	} while (1);
	
	/**
	* Read response. From the docs:
	*
	* If the CTS response byte is 0xFF, the host MCU should keep NSEL
	* asserted low and provide additional clock cycles on SCLK to read
	* out as many response bytes (on SDO) as necessary. The host MCU
	* should pull NSEL high upon completion of reading the response
	* stream.
	*/
	for (int i = 0; i < rx_count; i++) {
		data[i] = spi_bitbang_transfer(0xFF);
	}
	
	/* Disable select */
                if (radio_select_pin == 3)
                    PD_ODR_ODR3 = 1;
                else
                    PD_ODR_ODR2 = 1;

	return SI_TRX_OK;
}


/**
* Issues the POWER_UP command
*/
static uint8_t si_trx_power_up(uint8_t clock_source, uint32_t xo_freq)
{
	uint8_t buffer[7];
	
	buffer[0] = SI_CMD_POWER_UP;
	buffer[1] = SI_POWER_UP_FUNCTION;
	buffer[2] = clock_source;
	buffer[3] = (uint8_t)(xo_freq >> 24);
	buffer[4] = (uint8_t)(xo_freq >> 16);
	buffer[5] = (uint8_t)(xo_freq >> 8);
	buffer[6] = (uint8_t)xo_freq;
	
	return _si_trx_transfer(7, 0, buffer);
}

static uint8_t si_trx_boot(void)
{
	/* Power the configured oscillator, reset the radio, then issue POWER_UP. */
	si_trx_xo_power_on();

	_si_trx_sdn_enable();
	si_trx_delay_cycles(15000U);
	_si_trx_sdn_disable();
	si_trx_delay_cycles(15000U);

	return si_trx_power_up(XO_SOURCE, XO_FREQUENCY);
}

/**
* Gets the 16 bit part number
*/
static uint16_t si_trx_get_part_info(void)
{
	uint8_t buffer[3];
	
	buffer[0] = SI_CMD_PART_INFO;
	
	if (_si_trx_transfer(1, 3, buffer) != SI_TRX_OK) {
		return 0;
	}
	
	return (uint16_t)(((uint16_t)buffer[1] << 8) | (uint16_t)buffer[2]);
}
/**
* Clears pending interrupts. Set the corresponding bit low to clear
* the interrupt.
*/
static uint8_t si_trx_clear_pending_interrupts(uint8_t packet_handler_clear_pending,
											uint8_t chip_clear_pending)
{
	uint8_t buffer[4];
	
	buffer[0] = SI_CMD_GET_INT_STATUS;
	buffer[1] = packet_handler_clear_pending & ((1<<5)|(1<<1)); /* Mask used bits */
	buffer[2] = 0;
	buffer[3] = chip_clear_pending;
	
	return _si_trx_transfer(4, 0, buffer);
	
	/* This command returns the interrupts status, but we don't use it */
}
/**
* Sets the GPIO configuration for each pin
*/
static uint8_t si_trx_set_gpio_configuration(si_gpio_t gpio0, si_gpio_t gpio1,
										  si_gpio_t gpio2, si_gpio_t gpio3,
										  uint8_t drive_strength)
{
	uint8_t buffer[8];
	buffer[0] = SI_CMD_GPIO_PIN_CFG;
	buffer[1] = gpio0;
	buffer[2] = gpio1;
	buffer[3] = gpio2;
	buffer[4] = gpio3;
	buffer[5] = SI_GPIO_PIN_CFG_NIRQ_MODE_DONOTHING;
	buffer[6] = SI_GPIO_PIN_CFG_SDO_MODE_DONOTHING;
	buffer[7] = drive_strength;
	
	return _si_trx_transfer(8, 0, buffer);
}
/**
* Starts transmitting
*/
static uint8_t si_trx_start_tx(uint8_t channel)
{
	uint8_t buffer[5];
	buffer[0] = SI_CMD_START_TX;
	buffer[1] = channel;
	buffer[2] = (1 << 4);
	buffer[3] = 0;
	buffer[4] = 0;
	
	return _si_trx_transfer(5, 0, buffer);
}
/**
* Gets readings from the auxiliary ADC
*/
static uint8_t si_trx_get_adc_reading(uint8_t enable, uint8_t configuration,
                                      uint16_t *gpio_value,
                                      uint16_t *battery_value,
                                      uint16_t *temperature_value)
{
    uint8_t buffer[6] = {0};
    uint8_t status = SI_TRX_ERROR;

    *gpio_value = 0;
    *battery_value = 0;
    *temperature_value = 0;

    buffer[0] = SI_CMD_GET_ADC_READING;
    buffer[1] = enable;
    buffer[2] = configuration;

    if (si_trx_boot() == SI_TRX_OK &&
        _si_trx_transfer(3, 6, buffer) == SI_TRX_OK) {
        *gpio_value = (uint16_t)(((uint16_t)(buffer[0] & 0x07U) << 8) | buffer[1]);
        *battery_value = (uint16_t)(((uint16_t)(buffer[2] & 0x07U) << 8) | buffer[3]);
        *temperature_value = (uint16_t)(((uint16_t)(buffer[4] & 0x07U) << 8) | buffer[5]);
        status = SI_TRX_OK;
    }

    _si_trx_sdn_enable();
    si_trx_xo_power_off();
    return status;
}

/**
* Reads the measured internal die temperature of the radio.
*/
uint8_t si_trx_get_temperature(int16_t *temperature)
{
    uint16_t raw_gpio, raw_battery, raw_temperature;

    if (si_trx_get_adc_reading(SI_GET_ADC_READING_TEMPERATURE, 0xC5,
                               &raw_gpio, &raw_battery, &raw_temperature) != SI_TRX_OK) {
        return SI_TRX_ERROR;
    }

    *temperature = si_trx_temperature_from_raw(raw_temperature);
    return SI_TRX_OK;
}

/**
* Reads the measured supply voltage of the radio in mV.
*/
uint8_t si_trx_get_voltage(uint16_t *voltage)
{
    uint16_t raw_gpio, raw_battery, raw_temperature;
    uint32_t result;

    if (si_trx_get_adc_reading(SI_GET_ADC_READING_BATTERY, 0xC5,
                               &raw_gpio, &raw_battery, &raw_temperature) != SI_TRX_OK) {
        return SI_TRX_ERROR;
    }

    result = ((uint32_t)raw_battery * 75U) / 32U;
    *voltage = (uint16_t)result;
    return SI_TRX_OK;
}


/**
* Sets the internal frac-n pll synthesiser dividers
*/
static uint8_t si_trx_frequency_control_set_divider(uint8_t integer_divider,
												 uint32_t fractional_divider)
{
	uint32_t divider = (fractional_divider & 0xFFFFFF) | ( (uint32_t) integer_divider << 24);
	
	return _si_trx_set_property_32(SI_PROPERTY_GROUP_FREQ_CONTROL,
							SI_FREQ_CONTROL_INTE,
							divider);
}
/**
* Sets the output divider of the frac-n pll synthesiser
*/
static uint8_t si_trx_frequency_control_set_band(uint8_t band, uint8_t sy_sel)
{
	return _si_trx_set_property_8(SI_PROPERTY_GROUP_MODEM,
						   SI_MODEM_CLKGEN_BAND,
						   sy_sel | (band & 0x7));
}
/**
* Sets the modem frequency deviation. This is how much the external
* pin deviates the synthesiser from the centre frequency. In units of
* the resolution of the frac-n pll synthesiser.
*
* This is an unsigned 17-bit value.
*/
static uint8_t si_trx_modem_set_deviation(uint32_t deviation)
{
	return _si_trx_set_property_24(SI_PROPERTY_GROUP_MODEM,
							SI_MODEM_FREQ_DEV,
							deviation);
}
/**
* Sets the modem frequency offset manually. In units of the
* resolution of the frac-n pll synthesiser.
*
* This is a signed 16-bit value.
*/
static uint8_t si_trx_modem_set_offset(int16_t offset)
{
	return _si_trx_set_property_16(SI_PROPERTY_GROUP_MODEM,
							SI_MODEM_FREQ_OFFSET,
							(uint16_t)offset);
}

/**
* Sets the modulation mode
*/
static uint8_t si_trx_modem_set_modulation(uint8_t tx_direct_mode,
										uint8_t tx_direct_gpio,
										uint8_t tx_modulation_source,
										uint8_t modulation_type)
{
	return _si_trx_set_property_8(SI_PROPERTY_GROUP_MODEM, SI_MODEM_MOD_TYPE,
						   tx_direct_mode | tx_direct_gpio |
							   tx_modulation_source | modulation_type);
}
/**
* Sets the tx power
*/
static uint8_t si_trx_set_tx_power(uint8_t tx_power)
{
	return _si_trx_set_property_8(SI_PROPERTY_GROUP_PA, SI_PA_PWR_LVL, tx_power);
}

/**
* Set the synthesiser to the given frequency.
*
* frequency: Floating-point value for the frequency
* deviation: FSK-mode deviation, in channels. Usually 1
*
* Returns SI_TRX_OK on success or SI_TRX_ERROR if configuration fails.
*/
static uint8_t si_trx_set_frequency(uint32_t frequency, uint16_t deviation)
{
	uint8_t outdiv, band, nprescaler;
	
	/* Higher frequency resolution, but also higher power (~+200µA) */
	nprescaler = 2;
	
	
	if (frequency < 119000000UL || frequency > 1050000000UL) return SI_TRX_ERROR;

	if (frequency >= 705000000UL) {
		outdiv = 4;  band = SI_MODEM_CLKGEN_FVCO_DIV_4;
	} else if (frequency >= 525000000UL) {
		outdiv = 6;  band = SI_MODEM_CLKGEN_FVCO_DIV_6;
	} else if (frequency >= 353000000UL) {
		outdiv = 8;  band = SI_MODEM_CLKGEN_FVCO_DIV_8;
	} else if (frequency >= 239000000UL) {
		outdiv = 12; band = SI_MODEM_CLKGEN_FVCO_DIV_12;
	} else if (frequency >= 177000000UL) {
		outdiv = 16; band = SI_MODEM_CLKGEN_FVCO_DIV_16;
	} else {
		outdiv = 24; band = SI_MODEM_CLKGEN_FVCO_DIV_24;
	}
	
	float f_pfd = ((float)nprescaler * (float)XO_FREQUENCY) / (float)outdiv;
	
	uint16_t n = ((uint16_t)((float)frequency / f_pfd)) - 1U;
	
	float ratio = (float)frequency / f_pfd;
	float rest  = ratio - (float)n;
	
	uint32_t m = (uint32_t)(rest * (float)( (uint32_t) 1 << 19));
	
	
	/* Reject divider values outside the Si4463 property ranges. */
	if (n > 0x7f || m > 0xfffff) return SI_TRX_ERROR;
	
	
	/* Set the frac-n PLL output divider */
	if (nprescaler == 4) { /* Prescaler */
		if (si_trx_frequency_control_set_band(band, SI_MODEM_CLKGEN_SY_SEL_0) != SI_TRX_OK) return SI_TRX_ERROR;
	} else { /* Default Mode */
		if (si_trx_frequency_control_set_band(band, SI_MODEM_CLKGEN_SY_SEL_1) != SI_TRX_OK) return SI_TRX_ERROR;
	}
	
	
	/* Set the frac-n PLL divider */
	if (si_trx_frequency_control_set_divider((uint8_t)n, m) != SI_TRX_OK) return SI_TRX_ERROR;
	
	/* Set the external pin frequency deviation to the LSB tuning resolution */
	if (si_trx_modem_set_deviation(deviation) != SI_TRX_OK) return SI_TRX_ERROR;
	
	return SI_TRX_OK;
}

/**
* Resets the transceiver
*/
static uint8_t si_trx_reset(uint8_t modulation_type, uint16_t deviation)
{
	if (si_trx_boot() != SI_TRX_OK) return SI_TRX_ERROR;
	
	/* Clear pending interrupts */
	if (si_trx_clear_pending_interrupts(0, 0) != SI_TRX_OK) return SI_TRX_ERROR;
	
	/* Disable all interrupts */
	if (_si_trx_set_property_8(SI_PROPERTY_GROUP_INT_CTL, SI_INT_CTL_ENABLE, 0) != SI_TRX_OK) return SI_TRX_ERROR;
	
	/* Configure GPIOs */
	if (si_trx_set_gpio_configuration(SI_GPIO_PIN_CFG_GPIO_MODE_INPUT | SI_GPIO_PIN_CFG_PULL_ENABLE,
                                      SI_GPIO_PIN_CFG_GPIO_MODE_INPUT | SI_GPIO_PIN_CFG_PULL_ENABLE,
                                      SI_GPIO_PIN_CFG_GPIO_MODE_DRIVE1,
                                      SI_GPIO_PIN_CFG_GPIO_MODE_DRIVE0,
                                      SI_GPIO_PIN_CFG_DRV_STRENGTH_LOW) != SI_TRX_OK) return SI_TRX_ERROR;
	
	if (si_trx_set_frequency(RADIO_FREQUENCY, deviation) != SI_TRX_OK) return SI_TRX_ERROR;
	if (si_trx_set_tx_power(RADIO_POWER) != SI_TRX_OK) return SI_TRX_ERROR;
	
	/* RTTY from GPIO1 */
	if (si_trx_modem_set_modulation(SI_MODEM_MOD_DIRECT_MODE_ASYNC,
								SI_MODEM_MOD_GPIO_1,
								SI_MODEM_MOD_SOURCE_DIRECT,
								modulation_type) != SI_TRX_OK) return SI_TRX_ERROR;
	
	if (si_trx_state_tx_tune() != SI_TRX_OK) return SI_TRX_ERROR;

	return SI_TRX_OK;
}

/**
* Enables the radio and starts transmitting.
* Returns SI_TRX_OK on success or SI_TRX_ERROR on startup failure.
*/
uint8_t si_trx_on(uint8_t modulation_type, uint16_t deviation)
{
	if (si_trx_reset(modulation_type, deviation) != SI_TRX_OK ||
	    si_trx_start_tx(0) != SI_TRX_OK) {
		_si_trx_sdn_enable();
		si_trx_xo_power_off();
		return SI_TRX_ERROR;
	}

	return SI_TRX_OK;
}
/**
* Disables the radio and places it in shutdown
*/
void si_trx_off(void)
{
	si_trx_state_ready();
	
	/* Physical shutdown */
	_si_trx_sdn_enable();
        
        /* Power off the external oscillator when present. */
        si_trx_xo_power_off();
}

/**
* Switches the transmission to the specified channel. Signed 16-bit int
*/
uint8_t si_trx_switch_channel(int16_t channel)
{
	return si_trx_modem_set_offset(channel);
}

/**
* Initialises the radio interface to the radio
*/
void si_trx_init(void)
{
  /* Configure oscillator power control when a TCXO is installed. */
    si_trx_xo_power_init();

  /* Configure the SDN pin */
 
    PD_DDR_DDR4 = 1;        //  Port D, bit 4 is output.
    PD_CR1_C14 = 1;         //  Pin is set to Push-Pull mode.
    PD_CR2_C24 = 1;         //  Pin can run up to 10 MHz.
    
  /* Put the transciever in shutdown */
  _si_trx_sdn_enable();
  
 /* Configure the SPI serial port */
  spi_bitbang_init(); 
  
 /* Determine the SPI select Pin 
 *  this is different for the TSSOP and QFN versions
 *  Port D bit 3 for QFN
 *  Port D bit 2 for TSSOP */

  /* Configure the SPI select pin for QFN*/

  
    PD_DDR_DDR3 = 1;        //  Port D, bit 3 is output for QFN.
    PD_CR1_C13 = 1;         //  Pin is set to Push-Pull mode.
    PD_CR2_C23 = 1;         //  Pin can run up to 10 MHz.
    PD_ODR_ODR3 = 1;        //  Select is high
    
   /* Probe the QFN select pin using a complete boot sequence. */
    uint16_t part_number = 0;
    if (si_trx_boot() == SI_TRX_OK) {
        part_number = si_trx_get_part_info();
    }
    if (part_number != 0x4463 && part_number != 0x4438 ){ // Radio chip might be Si4463 or Si4438
      
        radio_select_pin =2;    //  TSSOP pin
        PD_DDR_DDR3 = 0;        //  Port D, bit 3 is input.
        PD_CR1_C13 = 0;         //  Pin has no pullup
        PD_CR2_C23 = 0;         //  Pin has no interrupt
        PD_DDR_DDR2 = 1;        //  Port D, bit 2 is output for TSSOP.
        PD_CR1_C12 = 1;         //  Pin is set to Push-Pull mode.
        PD_CR2_C22 = 1;         //  Pin can run up to 10 MHz.

        part_number = 0;
        if (si_trx_boot() == SI_TRX_OK) {
            part_number = si_trx_get_part_info();
        }
    }
    _si_trx_sdn_enable();  /* active high shutdown = reset */
    si_trx_xo_power_off();

  /* Configure the GPIO pins */
    PB_DDR_DDR4 = 0;        //  GPIO0 Port B, bit 4 is input.
    PB_CR1_C14 = 1;         //  Pin is set to pull-up.
    PB_CR2_C24 = 0;         //  Pin is set to NO Interrupt. 
    
   
    
    PC_DDR_DDR3 = 1;        //  GPIO1 Port C, bit 3 is output.
    PC_CR1_C13 = 1;         //  Pin is set to Push-Pull mode.
    PC_CR2_C23 = 1;         //  Pin can run up to 10 MHz.
    
    PC_ODR_ODR3 = 0;        // GPIO1 Modulation = 1
    
    
 

  /* nIRQ is not used in the direct-transmit path; command completion is polled through CTS over SPI. */

}




/**
* Quick and dirty loopback test. Should print 0x34
*/
uint8_t spi_loopback_test(void)
{
	
	
	/* Init loopback */
	spi_bitbang_init();
	
	/* Enable */
	
	/* Test transfer */
	uint8_t data = spi_bitbang_transfer(0x34);
	
	return data;
}
