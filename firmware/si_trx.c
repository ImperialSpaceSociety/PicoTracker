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


#define RADIO_FREQUENCY	434600000
#define RADIO_POWER	0x18

#define VCXO_FREQUENCY	SI406X_TCXO_FREQUENCY
#define RF_DEVIATION	500



/**
* Generic SPI Send / Receive
*/
void _si_trx_transfer(int tx_count, int rx_count, uint8_t *data)
{
	uint8_t response;
	
	/* Send command */
	_si_trx_cs_enable();
	
	for (int i = 0; i < tx_count; i++) {
		spi_bitbang_transfer(data[i]);
	}
	
	_si_trx_cs_disable();
	
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
		for (int i = 0; i < 200; i++); /* Approx. 20µS */
		_si_trx_cs_enable();
		
		/* Issue READ_CMD_BUFF */
		spi_bitbang_transfer(SI_CMD_READ_CMD_BUFF);
		response = spi_bitbang_transfer(0xFF);
		
		/* If the reply is 0xFF, read the response */
		if (response == 0xFF) break;
		
		/* Otherwise repeat the procedure */
		_si_trx_cs_disable();
		
	} while (1); /* TODO: Timeout? */
	
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
	
	_si_trx_cs_disable();
}


/**
* Issues the POWER_UP command
*/
static void si_trx_power_up(uint8_t clock_source, uint32_t xo_freq)
{
	uint8_t buffer[7];
	
	buffer[0] = SI_CMD_POWER_UP;
	buffer[1] = SI_POWER_UP_FUNCTION;
	buffer[2] = clock_source;
	buffer[3] = (xo_freq >> 24);
	buffer[4] = (xo_freq >> 16);
	buffer[5] = (xo_freq >> 8);
	buffer[6] = (xo_freq);
	
	_si_trx_transfer(7, 0, buffer);
}
/**
* Gets the 16 bit part number
*/
static uint16_t si_trx_get_part_info(void)
{
	uint8_t buffer[3];
	
	buffer[0] = SI_CMD_PART_INFO;
	
	_si_trx_transfer(1, 3, buffer);
	
	return (buffer[1] << 8) | buffer[2];
}
/**
* Clears pending interrupts. Set the corresponding bit low to clear
* the interrupt.
*/
static void si_trx_clear_pending_interrupts(uint8_t packet_handler_clear_pending,
											uint8_t chip_clear_pending)
{
	uint8_t buffer[4];
	
	buffer[0] = SI_CMD_GET_INT_STATUS;
	buffer[1] = packet_handler_clear_pending & ((1<<5)|(1<<1)); /* Mask used bits */
	buffer[2] = 0;
	buffer[3] = chip_clear_pending;
	
	_si_trx_transfer(4, 0, buffer);
	
	/* This command returns the interrupts status, but we don't use it */
}
/**
* Sets the GPIO configuration for each pin
*/
static void si_trx_set_gpio_configuration(si_gpio_t gpio0, si_gpio_t gpio1,
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
	
	_si_trx_transfer(8, 0, buffer);
}
/**
* Starts transmitting
*/
static void si_trx_start_tx(uint8_t channel)
{
	uint8_t buffer[5];
	buffer[0] = SI_CMD_START_TX;
	buffer[1] = channel;
	buffer[2] = (1 << 4);
	buffer[3] = 0;
	buffer[4] = 0;
	
	_si_trx_transfer(5, 0, buffer);
}
/**
* Gets readings from the auxillary ADC
*/
static void si_trx_get_adc_reading(uint8_t enable, uint8_t configuration,
								   uint16_t* gpio_value,
								   uint16_t* battery_value,
								   uint16_t* temperature_value)
{
	uint8_t buffer[6];
	buffer[0] = SI_CMD_GET_ADC_READING;
	buffer[1] = enable;
	buffer[2] = configuration;
        
        _si_trx_sdn_enable();  /* active high shutdown = reset */
	
	for (int i = 0; i < 15*1000; i++); /* Approx. 15ms */
	_si_trx_sdn_disable();   /* booting */
	for (int i = 0; i < 15*1000; i++); /* Approx. 15ms */
	
	
	
	
	/* Power Up */
	 si_trx_power_up(SI_POWER_UP_XTAL, VCXO_FREQUENCY);

	_si_trx_transfer(3, 6, buffer);

        /* Physical shutdown */
	_si_trx_sdn_enable();
        
	*gpio_value = ((buffer[0] & 0x7) << 8) | buffer[1];
	*battery_value = ((buffer[2] & 0x7) << 8) | buffer[3];
	*temperature_value = ((buffer[4] & 0x7) << 8) | buffer[5];
}
/**
* Returns the measured internal die temperature of the radio
* as integer
*/
int16_t si_trx_get_temperature (void)
{
	
        uint16_t raw_gpio, raw_battery, raw_temperature;
	
	/* Get the reading from the adc */
	si_trx_get_adc_reading(SI_GET_ADC_READING_TEMPERATURE, 0xC5,
						   &raw_gpio, &raw_battery, &raw_temperature);
        
       
        uint32_t result = ((899 * (uint32_t) raw_temperature) / 4096) - 293;
        
	
	return result;
}


/* Returns the measured supply voltage of the radio
* in mV
*/
int16_t si_trx_get_voltage(void)
{
	
     
        
    uint16_t raw_gpio, raw_battery, raw_temperature;
	
	/* Get the reading from the adc */
	si_trx_get_adc_reading(SI_GET_ADC_READING_BATTERY, 0xC5,
						   &raw_gpio, &raw_battery, &raw_temperature);
	uint32_t result = ((uint32_t) raw_battery * 75) / 32; // result * 2.34375;
	return result;
}


/**
* Sets the internal frac-n pll synthesiser divisiors
*/
static void si_trx_frequency_control_set_divider(uint8_t integer_divider,
												 uint32_t fractional_divider)
{
	uint32_t divider = (fractional_divider & 0xFFFFFF) | ( (uint32_t) integer_divider << 24);
	
	_si_trx_set_property_32(SI_PROPERTY_GROUP_FREQ_CONTROL,
							SI_FREQ_CONTROL_INTE,
							divider);
}
/**
* Sets the output divider of the frac-n pll synthesiser
*/
static void si_trx_frequency_control_set_band(uint8_t band, uint8_t sy_sel)
{
	_si_trx_set_property_8(SI_PROPERTY_GROUP_MODEM,
						   SI_MODEM_CLKGEN_BAND,
						   sy_sel | (band & 0x7));
}
/**
* Sets the modem frequency deviation. This is how much the external
* pin deviates the synthesiser from the centre frequency. In units of
* the resolution of the frac-n pll synthsiser.
*
* This is an unsigned 17-bit value.
*/
static void si_trx_modem_set_deviation(uint32_t deviation)
{
	_si_trx_set_property_24(SI_PROPERTY_GROUP_MODEM,
							SI_MODEM_FREQ_DEV,
							deviation);
}
/**
* Sets the modem frequency offset manually. In units of the
* resolution of the frac-n pll synthsiser.
*
* This is a signed 16-bit value.
*/
static void si_trx_modem_set_offset(int16_t offset)
{
	_si_trx_set_property_16(SI_PROPERTY_GROUP_MODEM,
							SI_MODEM_FREQ_OFFSET,
							offset);
}

/**
* Sets the modulation mode
*/
static void si_trx_modem_set_modulation(uint8_t tx_direct_mode,
										uint8_t tx_direct_gpio,
										uint8_t tx_modulation_source,
										uint8_t modulation_type)
{
	_si_trx_set_property_8(SI_PROPERTY_GROUP_MODEM, SI_MODEM_MOD_TYPE,
						   tx_direct_mode | tx_direct_gpio |
							   tx_modulation_source | modulation_type);
}
/**
* Sets the tx power
*/
static void si_trx_set_tx_power(uint8_t tx_power)
{
	_si_trx_set_property_8(SI_PROPERTY_GROUP_PA, SI_PA_PWR_LVL, tx_power);
}

/**
* Set the synthesiser to the given frequency.
*
* frequency: Floating-point value for the frequency
* deviation: FSK-mode deviation, in channels. Usually 1
*
* Returns the LSB tuning resolution of the frac-n pll synthesiser.
*/
static float si_trx_set_frequency(uint32_t frequency, uint16_t deviation)
{
	uint8_t outdiv, band, nprescaler;
	
	/* Higher frequency resolution, but also higher power (~+200µA) */
	nprescaler = 2;
	
	
	if (frequency < 705000000UL) {
		outdiv = 6;  band = SI_MODEM_CLKGEN_FVCO_DIV_6;
	}
	if (frequency < 525000000UL) {
		outdiv = 8;  band = SI_MODEM_CLKGEN_FVCO_DIV_8;
	}
	if (frequency < 353000000UL) {
		outdiv = 12; band = SI_MODEM_CLKGEN_FVCO_DIV_12;
	}
	if (frequency < 239000000UL) {
		outdiv = 16; band = SI_MODEM_CLKGEN_FVCO_DIV_16;
	}
	if (frequency < 177000000UL) {
		outdiv = 24; band = SI_MODEM_CLKGEN_FVCO_DIV_24;
	}
	
	float f_pfd = nprescaler * VCXO_FREQUENCY / outdiv;
	
	uint16_t n = ((uint16_t)(frequency / f_pfd)) - 1;
	
	float ratio = (float)frequency / f_pfd;
	float rest  = ratio - (float)n;
	
	uint32_t m = (uint32_t)(rest * (float)( (uint32_t) 1 << 19));
	
	
	/* Check n and m are in valid ranges, halt otherwise */
	if (n > 0x7f || m > 0xfffff) while (1);
	
	
	/* Set the frac-n PLL output divider */
	if (nprescaler == 4) { /* Prescaler */
		si_trx_frequency_control_set_band(band, SI_MODEM_CLKGEN_SY_SEL_0);
	} else { /* Default Mode */
		si_trx_frequency_control_set_band(band, SI_MODEM_CLKGEN_SY_SEL_1);
	}
	
	
	/* Set the frac-n PLL divisior */
	si_trx_frequency_control_set_divider(n, m);
	
	/* Set the external pin frequency deviation to the LSB tuning resoultion */
	si_trx_modem_set_deviation(deviation);
	
	/* Return the LSB tuning resolution of the frac-n pll synthesiser. */
	return f_pfd / (float)((uint32_t) 1 << 19);
}

/**
* Resets the transceiver
*/
void si_trx_reset(uint8_t modulation_type, uint16_t deviation)
{
	_si_trx_sdn_enable();  /* active high shutdown = reset */
	
	for (int i = 0; i < 15*1000; i++); /* Approx. 15ms */
	_si_trx_sdn_disable();   /* booting */
	for (int i = 0; i < 15*1000; i++); /* Approx. 15ms */
	
	
	uint16_t part_number = si_trx_get_part_info();
	
	/* Power Up */
	si_trx_power_up(SI_POWER_UP_XTAL, VCXO_FREQUENCY);
	
	/* Clear pending interrupts */
	si_trx_clear_pending_interrupts(0, 0);
	
	/* Disable all interrupts */
	_si_trx_set_property_8(SI_PROPERTY_GROUP_INT_CTL, SI_INT_CTL_ENABLE, 0);
	
	/* Configure GPIOs */
	si_trx_set_gpio_configuration(SI_GPIO_PIN_CFG_GPIO_MODE_INPUT | SI_GPIO_PIN_CFG_PULL_ENABLE,
								  SI_GPIO_PIN_CFG_GPIO_MODE_INPUT | SI_GPIO_PIN_CFG_PULL_ENABLE,
								  SI_GPIO_PIN_CFG_GPIO_MODE_DRIVE1,
								  SI_GPIO_PIN_CFG_GPIO_MODE_DRIVE0,
								  SI_GPIO_PIN_CFG_DRV_STRENGTH_LOW);
	
	si_trx_set_frequency(RADIO_FREQUENCY, deviation);
	si_trx_set_tx_power(RADIO_POWER);
	
	/* RTTY from GPIO1 */
	si_trx_modem_set_modulation(SI_MODEM_MOD_DIRECT_MODE_ASYNC,
								SI_MODEM_MOD_GPIO_1,
								SI_MODEM_MOD_SOURCE_DIRECT,
								modulation_type);
	
	si_trx_state_tx_tune();
}

/**
* Enables the radio and starts transmitting
*/
void si_trx_on(uint8_t modulation_type, uint16_t deviation)
{
	si_trx_reset(modulation_type, deviation);
	si_trx_start_tx(0);
}
/**
* Disables the radio and places it in shutdown
*/
void si_trx_off(void)
{
	si_trx_state_ready();
	
	/* Physical shutdown */
	_si_trx_sdn_enable();
}

/**
* Switches the transmission to the specified channel. Signed 16-bit int
*/
void si_trx_switch_channel(int16_t channel)
{
	si_trx_modem_set_offset(channel);
}

/**
* Initialises the radio interface to the radio
*/
void si_trx_init(void)
{
  /* Configure the SDN pin */
 
    PD_DDR_DDR4 = 1;        //  Port D, bit 4 is output.
    PD_CR1_C14 = 1;         //  Pin is set to Push-Pull mode.
    PD_CR2_C24 = 1;         //  Pin can run upto 10 MHz. 
 
  /* Put the transciever in shutdown */
  _si_trx_sdn_enable();

  /* Configure the SPI select pin */
#ifdef SILABS_QFN             
    PD_DDR_DDR3 = 1;        //  Port D, bit 3 is output for QFN.
    PD_CR1_C13 = 1;         //  Pin is set to Push-Pull mode.
    PD_CR2_C23 = 1;         //  Pin can run upto 10 MHz. 
#else
    PD_DDR_DDR2 = 1;        //  Port D, bit 2 is output for TSSOP.
    PD_CR1_C12 = 1;         //  Pin is set to Push-Pull mode.
    PD_CR2_C22 = 1;         //  Pin can run upto 10 MHz. 
#endif

  /* Put the SEL pin in reset */
  _si_trx_cs_disable();

  /* Configure the GPIO pins */
    PB_DDR_DDR4 = 0;        //  GPIO0 Port B, bit 4 is input.
    PB_CR1_C14 = 1;         //  Pin is set to Pullup.
    PB_CR2_C24 = 0;         //  Pin is set to NO Interrupt. 
    
    
    
    PC_DDR_DDR3 = 1;        //  GPIO1 Port C, bit 3 is output.
    PC_CR1_C13 = 1;         //  Pin is set to Push-Pull mode.
    PC_CR2_C23 = 1;         //  Pin can run upto 10 MHz. 
    
    PC_ODR_ODR3 = 0;        // GPIO1 Modulation = 0
    
    
 

  /* Configure the IRQ pin */
  // TODO

  /* Configure the serial port */
  spi_bitbang_init();
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
