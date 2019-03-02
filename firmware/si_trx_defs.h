/*
 * Definitions and macros for Si Labs Tranceivers
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

#ifndef SI_TRX_DEFS_H
#define SI_TRX_DEFS_H




/**
 * =============================================================================
 *                         COMMAND DEFINITIONS
 * =============================================================================
 */

/**
 * Si Boot Commands
 */
enum {
  SI_CMD_POWER_UP		= 0x02,
};
/**
 * Si Common Commands
 */
enum {
  SI_CMD_NOP			= 0x00,
  SI_CMD_PART_INFO		= 0x01,
  SI_CMD_FUNC_INFO		= 0x10,
  SI_CMD_SET_PROPERTY		= 0x11,
  SI_CMD_GET_PROPERTY		= 0x12,
  SI_CMD_GPIO_PIN_CFG		= 0x13,
  SI_CMD_FIFO_INFO		= 0x15,
  SI_CMD_GET_INT_STATUS		= 0x20,
  SI_CMD_REQUEST_DEVICE_STATE	= 0x33,
  SI_CMD_CHANGE_STATE		= 0x34,
  SI_CMD_READ_CMD_BUFF		= 0x44,
  SI_CMD_FRR_A_READ		= 0x50,
  SI_CMD_FRR_B_READ		= 0x51,
  SI_CMD_FRR_C_READ		= 0x53,
  SI_CMD_FRR_D_READ		= 0x57,
};
/**
 * Si Tx Commands
 */
enum {
  SI_CMD_START_TX		= 0x31,
  SI_CMD_WRITE_TX_FIFO		= 0x66,
};
/**
 * Si Rx Commands
 */
enum {
  SI_CMD_PACKET_INFO		= 0x16,
  SI_CMD_GET_MODEM_STATUS	= 0x22,
  SI_CMD_START_RX		= 0x32,
  SI_CMD_RX_HOP			= 0x36,
  SI_CMD_READ_RX_FIFO		= 0x77,
};
/**
 * Si Advanced Commands
 */
enum {
  SI_CMD_GET_ADC_READING	= 0x14,
  SI_CMD_PROTOCOL_CFG		= 0x18,
  SI_CMD_GET_PH_STATUS		= 0x21,
  SI_CMD_GET_CHIP_STATUS	= 0x23,
};

/**
 * =============================================================================
 *                         COMMAND ARGUMENTS
 * =============================================================================
 */

/**
 * Si Power Up
 */
enum {
  SI_POWER_UP_FUNCTION		= 0x01,
  SI_POWER_UP_XTAL		= 0x00,
  SI_POWER_UP_TCXO		= 0x01,
};

/**
 * Si GPIO configuration
 */
typedef uint8_t si_gpio_t;
enum {
  SI_GPIO_PIN_CFG_PULL_ENABLE 			= 0x40, /* enable or disable pull-up resistor */
  SI_GPIO_PIN_CFG_GPIO_MODE_DONOTHING		= 0x00, /* pin behaviour is not changed */
  SI_GPIO_PIN_CFG_GPIO_MODE_TRISTATE		= 0x01, /* input and output drivers are disabled */
  SI_GPIO_PIN_CFG_GPIO_MODE_DRIVE0		= 0x02, /* CMOS output "low" */
  SI_GPIO_PIN_CFG_GPIO_MODE_DRIVE1		= 0x03, /* CMOS output "high" */
  SI_GPIO_PIN_CFG_GPIO_MODE_INPUT		= 0x04, /* GPIO is input, for TXDATA etc, function is not configured here */
  SI_GPIO_PIN_CFG_GPIO_MODE_32K_CLK		= 0x05, /* outputs the 32kHz CLK when selected in CLK32_CLK_SEL */
  SI_GPIO_PIN_CFG_GPIO_MODE_BOOT_CLK		= 0x06, /* outputs boot clock when SPI_ACTIVE */
  SI_GPIO_PIN_CFG_GPIO_MODE_DIV_CLK		= 0x07, /* outputs divided xtal clk */
  SI_GPIO_PIN_CFG_GPIO_MODE_CTS			= 0x08, /* output, '1' when device is ready to accept new command */
  SI_GPIO_PIN_CFG_GPIO_MODE_INV_CNT		= 0x09, /* output, inverted CTS */
  SI_GPIO_PIN_CFG_GPIO_MODE_CMD_OVERLAP		= 0x0a, /* output, '1' if a command was issued while not ready */
  SI_GPIO_PIN_CFG_GPIO_MODE_SDO			= 0x0b, /* output, serial data out for SPI */
  SI_GPIO_PIN_CFG_GPIO_MODE_POR			= 0x0c, /* output, '0' while in POR state */
  SI_GPIO_PIN_CFG_GPIO_MODE_CAL_WUT		= 0x0d, /* output, '1' on expiration of wake up timer */
  SI_GPIO_PIN_CFG_GPIO_MODE_WUT			= 0x0e, /* wake up timer output */
  SI_GPIO_PIN_CFG_GPIO_MODE_EN_PA		= 0x0f, /* output, '1' when PA is enabled */
  SI_GPIO_PIN_CFG_GPIO_MODE_TX_DATA_CLK		= 0x10, /* data clock output, for TX direct sync mode */
  SI_GPIO_PIN_CFG_GPIO_MODE_TX_DATA		= 0x11, /* data output from TX FIFO, for debugging purposes */
  SI_GPIO_PIN_CFG_GPIO_MODE_IN_SLEEP		= 0x12, /* output, '0' when in sleep state */
  SI_GPIO_PIN_CFG_GPIO_MODE_TX_STATE		= 0x13, /* output, '1' when in TX state */
  SI_GPIO_PIN_CFG_GPIO_MODE_TX_FIFO_EMPTY	= 0x14, /* output, '1' when FIFO is empty */
  SI_GPIO_PIN_CFG_GPIO_MODE_LOW_BATT		= 0x15, /* output, '1' if low battery is detected */
  SI_GPIO_PIN_CFG_NIRQ_MODE_DONOTHING		= 0x00,
  SI_GPIO_PIN_CFG_SDO_MODE_DONOTHING		= 0x00,
  SI_GPIO_PIN_CFG_DRV_STRENGTH_HIGH		= (0x00 << 5),
  SI_GPIO_PIN_CFG_DRV_STRENGTH_MED_HIGH		= (0x01 << 5),
  SI_GPIO_PIN_CFG_DRV_STRENGTH_MED_LOW		= (0x02 << 5),
  SI_GPIO_PIN_CFG_DRV_STRENGTH_LOW		= (0x03 << 5),
};

/**
 * Si State Change
 */
enum {
  SI_STATE_CHANGE_NOCHANGE	= (0 << 8) | SI_CMD_CHANGE_STATE,
  SI_STATE_CHANGE_SLEEP		= (1 << 8) | SI_CMD_CHANGE_STATE,
  SI_STATE_CHANGE_SPI_ACTIVE	= (2 << 8) | SI_CMD_CHANGE_STATE,
  SI_STATE_CHANGE_READY		= (3 << 8) | SI_CMD_CHANGE_STATE,
  SI_STATE_CHANGE_TX_TUNE	= (5 << 8) | SI_CMD_CHANGE_STATE,
  SI_STATE_CHANGE_RX_TUNE	= (6 << 8) | SI_CMD_CHANGE_STATE,
  SI_STATE_CHANGE_TX		= (7 << 8) | SI_CMD_CHANGE_STATE,
  SI_STATE_CHANGE_RX		= (8 << 8) | SI_CMD_CHANGE_STATE,
};

/**
 * Si Get ADC reading
 */
enum {
  SI_GET_ADC_READING_TEMPERATURE	= (1 << 4),
  SI_GET_ADC_READING_BATTERY		= (1 << 3),
  SI_GET_ADC_READING_GPIO		= (1 << 2),
  SI_GET_ADC_READING_GPIO_PIN_GPIO3	= 3,
  SI_GET_ADC_READING_GPIO_PIN_GPIO2	= 2,
  SI_GET_ADC_READING_GPIO_PIN_GPIO1	= 1,
  SI_GET_ADC_READING_GPIO_PIN_GPIO0	= 0,
  SI_GET_ADC_READING_RANGE_0V8		= 0,
  SI_GET_ADC_READING_RANGE_1V6		= 4,
  SI_GET_ADC_READING_RANGE_3V2		= 5,
  SI_GET_ADC_READING_RANGE_2V4		= 8,
  SI_GET_ADC_READING_RANGE_3V6		= 9
};

/**
 * =============================================================================
 *                         PROPERTY DEFINITIONS
 * =============================================================================
 */

/**
 * Si Property Groups
 */
enum {
  SI_PROPERTY_GROUP_GLOBAL		= 0x00,
  SI_PROPERTY_GROUP_INT_CTL		= 0x01,
  SI_PROPERTY_GROUP_FRR_CTL		= 0x02,
  SI_PROPERTY_GROUP_PREAMBLE		= 0x10,
  SI_PROPERTY_GROUP_SYNC		= 0x11,
  SI_PROPERTY_GROUP_PKT			= 0x12,
  SI_PROPERTY_GROUP_MODEM		= 0x20,
  SI_PROPERTY_GROUP_PA			= 0x22,
  SI_PROPERTY_GROUP_SYNTH		= 0x23,
  SI_PROPERTY_GROUP_FREQ_CONTROL	= 0x40,
};
/**
 * Si Interrupt Control Properties
 */
enum {
  SI_INT_CTL_ENABLE			= 0x00,
  SI_INT_CTL_PH_ENABLE			= 0x01,
  SI_INT_CTL_CHIP_ENABLE		= 0x02,
};
/**
 * Si Global Properties
 */
enum {
  SI_GLOBAL_XO_TUNE			= 0x00,
  SI_GLOBAL_CONFIG			= 0x03,
};
/**
 * Si Preamble Properties
 */
enum {
  SI_PREAMBLE_TX_LENGTH			= 0x00,
};
/**
 * Si Sync Properties
 */
enum {
  SI_SYNC_CONFIG			= 0x11,
};
/**
 * Si Modem Properties
 */
enum {
  SI_MODEM_MOD_TYPE			= 0x00,
  SI_MODEM_MOD_TYPE_CW			= 0x00,
  SI_MODEM_MOD_TYPE_OOK			= 0x01,
  SI_MODEM_MOD_TYPE_2FSK		= 0x02, /* default */
  SI_MODEM_MOD_TYPE_2GFSK		= 0x03,
  SI_MODEM_MOD_TYPE_4FSK		= 0x04,
  SI_MODEM_MOD_TYPE_4GFSK		= 0x05,
  SI_MODEM_MOD_SOURCE_PACKET		= (0x00 << 3), /* default */
  SI_MODEM_MOD_SOURCE_DIRECT		= (0x01 << 3),
  SI_MODEM_MOD_SOURCE_PSEUDO		= (0x02 << 3),
  SI_MODEM_MOD_GPIO_0			= (0x00 << 5), /* default */
  SI_MODEM_MOD_GPIO_1			= (0x01 << 5),
  SI_MODEM_MOD_GPIO_2			= (0x02 << 5),
  SI_MODEM_MOD_GPIO_3			= (0x03 << 5),
  SI_MODEM_MOD_DIRECT_MODE_SYNC		= (0x00 << 7), /* default */
  SI_MODEM_MOD_DIRECT_MODE_ASYNC	= (0x01 << 7),
  SI_MODEM_FREQ_DEV			= 0x0a,
  SI_MODEM_FREQ_OFFSET			= 0x0d,
  SI_MODEM_CLKGEN_BAND			= 0x51,
  SI_MODEM_CLKGEN_SY_SEL_0		= (0x00 << 3), /* low power */
  SI_MODEM_CLKGEN_SY_SEL_1		= (0x01 << 3), /* default */
  SI_MODEM_CLKGEN_FVCO_DIV_4		= 0x00, /* default */
  SI_MODEM_CLKGEN_FVCO_DIV_6		= 0x01,
  SI_MODEM_CLKGEN_FVCO_DIV_8		= 0x02, /* for 70cm ISM band */
  SI_MODEM_CLKGEN_FVCO_DIV_12		= 0x03,
  SI_MODEM_CLKGEN_FVCO_DIV_16		= 0x04,
  SI_MODEM_CLKGEN_FVCO_DIV_24		= 0x05,
  SI_MODEM_CLKGEN_FVCO_DIV_24_2		= 0x06,
  SI_MODEM_CLKGEN_FVCO_DIV_24_3		= 0x07,
};
/**
 * Si PA Properties
 */
enum {
  SI_PA_MODE				= 0x00,
  SI_PA_PWR_LVL				= 0x01,
  SI_PA_BIAS_CLKDUTY			= 0x02,
  SI_PA_BIAS_CLKDUTY_SIN_25		= (0x03 << 6), /* for si4060 */
  SI_PA_BIAS_CLKDUTY_DIFF_50		= (0x00 << 6), /* for si4063 */
};
/**
 * Si Synthesizer Properties
 */
enum {
  SI_SYNTH_PFDCP_CPFF			= 0x00,
  SI_SYNTH_PFDCP_CPINT			= 0x01,
  SI_SYNTH_VCO_KV			= 0x02,
  SI_SYNTH_LPFILT3			= 0x03,
  SI_SYNTH_LPFILT2			= 0x04,
  SI_SYNTH_LPFILT1			= 0x05,
  SI_SYNTH_LPFILT0			= 0x06,
  SI_SYNTH_VCO_KVCAL			= 0x07,
};
/**
 * Si Frequency Control Properties
 */
enum {
  SI_FREQ_CONTROL_INTE			= 0x00,
  SI_FREQ_CONTROL_FRAC			= 0x01,
  SI_FREQ_CONTROL_CHANNEL_STEP_SIZE	= 0x04,
  SI_FREQ_CONTROL_W_SIZE		= 0x06,
};

/**
 * =============================================================================
 *                         Hardware Definitions
 * =============================================================================
 */

/**
 * Generic SPI Send / Receive
 */
void _si_trx_transfer(int tx_count, int rx_count, uint8_t *data);



/**
 * Shutdown. Active High (High = Shutdown, Low = Run)
 */
#define _si_trx_sdn_enable()			\
  PD_ODR_ODR4 = 1
#define _si_trx_sdn_disable()			\
  PD_ODR_ODR4 = 0

/**
 * HF Clock
 */
#define _si_trx_hf_clock_enable(void)		\
  /* NOT USED: Clock is always enabled */
#define _si_trx_hf_clock_disable(void)		\
  /* NOT USED: Clock is always enabled */


/**
 * =============================================================================
 *                         Helper Functions
 * =============================================================================
 */

/**
 * Convenience transfer functions
 */
static void _si_trx_transfer_uint16(uint16_t value)
{
  _si_trx_transfer(2, 0, (uint8_t*)&value);
}
static void _si_trx_set_property_8(uint8_t group, uint8_t property, uint8_t value)
{
  uint8_t buffer[5];

  buffer[0] = SI_CMD_SET_PROPERTY;
  buffer[1] = group;
  buffer[2] = 1;
  buffer[3] = property;
  buffer[4] = value;

  _si_trx_transfer(5, 0, buffer);
}
static void _si_trx_set_property_16(uint8_t group, uint8_t property, uint16_t value)
{
  uint8_t buffer[6];

  buffer[0] = SI_CMD_SET_PROPERTY;
  buffer[1] = group;
  buffer[2] = 2;
  buffer[3] = property;
  buffer[4] = (value >> 8);
  buffer[5] = (value);

  _si_trx_transfer(6, 0, buffer);
}
static void _si_trx_set_property_24(uint8_t group, uint8_t property, uint32_t value)
{
  uint8_t buffer[8];

  buffer[0] = SI_CMD_SET_PROPERTY;
  buffer[1] = group;
  buffer[2] = 3;
  buffer[3] = property;
  buffer[4] = (value >> 16);
  buffer[5] = (value >> 8);
  buffer[6] = (value);

  _si_trx_transfer(7, 0, buffer);
}
static void _si_trx_set_property_32(uint8_t group, uint8_t property, uint32_t value)
{
  uint8_t buffer[8];

  buffer[0] = SI_CMD_SET_PROPERTY;
  buffer[1] = group;
  buffer[2] = 4;
  buffer[3] = property;
  buffer[4] = (value >> 24);
  buffer[5] = (value >> 16);
  buffer[6] = (value >> 8);
  buffer[7] = (value);

  _si_trx_transfer(8, 0, buffer);
}

/**
 * State changes
 */
#define si_trx_state_ready()			\
  _si_trx_transfer_uint16(SI_STATE_CHANGE_READY)
/**
 * Change to TX tune state
 */
#define si_trx_state_tx_tune()				\
  _si_trx_transfer_uint16(SI_STATE_CHANGE_TX_TUNE)
/**
 * Change to RX tune state
 */
#define si_trx_state_rx_tune()				\
  _si_trx_transfer_uint16(SI_STATE_CHANGE_RX_TUNE)
/**
 * Change to TX state
 */
#define si_trx_state_tx()				\
  _si_trx_transfer_uint16(SI_STATE_CHANGE_TX)
/**
 * Change to RX state
 */
#define si_trx_state_rx()			\
  _si_trx_transfer_uint16(SI_STATE_CHANGE_RX)



#endif /* SI_TRX_DEFS_H */
