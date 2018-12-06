/*
 * si4060 software library
 *
 * Stefan Biereigel
 *
 */
#ifndef SI4060_H_
#define SI4060_H_

#include <inttypes.h>
#include "samd20.h"
#include "system/port.h"
#include "hw_config.h"


/**
 * Chip Select. Active Low (High = Inactive, Low = Active)
 */
#define _si406x_cs_enable()			\
  port_pin_set_output_level(SI406X_SEL_PIN, 0)
#define _si406x_cs_disable()			\
  port_pin_set_output_level(SI406X_SEL_PIN, 1)

#define spi_select _si406x_cs_enable
#define spi_deselect _si406x_cs_disable
#define spi_write spi_bitbang_transfer
#define spi_read() spi_bitbang_transfer(0xff)

#define __delay_cycles(n) do {				\
    for (int cyc = 0; cyc < n; cyc++) { __NOP(); }	\
  } while(0)

void si4060_shutdown(void);


#define XO_FREQ 16369000UL
#define RF_FREQ_HZ 434600000.0f
#define RF_DEV_HZ 200.0f

#define F_INT (2 * XO_FREQ / 8)
#define FDIV_INTE ( (RF_FREQ_HZ / F_INT) - 1)
#define FDIV_FRAC ( (RF_FREQ_HZ - F_INT*(int)FDIV_INTE) * ((uint32_t)1 << 19)  ) / F_INT
#define FDEV ( ( ( (uint32_t)1 << 19) * 8 * RF_DEV_HZ)/ (2*XO_FREQ))

/* function prototypes */
void si4060_shutdown (void);
void si4060_wakeup (void);
void si4060_reset (void);
void si4060_power_up (void);
void si4060_nop (void);
uint8_t si4060_get_state(void);
void si4060_get_freq(void);
float si4060_get_temperature(void);

void si4060_start_tx (uint8_t channel);
void si4060_stop_tx (void);
void si4060_setup (uint8_t mod_type);
uint8_t si4060_get_property_8 (uint8_t group, uint8_t prop);
uint16_t si4060_part_info (void);

/* ===== command definitions ===== */
#define CMD_NOP	0x00
#define CMD_PART_INFO	0x01
#define CMD_POWER_UP	0x02
#define CMD_SET_PROPERTY	0x11
#define CMD_GET_PROPERTY	0x12
#define CMD_GPIO_PIN_CFG	0x13
#define CMD_START_TX	0x31
#define CMD_REQUEST_STATE	0x33
#define CMD_CHANGE_STATE	0x34
#define CMD_READ_CMD_BUF	0x44
#define CMD_GET_ADC_READING	0x14

/* ===== device states ===== */
#define STATE_NOCHANGE	0x00
#define STATE_SLEEP	0x01
#define STATE_SPI_ACTIVE	0x02
#define STATE_READY	0x03
#define STATE_TX_TUNE	0x05
#define STATE_TXA	0x07

/* ===== property group definitions ===== */
#define PROP_GLOBAL	0x00
#define PROP_INT_CTL	0x01
#define PROP_FRR_CTL	0x02
#define PROP_PREAMBLE	0x10
#define PROP_SYNC	0x11
#define PROP_PKT	0x12
#define PROP_MODEM	0x20
#define PROP_PA	0x22
#define PROP_SYNTH	0x23
#define PROP_FREQ_CONTROL	0x40

/* ===== property definitions ===== */
/* global properties */
#define GLOBAL_XO_TUNE	0x00
#define GLOBAL_CONFIG	0x03
/* preamble properties */
#define PREAMBLE_TX_LENGTH	0x00
/* sync properties */
#define SYNC_CONFIG	0x11
/* modem properties */
#define MODEM_MOD_TYPE	0x00
#define MODEM_FREQ_DEV	0x0a
#define MODEM_FREQ_OFFSET	0x0d
#define MODEM_CLKGEN_BAND	0x51
/* PA properties */
#define PA_MODE		0x00
#define PA_PWR_LVL	0x01
#define PA_BIAS_CLKDUTY	0x02
/* synthesizer properties */
#define SYNTH_PFDCP_CPFF	0x00
#define SYNTH_PFDCP_CPINT	0x01
#define SYNTH_VCO_KV	0x02
#define SYNTH_LPFILT3	0x03
#define SYNTH_LPFILT2	0x04
#define SYNTH_LPFILT1	0x05
#define SYNTH_LPFILT0	0x06
#define SYNTH_VCO_KVCAL	0x07
/* frequency control properties */
/* INTE shall be decreased by 1, because FRAC shall be between 1 and 2 */
#define FREQ_CONTROL_INTE	0x00
/* FRAC shall be added to 2**19, to ensure MSB is set! */
#define FREQ_CONTROL_FRAC	0x01
#define FREQ_CONTROL_CHANNEL_STEP_SIZE	0x04
#define FREQ_CONTROL_W_SIZE	0x06

/* ===== command arguments ===== */
/* POWER_UP arguments */
/* byte 1 */
#define PATCH  (	0x01 << 7) /* set patch mode */
#define FUNC	0x01 /* power on device */
/* byte 2 */
#define TCXO	0x01/* select if TCXO  (1) or crystal  (0) is used */

/* GPIO_PIN_CFG arguments */
/* bytes 1 .. 6 */
#define PULL_CTL 	0x40 /* enable or disable pull-up resistor */
/* bytes 1 .. 4 */
#define GPIO_MODE_DONOTHING	0x00/* pin behaviour is not changed */
#define GPIO_MODE_TRISTATE	0x01/* input and output drivers are disabled */
#define GPIO_MODE_DRIVE0	0x02/* CMOS output "low" */
#define GPIO_MODE_DRIVE1	0x03/* CMOS output "high" */
#define GPIO_MODE_INPUT	0x04/* GPIO is input, for TXDATA etc, function is not configured here */
#define GPIO_MODE_32K_CLK	0x05/* outputs the 32kHz CLK when selected in CLK32_CLK_SEL */
#define GPIO_MODE_BOOT_CLK	0x06/* outputs boot clock when SPI_ACTIVE */
#define GPIO_MODE_DIV_CLK	0x07/* outputs divided xtal clk */
#define GPIO_MODE_CTS	0x08/* output, '1' when device is ready to accept new command */
#define GPIO_MODE_INV_CNT	0x09/* output, inverted CTS */
#define GPIO_MODE_CMD_OVERLAP	0x0a/* output, '1' if a command was issued while not ready */
#define GPIO_MODE_SDO	0x0b/* output, serial data out for SPI */
#define GPIO_MODE_POR	0x0c/* output, '0' while in POR state */
#define GPIO_MODE_CAL_WUT	0x0d/* output, '1' on expiration of wake up timer */
#define GPIO_MODE_WUT	0x0e/* wake up timer output */
#define GPIO_MODE_EN_PA	0x0f/* output, '1' when PA is enabled */
#define GPIO_MODE_TX_DATA_CLK	0x10/* data clock output, for TX direct sync mode */
#define GPIO_MODE_TX_DATA	0x11/* data output from TX FIFO, for debugging purposes */
#define GPIO_MODE_IN_SLEEP	0x12/* output, '0' when in sleep state */
#define GPIO_MODE_TX_STATE	0x13/* output, '1' when in TX state */
#define GPIO_MODE_TX_FIFO_EMPTY	0x14/* output, '1' when FIFO is empty */
#define GPIO_MODE_LOW_BATT	0x15/* output, '1' if low battery is detected */
/* byte 5 omitted - no IRQ support */
#define NIRQ_MODE_DONOTHING	0x00
/* byte 6 omitted - no SDO reconfiguration support */
#define SDO_MODE_DONOTHING	0x00
/* byte 7 */
#define DRV_STRENGTH_HIGH (	0x00 << 5)
#define DRV_STRENGTH_MED_HIGH (	0x01 << 5)
#define DRV_STRENGTH_MED_LOW (	0x02 << 5)
#define DRV_STRENGTH_LOW (	0x03 << 5)

/* START_TX arguments */
/* byte 2 */
#define START_TX_TXC_STATE_NOCHANGE (	0x00 << 4)
#define START_TX_TXC_STATE_SLEEP (	0x01 << 4)
#define START_TX_TXC_STATE_SPI_ACTIVE (	0x02 << 4)
#define START_TX_TXC_STATE_READY (	0x03 << 4)
#define START_TX_RETRANSMIT_0 (	0x00 << 2)/* send data that has been written to the TX FIFO */
#define START_TX_START_IMM (	0x00 << 0)/* start transmission immediately */

/* ===== property values ===== */
/* GLOBAL_CONFIG values */
#define GLOBAL_RESERVED (	0x01 << 6) /* shall be set to 1 */
#define POWER_MODE_LOW_POWER	0x00/* default */
#define POWER_MODE_HIGH_PERF	0x01
#define SEQUENCER_MODE_FAST (	0x00 << 5)/* default */
#define SEQUENCER_MODE_GUARANT (	0x01 << 5)
/* SYNC_CONFIG values */
#define SYNC_XMIT (	0x00 << 7)/* default */
#define SYNC_NO_XMIT (	0x01 << 7)
/* MODEM_MOD_TYPE values */
#define MOD_TYPE_CW	0x00
#define MOD_TYPE_OOK	0x01
#define MOD_TYPE_2FSK	0x02/* default */
#define MOD_TYPE_2GFSK	0x03
#define MOD_TYPE_4FSK	0x04
#define MOD_TYPE_4GFSK	0x05
#define MOD_SOURCE_PACKET (	0x00 << 3) /* default */
#define MOD_SOURCE_DIRECT (	0x01 << 3)
#define MOD_SOURCE_PSEUDO (	0x02 << 3)
#define MOD_GPIO_0 (	0x00 << 5) /* default */
#define MOD_GPIO_1 (	0x01 << 5)
#define MOD_GPIO_2 (	0x02 << 5)
#define MOD_GPIO_3 (	0x03 << 5)
#define MOD_DIRECT_MODE_SYNC (	0x00 << 7) /* default */
#define MOD_DIRECT_MODE_ASYNC (	0x01 << 7)
/* MODEM_CLKGEN_BAND values */
#define SY_SEL_0 (	0x00 << 3) /* low power */
#define SY_SEL_1 (	0x01 << 3) /* default */
#define FVCO_DIV_4	0x00 /* default */
#define FVCO_DIV_6	0x01
#define FVCO_DIV_8	0x02 /* for 70cm ISM band */
#define FVCO_DIV_12	0x03
#define FVCO_DIV_16	0x04
#define FVCO_DIV_24	0x05
#define FVCO_DIV_24_2	0x06
#define FVCO_DIV_24_3	0x07
/* PA_MODE values*/
/* PA_BIAS_CLKDUTY values */
#define PA_BIAS_CLKDUTY_SIN_25		(0x03 << 6) /* for si4060 */
#define PA_BIAS_CLKDUTY_DIFF_50		(0x00 << 6) /* for si4063 */

#endif
