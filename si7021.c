/*
 * si7021.c
 *
 *  Created on: 23/10/2019
 *      Author: andru
 *
 *      SI7021 - nRF24LE1 relative humidity and temperature driver
 *
 */

#define SI7021_ADDR 	0x40 // device address

// SI7021 commands
#define MEASURE_RH_HM	0xE9
#define MEASURE_RH_NHM	0xF5
#define MEASURE_T_HM	0xE3
#define MEASURE_T_NHM	0xF3
#define READ_T_PREV_RH	0xE0
#define RESET			0xE9
#define WRITE_RHT_USER	0xE6
#define READ_RHT_USER	0xE7
#define WRITE_HEATER	0x51
#define READ_HEATER		0x11
#define READ_EID_1BYTE	0xFA0F
#define READ_EID_2BYTE	0xFCC9
#define READ_FIRMWARE	0x84B8

// user register masks
#define USER_MASK_VDDS	(1 << 6)
#define USER_MASK_HTRE	(1 << 2)
#define USER_MASK_RES	0x81
#define HEATER_MASK		0x0F

#define SPWR	GPIO_PIN_ID_P0_2		// P0.2 - SI7021 power PIN
#define W2SCL	GPIO_PIN_ID_P0_4		// P0.4 - SI7021 2-wire SCL
#define W2SDA	GPIO_PIN_ID_P0_5		// P0.5 - SI7021 2-wire SDA

#include "delay.h"
#include "gpio.h"
#include "w2.h"

#include "si7021.h"
#include "main.h"

static uint8_t convtime;

si7021error_t si7021_init(uint8_t res) {
	uint8_t cmd, txbuf, rxbuf;

	gpio_pin_configure(SPWR,
			GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT |
			GPIO_PIN_CONFIG_OPTION_OUTPUT_VAL_SET |
			GPIO_PIN_CONFIG_OPTION_PIN_MODE_OUTPUT_BUFFER_NORMAL_DRIVE_STRENGTH
	);

#if EN_SLEEP
	dosleep(SI7021_TIME_PWRUP * 1000 / 30);
#else
	delay_ms(SI7021_TIME_PWRUP);
#endif

	w2_configure(W2_CONFIG_OPTION_ENABLE |
				 W2_CONFIG_OPTION_MODE_MASTER |
				 W2_CONFIG_OPTION_CLOCK_FREQ_100_KHZ |
				 W2_CONFIG_OPTION_ALL_INTERRUPTS_ENABLE,
				 0
				 );

	cmd = READ_RHT_USER;
	if (w2_master_write_to(SI7021_ADDR, &cmd, 1, &txbuf, 0) == W2_NACK_VAL)
		return SI7021_TIMEOUT;

	if (w2_master_cur_address_read(SI7021_ADDR, &rxbuf, 1) == W2_NACK_VAL)
		return SI7021_I2CERROR;

	if (rxbuf & USER_MASK_VDDS)
		return SI7021_VDDS;

	switch (res) {
	case SI7021_RES_RH12_T14:
		convtime = SI7021_TIME_RH12_T14;
		break;
	case SI7021_RES_RH8_T12:
		convtime = SI7021_TIME_RH8_T12;
		break;
	case SI7021_RES_RH10_T13:
		convtime = SI7021_TIME_RH10_T13;
		break;
	case SI7021_RES_RH11_T11:
		convtime = SI7021_TIME_RH11_T11;
		break;
	default:
		return SI7021_PARAMERR;
	}

	cmd = WRITE_RHT_USER;
	txbuf = rxbuf & ~USER_MASK_RES;
	txbuf |= res;

	if (w2_master_write_to(SI7021_ADDR, &cmd, 1, &txbuf, 1) == W2_NACK_VAL)
		return SI7021_I2CERROR;

	return SI7021_OK;
}

void si7021_stop(void) {
	w2_disable();
	gpio_pin_configure(SPWR,
			GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT |
			GPIO_PIN_CONFIG_OPTION_OUTPUT_VAL_CLEAR |
			GPIO_PIN_CONFIG_OPTION_PIN_MODE_OUTPUT_BUFFER_NORMAL_DRIVE_STRENGTH
	);
}

// if return SI7021_OK: relative humidity * 10 (%), temperature * 10 (C)
si7021error_t si7021_read(uint16_t *humidity, int16_t *temperature) {
	uint8_t cmd, txbuf=0, rxbuf[2];
	int32_t data;

	cmd = MEASURE_RH_NHM;
	if (w2_master_write_to(SI7021_ADDR, &cmd, 1, &txbuf, 0) == W2_NACK_VAL)
		return SI7021_TIMEOUT;

#if EN_SLEEP
	dosleep(convtime * 1000 / 30);
#else
	delay_ms(convtime);
#endif

	if (w2_master_cur_address_read(SI7021_ADDR, rxbuf, 2) == W2_NACK_VAL)
		return SI7021_I2CERROR;

	data = (uint32_t) ((rxbuf[0] << 8) | rxbuf[1]);
    data = ((data * 1250) >> 16) - 60;
    *humidity = (uint16_t) data;
    if (*humidity > 1000) *humidity = 1000;

	cmd = READ_T_PREV_RH;
	if (w2_master_write_to(SI7021_ADDR, &cmd, 1, &txbuf, 0) == W2_NACK_VAL)
		return SI7021_I2CERROR;

	if (w2_master_cur_address_read(SI7021_ADDR, rxbuf, 2) == W2_NACK_VAL)
		return SI7021_I2CERROR;

	data = (int32_t) ((rxbuf[0] << 8) | rxbuf[1]);
	data = ((data * 17572) >> 16) - 4685;
	*temperature = (int16_t) data / 10;

	return SI7021_OK;
}
