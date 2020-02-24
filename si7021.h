/*
 * si7021.h - Si7021-A20 driver
 *
 *  Created on: 23/10/2019
 *      Author: andru
 */

#ifndef SI7021_H_
#define SI7021_H_

// resolutions masks
#define SI7021_RES_RH12_T14		0x00 // 12bit RH, 14bit temperature
#define SI7021_RES_RH8_T12		0x01
#define SI7021_RES_RH10_T13		0x80
#define SI7021_RES_RH11_T11		0x81

// conversation times
#define SI7021_TIME_RH12_T14	23	// conversation time, mS
#define SI7021_TIME_RH8_T12		7	// conversation time, mS
#define SI7021_TIME_RH10_T13	11	// conversation time, mS
#define SI7021_TIME_RH11_T11	10	// conversation time, mS

#define SI7021_TIME_PWRUP		25	// power up time (datasheet says max. 25ms)
#define SI7021_TIME_RESET		15	// software reset time (datasheet says 15ms)

typedef enum {
	SI7021_OK,
	SI7021_TIMEOUT,
	SI7021_I2CERROR,
	SI7021_VDDS,
	SI7021_PARAMERR,
} si7021error_t;

si7021error_t si7021_init(uint8_t res);
void si7021_stop(void);
si7021error_t si7021_read(uint16_t *humidity, int16_t *temperature);

#endif /* SI7021_H_ */
