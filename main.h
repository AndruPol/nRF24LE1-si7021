/*
 * main.h
 *
 *  Created on: 20/09/2016
 *      Author: andru
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>

#define FIRMWARE		101		// firmware revision
#define BATTERY			1		// battery power
#define DEBUG			0		// debug output via UART
//#define printf		printf_tiny

#define EN_SI7021		1		// SI7021 sensor enable
#define EN_RF			1		// radio enable
#define EN_VBAT			1		// battery voltage
#define EN_SLEEP		1		// power save mode enable
#define EN_AES			1		// AES encryption enable

#include "packet.h"

// NVM configuration data for nRF24LE1
typedef struct CONFIG CONFIG_T;
struct CONFIG {
	uint8_t version;	// configuration version
	uint8_t deviceid;	// device ID
	uint8_t sndaddr[5];	// gateway address to send
	uint8_t rcvaddr[5];	// device address to receive command
	uint8_t channel;	// radio channel: 0-199
	uint8_t maxsend;	// max message send retries
	uint16_t sleeptm;	// wakeup by watchdog timer, S (LSB first)
#if EN_AES
	uint8_t aeskey[16];	// aes encryption key
#endif
	uint8_t crcbyte;	// CRC8 sizeof(config) - 1
};

// internal address map
#define ADDR_NUM	4
typedef enum {
#if EN_SI7021
	ADDR_SI7021_TEMP = 0,	// SI7021 temperature
	ADDR_SI7021_HUM = 1,	// SI7021 humidity
#endif
#if EN_VBAT
	ADDR_VBAT = 2,			// VBAT state
#endif
	CFG_SLEEP = 3,			// uint16_t sleeptm;
} address_t;

// error codes
typedef enum {
	ERR_VBATLOW = 1,
	ERR_CFGWRITE,
	ERR_CMDPARAM,
} msgerr_t;

extern CONFIG_T config;

#if EN_SLEEP
void dosleep(uint16_t tm);
#endif

#endif /* MAIN_H_ */
