/*
 * packet.h
 *
 *  NRF24 radio packet definition
 *
 *  Created on: 22/10/2019
 *      Author: andru
 */

#ifndef PACKET_H_
#define PACKET_H_

// message type
typedef enum {
	MSG_INFO = 0,
	MSG_DATA,
	MSG_ERROR,
	MSG_CMD,
} msgtype_t;

// data type
typedef enum {
	VAL_ch,
	VAL_i16,
	VAL_i32,
	VAL_fl,
} msgvalue_t;

// command type
typedef enum {
	CMD_CFGREAD = 1,	// read configuration value
	CMD_CFGWRITE,		// write configuration value
	CMD_RESET,			// reset device
	CMD_MSGWAIT,		//remote device waiting messages
	CMD_SENSOREAD = 10,	// read sensor value
	CMD_ON = 20,		// ON
	CMD_OFF,			// OFF
	CMD_ONTM,			// ON timeout (S)
	CMD_OFFTM,			// OFF timeout (S)
	CMD_GETREG = 40,	// GET register value
	CMD_SETREG,			// SET register value
} command_t;

// device message send type
typedef enum {
	CMD_NONE,		// command none
	CMD_SEND,		// command send immediately
	CMD_WAIT,		// wait message from device & send command
} devicecmd_t;

// message length = 16 bytes for AES
#define MSGLEN		sizeof(MESSAGE_T)

// NRF receive/send message format
typedef struct MESSAGE MESSAGE_T;
struct MESSAGE {
    uint8_t deviceid;			// (in/out): remote device id
    uint8_t firmware;			// (in/out): remote firmware
    uint8_t addrnum;			// (out): remote device internal address number
    uint8_t address;			// (in/out): on remote device internal address
    msgtype_t msgtype;			// (in/out): message type
    uint8_t error;				// (in): remote error code
    msgvalue_t datatype;		// (in/out): type of data
    uint8_t datapower;			// (in/out): ^10 data power;
    union {						// (in): value depend of message type
	uint8_t c4[4];				// char value in c4[0]
	int16_t i16[2];				// int16 value in i16[0]
	int32_t	i32;				// int32 value
	float	f;					// float value
    } data;
    int16_t cmdparam;			// (in): command parameter
    command_t command;			// (in): command
    uint8_t crc;				// radio packet CRC
};

#endif /* PACKET_H_ */
