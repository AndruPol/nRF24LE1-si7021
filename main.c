/*
 *  Created on: 20/09/2016
 *      Author: andru
 *
 *      nRF24LE1 remote sensors unit
 *      support SI7021, DS18B20, AES encryption
 *
 *		based on great nRF24LE1 SDK https://github.com/DeanCording/nRF24LE1_SDK
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "gpio.h"
#include "delay.h"
#include "memory.h"
#include "adc.h"
#include "rtc2.h"
#include "pwr_clk_mgmt.h"
#include "watchdog.h"
#include "interrupt.h"

#include "main.h"
#include "crc8.h"
#include "radio.h"
#if EN_SI7021
#include "si7021.h"
#endif

#define CMDWAIT			4000	// command wait time in ~20us intervals
#define WDGTIMEOUT		2		// watchdog timeout, sec
#define SLEEPERR		300		// sleep, S on config error

#if DEBUG
#define EN_UART			1	// use UART for debugging
#define UARTTXPIN		GPIO_PIN_ID_P0_3		// P0.3 - UART TX
#define UARTRXPIN		GPIO_PIN_ID_P0_4		// P0.4	- UART RX
#else
#define EN_UART			0	// use UART for debugging
#endif

//#define SPWR	GPIO_PIN_ID_P0_2		// P0.2 - SI7021 power PIN
//#define W2SCL	GPIO_PIN_ID_P0_4		// P0.4 - SI7021 2-wire SCL
//#define W2SDA	GPIO_PIN_ID_P0_5		// P0.5 - SI7021 2-wire SDA

#define NVM_START_ADDRESS	MEMORY_FLASH_NV_STD_END_START_ADDRESS
#define ENVM_START_ADDRESS	MEMORY_FLASH_NV_EXT_END_START_ADDRESS
#define ENVM_PAGE_NUM		MEMORY_FLASH_NV_EXT_END_FIRST_PAGE_NUM

#if EN_UART
#include "uart.h"
#endif

CONFIG_T config;
MESSAGE_T message;
static volatile uint8_t pwrcrit = 0;

interrupt_isr_pwr_fail() {
	if (pwr_clk_mgmt_is_vdd_below_bor_threshold()) {
		pwrcrit = 1;
	}
}

// halt
static void halt(void) {
	while (1) {
		delay_ms(500);
	}
}

static void uart_init(void) {
#if EN_UART
	// Setup UART pins
	gpio_pin_configure(GPIO_PIN_ID_FUNC_RXD,
		GPIO_PIN_CONFIG_OPTION_DIR_INPUT |
		GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_ON_NO_RESISTORS
		);

	gpio_pin_configure(GPIO_PIN_ID_FUNC_TXD,
		GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT |
		GPIO_PIN_CONFIG_OPTION_OUTPUT_VAL_SET |
		GPIO_PIN_CONFIG_OPTION_PIN_MODE_OUTPUT_BUFFER_NORMAL_DRIVE_STRENGTH
		);

	uart_configure_8_n_1_38400();
#endif
}

static void pof_init(void) {
#if 0
	pwr_clk_mgmt_pwr_failure_configure(
			PWR_CLK_MGMT_PWR_FAILURE_CONFIG_OPTION_POF_ENABLE ||
			PWR_CLK_MGMT_PWR_FAILURE_CONFIG_OPTION_POF_THRESHOLD_2_1V
	);
#endif
	POFCON = 0x80;

	interrupt_control_pwr_fail_enable();
	interrupt_control_global_enable();
}

static void rtc_init(void) {
	// RTC2 init on 1s interval
	pwr_clk_mgmt_clklf_configure(PWR_CLK_MGMT_CLKLF_CONFIG_OPTION_CLK_SRC_RCOSC32K);
	pwr_clk_mgmt_wait_until_clklf_is_ready();

	watchdog_setup();
	watchdog_set_wdsv_count(watchdog_calc_timeout_from_sec(WDGTIMEOUT));

	pwr_clk_mgmt_op_mode_configure(
		PWR_CLK_MGMT_OP_MODE_CONFIG_OPTION_RUN_WD_NORMALLY |
		PWR_CLK_MGMT_WAKEUP_CONFIG_OPTION_WAKEUP_ON_RTC2_TICK_ALWAYS
	);

	rtc2_configure(
		RTC2_CONFIG_OPTION_COMPARE_MODE_0_RESET_AT_IRQ,
		32767);			// 1s
}

// write NVM config to eNVM
static uint8_t write_config(void) {
	uint8_t ret = CRC8((uint8_t *) &config, sizeof(CONFIG_T) - 1);
	config.crcbyte = ret;
	if (memory_flash_erase_page(ENVM_PAGE_NUM) != MEMORY_FLASH_OK)
		return 0;
	if (memory_flash_write_bytes(ENVM_START_ADDRESS, sizeof(CONFIG_T), (uint8_t *) &config) != MEMORY_FLASH_OK)
		return 0;
	return 1;
}

// read NVM comfig
static void read_config(uint16_t addr) {
	uint16_t i;
	memory_movx_accesses_data_memory();
	for (i = 0; i < sizeof(CONFIG_T); i++) {
		*((uint8_t*) &config + i) = *((__xdata uint8_t*) addr + i);
	}
}

static void message_fill(uint8_t addr) {
	memset(&message, 0, MSGLEN);
	message.firmware = FIRMWARE;
	message.addrnum = ADDR_NUM;
	message.cmdparam = CMD_WAIT;
	message.deviceid = config.deviceid;
	message.address = addr;
}

static void send_config(uint8_t addr, uint16_t value) {
	message_fill(addr);
	message.msgtype = MSG_INFO;
	message.datatype = VAL_i32;
	message.datapower = 0;
	message.data.i32 = value;
	rfsend(&message);
}

static void send_config_err(uint8_t addr, uint8_t errcode) {
	message_fill(addr);
	message.msgtype = MSG_ERROR;
	message.error = errcode;
	message.datapower = 0;
	message.data.i32 = errcode;
	rfsend(&message);
}

#if EN_SLEEP
// register retension sleep 30.51uS * time <= 65535
void dosleep(uint16_t tm) {
	if (tm == 0)	return;

	rtc2_configure(
		RTC2_CONFIG_OPTION_COMPARE_MODE_0_RESET_AT_IRQ,
		tm);

	watchdog_set_wdsv_count(watchdog_calc_timeout_from_sec(WDGTIMEOUT));

	rtc2_run();
	pwr_clk_mgmt_enter_pwr_mode_register_ret();
	rtc2_stop();
#if DEBUG
	printf("sleep: time=%d\r\n", tm);
#endif
}
#endif


// main
void main(void) {
	// variable definition
	uint8_t ret, cmd;

#if EN_SI7021
	int16_t SITemp;
	uint16_t SIHum;
#endif

	pof_init();
	rtc_init();

	read_config(NVM_START_ADDRESS);
	ret = CRC8((uint8_t *) &config, sizeof(CONFIG_T)-1);
	if (config.crcbyte != ret) {
		// NVM config wrong stop work
		config.sleeptm = SLEEPERR;
		goto SLEEP;
	}

	cmd = config.version;
	read_config(ENVM_START_ADDRESS);
	ret = CRC8((uint8_t *) &config, sizeof(CONFIG_T)-1);
	if (config.crcbyte != ret || config.version != cmd) {
		read_config(NVM_START_ADDRESS);
		if (!write_config()) {
			// config write error stop work
			config.sleeptm = SLEEPERR;
			goto SLEEP;
		}
	}

	uart_init();

#if 0
	printf("\r\nid=%d\r\ncfgver=%d\r\nchannel=%d\r\n", config.deviceid, config.version, config.channel);
	printf("maxsend=%d\r\nsleep=%d\r\n", config.maxsend, config.sleeptm);
	printf("sndaddr=%02X%02X%02X%02X%02X, ", config.sndaddr[0],config.sndaddr[1],config.sndaddr[2],config.sndaddr[3],config.sndaddr[4]);
	printf("rcvaddr=%02X%02X%02X%02X%02X\r\n", config.rcvaddr[0],config.rcvaddr[1],config.rcvaddr[2],config.rcvaddr[3],config.deviceid);
#endif

	// main loop
	while (1) {

		if (pwrcrit)
			goto VBATLOW;

#if EN_SI7021
		ret = si7021_init(SI7021_RES_RH10_T13);
		if (ret != SI7021_OK) {
			message_fill(ADDR_SI7021_TEMP);
			message.msgtype = MSG_ERROR;
			message.error = ret;
			rfsend(&message);

			message_fill(ADDR_SI7021_HUM);
			message.msgtype = MSG_ERROR;
			message.error = ret;
			rfsend(&message);

			si7021_stop();
			goto VBATLOW;
		}

		ret = si7021_read(&SIHum, &SITemp);
		si7021_stop();

		message_fill(ADDR_SI7021_TEMP);
		message.datatype = 	VAL_i32;
		message.datapower = 1;
		if (ret == SI7021_OK) {
			message.msgtype = MSG_DATA;
			message.data.i32 = SITemp;
#if DEBUG
			printf("si7021 temp=%d\r\n", SITemp);
#endif
		} else {
			message.msgtype = MSG_ERROR;
			message.error = ret;
#if DEBUG
			printf("si7021 temp err=%d\r\n", ret);
#endif
		}
		rfsend(&message);

		message_fill(ADDR_SI7021_HUM);
		message.datatype = 	VAL_i32;
		message.datapower = 1;
		if (ret == SI7021_OK) {
			message.msgtype = MSG_DATA;
			message.data.i32 = SIHum;
#if DEBUG
			printf("si7021 hum=%d\r\n", SIHum);
#endif
		} else {
			message.msgtype = MSG_ERROR;
			message.error = ret;
#if DEBUG
			printf("si7021 hum err=%d\r\n", ret);
#endif
		}
		rfsend(&message);
#endif	//SI7021

VBATLOW:

#if EN_VBAT
		message_fill(ADDR_VBAT);
		if (pwrcrit) {
			message.msgtype = MSG_ERROR;
			message.error = ERR_VBATLOW;
			message.data.i32 = ERR_VBATLOW;
#if DEBUG
			printf("vbat low fired\r\n");
#endif
		} else {
			message.msgtype = MSG_DATA;
			message.datatype = 	VAL_i32;
			message.data.i32 = 211;
		}
		rfsend(&message);
#endif

WAITCMD:
		message_fill(0);
		message.msgtype = MSG_CMD;
		message.command = CMD_MSGWAIT;
		rfsend(&message);

		// wait command from smarthome gateway
		memset(&message, 0, MSGLEN);
		cmd = rfread(&message, CMDWAIT);

		if (cmd && message.deviceid == config.deviceid && message.msgtype == MSG_CMD) {
#if DEBUG
			printf("\r\ncommand: %d\r\n", message.command);
			printf("address: %d\r\n", message.address);
			printf("param: %d\r\n\r\n", (uint16_t) message.data.i32);
#endif
			// команда чтения/записи конфигурации
			if ((message.command == CMD_CFGREAD || message.command == CMD_CFGWRITE)) {
				switch (message.address) {
				case CFG_SLEEP:
					if (message.command == CMD_CFGWRITE) {
						if (message.data.i32 < 0 || message.data.i32 > 512) {
						    send_config_err(CFG_SLEEP, ERR_CMDPARAM);
						    break;
						}
						config.sleeptm = (uint16_t) message.data.i32;
						if (!write_config())	{
							send_config_err(CFG_SLEEP, ERR_CFGWRITE);
							break;
						}
					}
					send_config(CFG_SLEEP, config.sleeptm);
					break;
				default:
					break;
				}
			}
			goto WAITCMD;
		}

		rfdown();

SLEEP:

#if EN_SLEEP
		if (config.sleeptm > 0) {
			watchdog_set_wdsv_count(watchdog_calc_timeout_from_sec(config.sleeptm));
			pwr_clk_mgmt_enter_pwr_mode_memory_ret_tmr_on(); // 1mkA
		}
#else
   		if (config.sleeptm > 0) {
			watchdog_set_wdsv_count(watchdog_calc_timeout_from_sec(config.sleeptm+1));
   			delay_s(config.sleeptm);
   		}
#endif
	} // main loop
}
