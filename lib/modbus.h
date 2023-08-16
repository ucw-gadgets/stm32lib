/*
 *	Generic MODBUS Library for STM32
 *
 *	(c) 2019--2023 Martin Mareš <mj@ucw.cz>
 */

#ifndef _MODBUS_H
#define _MODBUS_H

void modbus_init(void);
void modbus_loop(void);

// If a call-back wants to signal a slave error in the reply
void modbus_slave_error(void);

bool modbus_is_idle(void);

// Callbacks

bool modbus_check_discrete_input(u16 addr);
bool modbus_get_discrete_input(u16 addr);

bool modbus_check_coil(u16 addr);
bool modbus_get_coil(u16 addr);
void modbus_set_coil(u16 addr, bool value);

bool modbus_check_input_register(u16 addr);
u16 modbus_get_input_register(u16 addr);

bool modbus_check_holding_register(u16 addr);
u16 modbus_get_holding_register(u16 addr);
void modbus_set_holding_register(u16 addr, u16 value);

void modbus_ready_hook(void);
void modbus_frame_start_hook(void);

enum modbus_id_object {
	MODBUS_ID_VENDOR_NAME,		// first three must be always defined
	MODBUS_ID_PRODUCT_CODE,
	MODBUS_ID_MAJOR_MINOR_REVISION,
	MODBUS_ID_VENDOR_URL,		// the rest may be NULL
	MODBUS_ID_PRODUCT_NAME,
	MODBUS_ID_USER_APP_NAME,
	MODBUS_ID_MAX,
};

extern const char * const modbus_id_strings[MODBUS_ID_MAX];

#endif
