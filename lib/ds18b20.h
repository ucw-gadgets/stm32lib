/*
 *	Interface to DS18B20 Temperature Sensors
 *
 *	(c) 2019 Martin Mareš <mj@ucw.cz>
 */

#ifndef _DS18B20_H
#define _DS18B20_H

struct ds_sensor {
	byte address[8];	// All zeroes if sensor does not exist.
				// Otherwise, address[0] is guaranteed to be non-zero.
	int current_temp;	// Temperature in m°C or DS_TEMP_UNKNOWN
};

extern struct ds_sensor ds_sensors[DS_NUM_SENSORS];

#define DS_TEMP_UNKNOWN 0x7fffffff

void ds_init(void);
void ds_step(void);

#endif
