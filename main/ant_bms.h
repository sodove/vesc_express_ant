/*
	ANT BMS BLE bridge for VESC Express
	Connects to ANT BMS via BLE GATT Client, parses battery data,
	and populates VESC bms_values for CAN forwarding.
*/

#ifndef ANT_BMS_H_
#define ANT_BMS_H_

#include <stdbool.h>

void ant_bms_init(void);
bool ant_bms_is_connected(void);

#endif /* ANT_BMS_H_ */
