/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef MAIN_COMM_BLE_H_
#define MAIN_COMM_BLE_H_

#include <stdint.h>
#include <stdbool.h>

#include "esp_gap_ble_api.h"

void comm_ble_init(void);
bool comm_ble_is_connected();
int comm_ble_mtu_now(void);
void comm_ble_send_packet(unsigned char *data, unsigned int len);

// GAP event hook — allows other modules (e.g. ANT BMS GATTC) to receive GAP events
void comm_ble_set_gap_hook(void (*hook)(esp_gap_ble_cb_event_t, esp_ble_gap_cb_param_t *));

#endif /* MAIN_COMM_BLE_H_ */
