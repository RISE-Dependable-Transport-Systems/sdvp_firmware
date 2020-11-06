/*
	Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se
	          2020        Marvin Damschen	marvin.damschen@ri.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CONF_GENERAL_H_
#define CONF_GENERAL_H_

#include "datatypes.h"

#define TIMEOUT_MIN_RPM_BRAKE		200

#define VEHICLE_TYPE_ROVER			0
#define VEHICLE_TYPE_COPTER			1

#define VEHICLE_TYPE				VEHICLE_TYPE_COPTER

// Firmware version
#define FW_VERSION_MAJOR			20
#define FW_VERSION_MINOR			1

// General settings
#define ID_ALL						255
#define ID_CAR_CLIENT				254 // Packet for car client only
#ifndef VESC_ID
#define VESC_ID						ID_ALL // id, or ID_ALL for any VESC (not used in diff steering mode)
#endif

// Car parameters
#ifndef BOARD_YAW_ROT
#define BOARD_YAW_ROT				135.0
#endif

// Autopilot settings
#define AP_ROUTE_SIZE				1024

// Global variables
extern MAIN_CONFIG main_config;
extern int main_id;

// Functions
void conf_general_init(void);
void conf_general_get_default_main_config(MAIN_CONFIG *conf);
bool conf_general_store_main_config(MAIN_CONFIG *conf);

#endif /* CONF_GENERAL_H_ */
