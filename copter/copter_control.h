/*
	Copyright 2017 Benjamin Vedder	benjamin@vedder.se

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

#ifndef COPTER_CONTROL_H_
#define COPTER_CONTROL_H_

#include "conf_general.h"
#include "ch.h"
#include "hal.h"

// Functions
void copter_control_init(void);
systime_t copter_control_time_since_input_update(void);
void copter_control_set_input(float throttle, float roll, float pitch, float yaw);
void copter_control_set_motor_override(int motor, float power);
bool copter_control_is_throttle_over_tres(void);
void copter_control_run_iteration(float dt);
void copter_control_pos_correction_gnss(POS_STATE *pos, float dt);
void copter_control_pos_correction_imu(POS_STATE *pos, float dt);

#endif /* COPTER_CONTROL_H_ */
