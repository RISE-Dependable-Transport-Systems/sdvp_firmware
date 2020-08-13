/*
	Copyright 2016 - 2018 Benjamin Vedder	benjamin@vedder.se
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

#include "pos_mc.h"
#include "pos.h"
#include "conf_general.h"
#include "servo_pwm.h"
#include <string.h>
#include <math.h>

// Private variables
static mc_values m_mc_val;

void pos_mc_init(void) {
	memset(&m_mc_val, 0, sizeof(m_mc_val));
}

void pos_mc_values_cb(mc_values *val) {
	m_mc_val = *val;

	static float last_tacho = 0;
	static bool tacho_read = false;
	float tacho = m_mc_val.tachometer;
	float rpm = m_mc_val.rpm;

	// Reset tacho the first time.
	if (!tacho_read) {
		tacho_read = true;
		last_tacho = tacho;
	}

	float distance = (tacho - last_tacho) * main_config.car.gear_ratio
			* (2.0 / main_config.car.motor_poles) * (1.0 / 6.0)
			* main_config.car.wheel_diam * M_PI;
	last_tacho = tacho;

	float angle_diff = 0.0;
	float turn_rad_rear = 0.0;

	float steering_angle = (servo_pwm_get(0) // TODO: generalize
			- main_config.car.steering_center)
			* ((2.0 * main_config.car.steering_max_angle_rad)
					/ main_config.car.steering_range);

	if (fabsf(steering_angle) >= 1e-6) {
		turn_rad_rear = main_config.car.axis_distance / tanf(steering_angle);
		float turn_rad_front = sqrtf(
				main_config.car.axis_distance * main_config.car.axis_distance
				+ turn_rad_rear * turn_rad_rear);

		if (turn_rad_rear < 0) {
			turn_rad_front = -turn_rad_front;
		}

		angle_diff = (distance * 2.0) / (turn_rad_rear + turn_rad_front);
	}

	float speed = rpm * main_config.car.gear_ratio
			* (2.0 / main_config.car.motor_poles) * (1.0 / 60.0)
			* main_config.car.wheel_diam * M_PI;

	pos_correction_mc(distance, turn_rad_rear, angle_diff, speed);
}

void pos_mc_get(mc_values *val) {
	*val = m_mc_val;
}
