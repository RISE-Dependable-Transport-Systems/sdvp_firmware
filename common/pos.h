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

#ifndef POS_H_
#define POS_H_

#include "datatypes.h"

// Functions
void pos_init(void);
void pos_get(POS_STATE *p);
float pos_get_yaw(void);
float pos_get_speed(void);
float pos_get_gnss_speed(void);
void pos_set_xya(float x, float y, float angle);
void pos_set_yaw_offset(float angle);
void pos_correction_imu(const float roll, const float pitch, const float yaw, const float yaw_mag, const float gyro[3], const float quaternions[4], const float dt);
void pos_correction_gnss(const float gnss_px, const float gnss_py, const float gnss_pz, const int32_t gnss_ms, const int fix_type);
void pos_correction_mc(float distance, float turn_rad_rear, float angle_diff, float speed);
void pos_set_correction_imu_hook(void (pos_correction_imu_hook)(POS_STATE *pos, float dt));
void pos_set_correction_imu_post_hook(void (pos_correction_imu_post_hook)(float dt));
void pos_set_correction_gnss_hook(void (pos_correction_gnss_hook)(POS_STATE *pos, float dt));

#endif /* POS_H_ */
