/*
	Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

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

#include "conf_general.h"
#include <string.h>

// Global variables
MAIN_CONFIG main_config;
int main_id = 0;

void conf_general_init(void) {
	main_id = 0;
	conf_general_get_default_main_config(&main_config);
}

/**
 * Load the compiled default app_configuration.
 *
 * @param conf
 * A pointer to store the default configuration to.
 */
void conf_general_get_default_main_config(MAIN_CONFIG *conf) {
	// Default settings
	conf->mag_use = false;
	conf->mag_comp = true;
	conf->yaw_mag_gain = 1.2;

	conf->mag_cal_cx = 0.0;
	conf->mag_cal_cy = 0.0;
	conf->mag_cal_cz = 0.0;
	conf->mag_cal_xx = 1.0;
	conf->mag_cal_xy = 0.0;
	conf->mag_cal_xz = 0.0;
	conf->mag_cal_yx = 0.0;
	conf->mag_cal_yy = 1.0;
	conf->mag_cal_yz = 0.0;
	conf->mag_cal_zx = 0.0;
	conf->mag_cal_zy = 0.0;
	conf->mag_cal_zz = 1.0;

	conf->gps_ant_x = 0.0;
	conf->gps_ant_y = 0.0;
	conf->gps_comp = true;
	conf->gps_req_rtk = true;
	conf->gps_use_rtcm_base_as_enu_ref = true;
	conf->gps_corr_gain_stat = 0.05;
	conf->gps_corr_gain_dyn = 0.05;
	conf->gps_corr_gain_yaw = 1.0;
	conf->gps_send_nmea = true;
	conf->gps_use_ubx_info = true;
	conf->gps_ubx_max_acc = 0.12;

	conf->uwb_max_corr = 0.1;

	conf->ap_repeat_routes = true;
	conf->ap_base_rad = 0.8;
	conf->ap_rad_time_ahead = 0.8;
	conf->ap_mode_time = false;
	conf->ap_max_speed = 30.0 / 3.6;
	conf->ap_time_add_repeat_ms = 60 * 1000;

	conf->log_rate_hz = 50;
	conf->log_en = false;
	strcpy(conf->log_name, "New Log");
	conf->log_mode_ext = 0;
	conf->log_uart_baud = 115200;

	// Default car settings
	conf->car.yaw_use_odometry = false;
	conf->car.yaw_imu_gain = 0.5;
	conf->car.disable_motor = false;
	conf->car.simulate_motor = false;
	conf->car.clamp_imu_yaw_stationary = true;
	conf->car.use_uwb_pos = false;

	conf->car.gear_ratio = (1.0 / 3.0) * (21.0 / 37.0);
	conf->car.wheel_diam = 0.11;
	conf->car.motor_poles = 4.0;
	conf->car.steering_max_angle_rad = 0.42041;
	conf->car.steering_center = 0.5;
	conf->car.steering_range = 0.58;
	conf->car.steering_ramp_time = 0.0;
	conf->car.axis_distance = 0.475;

	// Default multirotor settings
	conf->mr.vel_decay_e = 0.8;
	conf->mr.vel_decay_l = 0.02;
	conf->mr.vel_max = 80.0 / 3.6;

	conf->mr.map_min_x = -500.0;
	conf->mr.map_max_x = 500.0;
	conf->mr.map_min_y = -500.0;
	conf->mr.map_max_y = 500.0;

	conf->mr.vel_gain_p = 0.1;
	conf->mr.vel_gain_i = 0.0;
	conf->mr.vel_gain_d = 0.2;

	conf->mr.tilt_gain_p = 0.2;
	conf->mr.tilt_gain_i = 0.0;
	conf->mr.tilt_gain_d = 0.05;

	conf->mr.max_corr_error = 0.5;
	conf->mr.max_tilt_error = 6.0;

	conf->mr.ctrl_gain_roll_p = 0.8;
	conf->mr.ctrl_gain_roll_i = 1.0;
	conf->mr.ctrl_gain_roll_dp = 0.3;
	conf->mr.ctrl_gain_roll_de = 0.2;

	conf->mr.ctrl_gain_pitch_p = 0.8;
	conf->mr.ctrl_gain_pitch_i = 1.0;
	conf->mr.ctrl_gain_pitch_dp = 0.3;
	conf->mr.ctrl_gain_pitch_de = 0.2;

	conf->mr.ctrl_gain_yaw_p = 3.0;
	conf->mr.ctrl_gain_yaw_i = 0.2;
	conf->mr.ctrl_gain_yaw_dp = 0.4;
	conf->mr.ctrl_gain_yaw_de = 0.2;

	conf->mr.ctrl_gain_pos_p = 0.8;
	conf->mr.ctrl_gain_pos_i = 0.09;
	conf->mr.ctrl_gain_pos_d = 0.6;

	conf->mr.ctrl_gain_alt_p = 0.1;
	conf->mr.ctrl_gain_alt_i = 0.1;
	conf->mr.ctrl_gain_alt_d = 0.14;

	conf->mr.js_gain_tilt = 1.0;
	conf->mr.js_gain_yaw = 0.6;
	conf->mr.js_mode_rate = false;

	conf->mr.motor_fl_f = 0;
	conf->mr.motor_bl_l = 1;
	conf->mr.motor_fr_r = 2;
	conf->mr.motor_br_b = 3;
	conf->mr.motors_x = true;
	conf->mr.motors_cw = true;
	conf->mr.motor_pwm_min_us = 1200;
	conf->mr.motor_pwm_max_us = 2000;

	// Only the SLU testbot for now
#if HAS_DIFF_STEERING
	conf->car.gear_ratio = 1.0;
	conf->car.axis_distance = 0.5;
	conf->car.wheel_diam = 0.3;
	conf->car.motor_poles = 22.0;
	conf->gps_ant_x = 0.5;
#endif

#ifdef IS_DRANGEN
	conf->car.steering_center = 0.5;
	conf->car.steering_range = -0.9;
	conf->car.axis_distance = 1.0;
	conf->car.steering_max_angle_rad = atanf(conf->car.axis_distance / 1.5);
#endif

#ifdef IS_MACTRAC
	conf->car.steering_center = 0.5;
	conf->car.steering_range = -1.0;
	conf->car.axis_distance = 1.7;
	conf->car.steering_max_angle_rad = atanf(conf->car.axis_distance / 1.5);
	conf->gps_corr_gain_yaw = 2.0;
	conf->ap_base_rad = 4.0;
	conf->ap_repeat_routes = false;

	conf->gps_ant_x = 1.25;
	conf->gps_ant_y = -0.3;

	conf->log_en = true;
	conf->log_mode_ext = LOG_EXT_ETHERNET;
	conf->log_rate_hz = 10;
#endif
}
