#include "commands.h"
#include "packet.h"
#include "buffer.h"
#include "conf_general.h"
#include "datatypes.h"
#include "utils.h"
#include "terminal.h"
#include "servo_pwm.h"
#include "bldc_interface.h"
#include "comm_can.h"
#include "pos.h"
#include "timeout.h"
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>

// Private variables
static uint8_t m_send_buffer[PACKET_MAX_PL_LEN];
static void(*m_send_func)(unsigned char *data, unsigned int len) = 0;

void commands_init(void) {

}

/**
 * Provide a function to use the next time there are packets to be sent.
 *
 * @param func
 * A pointer to the packet sending function.
 */
void commands_set_send_func(void(*func)(unsigned char *data, unsigned int len)) {
	m_send_func = func;
}

/**
 * Send a packet using the set send function.
 *
 * @param data
 * The packet data.
 *
 * @param len
 * The data length.
 */
void commands_send_packet(unsigned char *data, unsigned int len) {
	if (m_send_func) {
		m_send_func(data, len);
	}
}

/**
 * Process a received buffer with commands and data.
 *
 * @param data
 * The buffer to process.
 *
 * @param len
 * The length of the buffer.
 *
 * @param func
 * A pointer to the packet sending function.
 */
void commands_process_packet(unsigned char *data, unsigned int len,
		void (*func)(unsigned char *data, unsigned int len)) {

	if (!len) {
		return;
	}

	uint8_t receiver_id = data[0];
	CMD_PACKET packet_id = data[1];
	data+=2;
	len-=2;

	if (receiver_id == main_id || receiver_id == ID_ALL || receiver_id == ID_CAR_CLIENT) {
		int id_ret = main_id;

		if (receiver_id == ID_CAR_CLIENT) {
			id_ret = ID_CAR_CLIENT;
		}

		switch (packet_id) {
		case CMD_HEARTBEAT: {
			timeout_reset();
		} break;

		case CMD_TERMINAL_CMD: {
			commands_set_send_func(func);

			data[len] = '\0';
			terminal_process_string((char*)data);
		} break;
		case CMD_GET_STATE: {
			POS_STATE pos;
			float accel[3];
			float gyro[3];
			float mag[3];

			commands_set_send_func(func);

			pos_get_pos(&pos);
			pos_get_imu(accel, gyro, mag);

			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret; // 1
			m_send_buffer[send_index++] = CMD_GET_STATE; // 2
			m_send_buffer[send_index++] = FW_VERSION_MAJOR; // 3
			m_send_buffer[send_index++] = FW_VERSION_MINOR; // 4
			buffer_append_float32(m_send_buffer, pos.roll, 1e6, &send_index); // 8
			buffer_append_float32(m_send_buffer, pos.pitch, 1e6, &send_index); // 12
			buffer_append_float32(m_send_buffer, pos.yaw, 1e6, &send_index); // 16
			buffer_append_float32(m_send_buffer, accel[0], 1e6, &send_index); // 20
			buffer_append_float32(m_send_buffer, accel[1], 1e6, &send_index); // 24
			buffer_append_float32(m_send_buffer, accel[2], 1e6, &send_index); // 28
			buffer_append_float32(m_send_buffer, gyro[0], 1e6, &send_index); // 32
			buffer_append_float32(m_send_buffer, gyro[1], 1e6, &send_index); // 36
			buffer_append_float32(m_send_buffer, gyro[2], 1e6, &send_index); // 40
			buffer_append_float32(m_send_buffer, mag[0], 1e6, &send_index); // 44
			buffer_append_float32(m_send_buffer, mag[1], 1e6, &send_index); // 48
			buffer_append_float32(m_send_buffer, mag[2], 1e6, &send_index); // 52
			buffer_append_float32(m_send_buffer, pos.px, 1e4, &send_index); // 56
			buffer_append_float32(m_send_buffer, pos.py, 1e4, &send_index); // 60
			buffer_append_float32(m_send_buffer, 0.0, 1e6, &send_index); // 64
			buffer_append_float32(m_send_buffer, 0.0, 1e6, &send_index); // 68
			buffer_append_float32(m_send_buffer, 0.0, 1e6, &send_index); // 72
			m_send_buffer[send_index++] = 0; // 73
			buffer_append_float32(m_send_buffer, pos.px_gps, 1e4, &send_index); // 77
			buffer_append_float32(m_send_buffer, pos.px_gps, 1e4, &send_index); // 81
			buffer_append_float32(m_send_buffer, 0.0, 1e4, &send_index); // 85
			buffer_append_float32(m_send_buffer, 0.0, 1e4, &send_index); // 89
			buffer_append_float32(m_send_buffer, 0.0, 1e6, &send_index); // 93
			buffer_append_int32(m_send_buffer, 0, &send_index); // 97
			buffer_append_int16(m_send_buffer, 0, &send_index); // 99
			buffer_append_float32(m_send_buffer, pos.px, 1e4, &send_index); // 103
			buffer_append_float32(m_send_buffer, pos.py, 1e4, &send_index); // 107

			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_SET_MAIN_CONFIG: {
			commands_set_send_func(func);

			int32_t ind = 0;
			main_config.mag_use = data[ind++];
			main_config.mag_comp = data[ind++];
			main_config.yaw_mag_gain = buffer_get_float32_auto(data, &ind);

			main_config.mag_cal_cx = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_cy = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_cz = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_xx = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_xy = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_xz = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_yx = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_yy = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_yz = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_zx = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_zy = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_zz = buffer_get_float32_auto(data, &ind);

			main_config.gps_ant_x = buffer_get_float32_auto(data, &ind);
			main_config.gps_ant_y = buffer_get_float32_auto(data, &ind);
			main_config.gps_comp = data[ind++];
			main_config.gps_req_rtk = data[ind++];
			main_config.gps_use_rtcm_base_as_enu_ref = data[ind++];
			main_config.gps_corr_gain_stat = buffer_get_float32_auto(data, &ind);
			main_config.gps_corr_gain_dyn = buffer_get_float32_auto(data, &ind);
			main_config.gps_corr_gain_yaw = buffer_get_float32_auto(data, &ind);
			main_config.gps_send_nmea = data[ind++];
			main_config.gps_use_ubx_info = data[ind++];
			main_config.gps_ubx_max_acc = buffer_get_float32_auto(data, &ind);

			main_config.uwb_max_corr = buffer_get_float32_auto(data, &ind);

			main_config.ap_repeat_routes = data[ind++];
			main_config.ap_base_rad = buffer_get_float32_auto(data, &ind);
			main_config.ap_rad_time_ahead = buffer_get_float32_auto(data, &ind);
			main_config.ap_mode_time = data[ind++];
			main_config.ap_max_speed = buffer_get_float32_auto(data, &ind);
			main_config.ap_time_add_repeat_ms = buffer_get_int32(data, &ind);

			main_config.log_rate_hz = buffer_get_int16(data, &ind);
			main_config.log_en = data[ind++];
			strcpy(main_config.log_name, (const char*)(data + ind));
			ind += strlen(main_config.log_name) + 1;
			main_config.log_mode_ext = data[ind++];
			main_config.log_uart_baud = buffer_get_uint32(data, &ind);

//			log_set_rate(main_config.log_rate_hz);
//			log_set_enabled(main_config.log_en);
//			log_set_name(main_config.log_name);
//			log_set_ext(main_config.log_mode_ext, main_config.log_uart_baud);

			// Car settings
			main_config.car.yaw_use_odometry = data[ind++];
			main_config.car.yaw_imu_gain = buffer_get_float32_auto(data, &ind);
			main_config.car.disable_motor = data[ind++];
			main_config.car.simulate_motor = data[ind++];
			main_config.car.clamp_imu_yaw_stationary = data[ind++];
			main_config.car.use_uwb_pos = data[ind++];

			main_config.car.gear_ratio = buffer_get_float32_auto(data, &ind);
			main_config.car.wheel_diam = buffer_get_float32_auto(data, &ind);
			main_config.car.motor_poles = buffer_get_float32_auto(data, &ind);
			main_config.car.steering_max_angle_rad = buffer_get_float32_auto(data, &ind);
			main_config.car.steering_center = buffer_get_float32_auto(data, &ind);
			main_config.car.steering_range = buffer_get_float32_auto(data, &ind);
			main_config.car.steering_ramp_time = buffer_get_float32_auto(data, &ind);
			main_config.car.axis_distance = buffer_get_float32_auto(data, &ind);

			// Multirotor settings
			main_config.mr.vel_decay_e = buffer_get_float32_auto(data, &ind);
			main_config.mr.vel_decay_l = buffer_get_float32_auto(data, &ind);
			main_config.mr.vel_max = buffer_get_float32_auto(data, &ind);
			main_config.mr.map_min_x = buffer_get_float32_auto(data, &ind);
			main_config.mr.map_max_x = buffer_get_float32_auto(data, &ind);
			main_config.mr.map_min_y = buffer_get_float32_auto(data, &ind);
			main_config.mr.map_max_y = buffer_get_float32_auto(data, &ind);

			main_config.mr.vel_gain_p = buffer_get_float32_auto(data, &ind);
			main_config.mr.vel_gain_i = buffer_get_float32_auto(data, &ind);
			main_config.mr.vel_gain_d = buffer_get_float32_auto(data, &ind);

			main_config.mr.tilt_gain_p = buffer_get_float32_auto(data, &ind);
			main_config.mr.tilt_gain_i = buffer_get_float32_auto(data, &ind);
			main_config.mr.tilt_gain_d = buffer_get_float32_auto(data, &ind);

			main_config.mr.max_corr_error = buffer_get_float32_auto(data, &ind);
			main_config.mr.max_tilt_error = buffer_get_float32_auto(data, &ind);

			main_config.mr.ctrl_gain_roll_p = buffer_get_float32_auto(data, &ind);
			main_config.mr.ctrl_gain_roll_i = buffer_get_float32_auto(data, &ind);
			main_config.mr.ctrl_gain_roll_dp = buffer_get_float32_auto(data, &ind);
			main_config.mr.ctrl_gain_roll_de = buffer_get_float32_auto(data, &ind);

			main_config.mr.ctrl_gain_pitch_p = buffer_get_float32_auto(data, &ind);
			main_config.mr.ctrl_gain_pitch_i = buffer_get_float32_auto(data, &ind);
			main_config.mr.ctrl_gain_pitch_dp = buffer_get_float32_auto(data, &ind);
			main_config.mr.ctrl_gain_pitch_de = buffer_get_float32_auto(data, &ind);

			main_config.mr.ctrl_gain_yaw_p = buffer_get_float32_auto(data, &ind);
			main_config.mr.ctrl_gain_yaw_i = buffer_get_float32_auto(data, &ind);
			main_config.mr.ctrl_gain_yaw_dp = buffer_get_float32_auto(data, &ind);
			main_config.mr.ctrl_gain_yaw_de = buffer_get_float32_auto(data, &ind);

			main_config.mr.ctrl_gain_pos_p = buffer_get_float32_auto(data, &ind);
			main_config.mr.ctrl_gain_pos_i = buffer_get_float32_auto(data, &ind);
			main_config.mr.ctrl_gain_pos_d = buffer_get_float32_auto(data, &ind);

			main_config.mr.ctrl_gain_alt_p = buffer_get_float32_auto(data, &ind);
			main_config.mr.ctrl_gain_alt_i = buffer_get_float32_auto(data, &ind);
			main_config.mr.ctrl_gain_alt_d = buffer_get_float32_auto(data, &ind);

			main_config.mr.js_gain_tilt = buffer_get_float32_auto(data, &ind);
			main_config.mr.js_gain_yaw = buffer_get_float32_auto(data, &ind);
			main_config.mr.js_mode_rate = data[ind++];

			main_config.mr.motor_fl_f = data[ind++];
			main_config.mr.motor_bl_l = data[ind++];
			main_config.mr.motor_fr_r = data[ind++];
			main_config.mr.motor_br_b = data[ind++];
			main_config.mr.motors_x = data[ind++];
			main_config.mr.motors_cw = data[ind++];
			main_config.mr.motor_pwm_min_us = buffer_get_uint16(data, &ind);
			main_config.mr.motor_pwm_max_us = buffer_get_uint16(data, &ind);

//			conf_general_store_main_config(&main_config);

			// Doing this while driving will get wrong as there is so much accelerometer noise then.
			//pos_reset_attitude();

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_GET_MAIN_CONFIG:
		case CMD_GET_MAIN_CONFIG_DEFAULT: {
			commands_set_send_func(func);

			MAIN_CONFIG main_cfg_tmp;

			if (packet_id == CMD_GET_MAIN_CONFIG) {
				main_cfg_tmp = main_config;
			} else {
				conf_general_get_default_main_config(&main_cfg_tmp);
			}

			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;

			m_send_buffer[send_index++] = main_cfg_tmp.mag_use;
			m_send_buffer[send_index++] = main_cfg_tmp.mag_comp;
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.yaw_mag_gain, &send_index);

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_cx, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_cy, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_cz, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_xx, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_xy, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_xz, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_yx, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_yy, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_yz, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_zx, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_zy, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_zz, &send_index);

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.gps_ant_x, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.gps_ant_y, &send_index);
			m_send_buffer[send_index++] = main_cfg_tmp.gps_comp;
			m_send_buffer[send_index++] = main_cfg_tmp.gps_req_rtk;
			m_send_buffer[send_index++] = main_cfg_tmp.gps_use_rtcm_base_as_enu_ref;
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.gps_corr_gain_stat, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.gps_corr_gain_dyn, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.gps_corr_gain_yaw, &send_index);
			m_send_buffer[send_index++] = main_cfg_tmp.gps_send_nmea;
			m_send_buffer[send_index++] = main_cfg_tmp.gps_use_ubx_info;
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.gps_ubx_max_acc, &send_index);

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.uwb_max_corr, &send_index);

			m_send_buffer[send_index++] = main_cfg_tmp.ap_repeat_routes;
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.ap_base_rad, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.ap_rad_time_ahead, &send_index);
			m_send_buffer[send_index++] = main_cfg_tmp.ap_mode_time;
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.ap_max_speed, &send_index);
			buffer_append_int32(m_send_buffer, main_cfg_tmp.ap_time_add_repeat_ms, &send_index);

			buffer_append_int16(m_send_buffer, main_cfg_tmp.log_rate_hz, &send_index);
			m_send_buffer[send_index++] = main_cfg_tmp.log_en;
			strcpy((char*)(m_send_buffer + send_index), main_cfg_tmp.log_name);
			send_index += strlen(main_config.log_name) + 1;
			m_send_buffer[send_index++] = main_cfg_tmp.log_mode_ext;
			buffer_append_uint32(m_send_buffer, main_cfg_tmp.log_uart_baud, &send_index);

			// Car settings
			m_send_buffer[send_index++] = main_cfg_tmp.car.yaw_use_odometry;
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.car.yaw_imu_gain, &send_index);
			m_send_buffer[send_index++] = main_cfg_tmp.car.disable_motor;
			m_send_buffer[send_index++] = main_cfg_tmp.car.simulate_motor;
			m_send_buffer[send_index++] = main_cfg_tmp.car.clamp_imu_yaw_stationary;
			m_send_buffer[send_index++] = main_cfg_tmp.car.use_uwb_pos;

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.car.gear_ratio, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.car.wheel_diam, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.car.motor_poles, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.car.steering_max_angle_rad, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.car.steering_center, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.car.steering_range, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.car.steering_ramp_time, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.car.axis_distance, &send_index);

			// Multirotor settings
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.vel_decay_e, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.vel_decay_l, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.vel_max, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.map_min_x, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.map_max_x, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.map_min_y, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.map_max_y, &send_index);

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.vel_gain_p, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.vel_gain_i, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.vel_gain_d, &send_index);

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.tilt_gain_p, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.tilt_gain_i, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.tilt_gain_d, &send_index);

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.max_corr_error, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.max_tilt_error, &send_index);

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_roll_p, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_roll_i, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_roll_dp, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_roll_de, &send_index);

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_pitch_p, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_pitch_i, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_pitch_dp, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_pitch_de, &send_index);

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_yaw_p, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_yaw_i, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_yaw_dp, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_yaw_de, &send_index);

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_pos_p, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_pos_i, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_pos_d, &send_index);

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_alt_p, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_alt_i, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_alt_d, &send_index);

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.js_gain_tilt, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.js_gain_yaw, &send_index);
			m_send_buffer[send_index++] = main_cfg_tmp.mr.js_mode_rate;

			m_send_buffer[send_index++] = main_cfg_tmp.mr.motor_fl_f;
			m_send_buffer[send_index++] = main_cfg_tmp.mr.motor_bl_l;
			m_send_buffer[send_index++] = main_cfg_tmp.mr.motor_fr_r;
			m_send_buffer[send_index++] = main_cfg_tmp.mr.motor_br_b;
			m_send_buffer[send_index++] = main_cfg_tmp.mr.motors_x;
			m_send_buffer[send_index++] = main_cfg_tmp.mr.motors_cw;
			buffer_append_uint16(m_send_buffer, main_cfg_tmp.mr.motor_pwm_min_us, &send_index);
			buffer_append_uint16(m_send_buffer, main_cfg_tmp.mr.motor_pwm_max_us, &send_index);

			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_RC_CONTROL: {
			RC_MODE mode;
			float throttle, steering;
			int32_t ind = 0;
			mode = data[ind++];
			throttle = buffer_get_float32(data, 1e4, &ind);
			steering = buffer_get_float32(data, 1e6, &ind);

			utils_truncate_number(&steering, -1.0, 1.0);
			//steering *= autopilot_get_steering_scale();

			//autopilot_set_active(false);

			switch (mode) {
			case RC_MODE_CURRENT:
				if (!main_config.car.disable_motor) {
					comm_can_set_vesc_id(VESC_ID);
					bldc_interface_set_current(throttle);
				}
				break;
			case RC_MODE_DUTY:
				utils_truncate_number(&throttle, -1.0, 1.0);
				if (!main_config.car.disable_motor) {
					comm_can_set_vesc_id(VESC_ID);
					bldc_interface_set_duty_cycle(throttle);
				}
				break;
			default:
				break;
			}
			steering = utils_map(steering, -1.0, 1.0,
					main_config.car.steering_center + (main_config.car.steering_range / 2.0),
					main_config.car.steering_center - (main_config.car.steering_range / 2.0));
			servo_pwm_set(0, steering);
		} break;

		case CMD_SET_SERVO_DIRECT: {
			int32_t ind = 0;
			float steering = buffer_get_float32(data, 1e6, &ind);
			utils_truncate_number(&steering, 0.0, 1.0);
			servo_pwm_set(0, steering);
		} break;

		case CMD_SET_ENU_REF: {
			commands_set_send_func(func);

			int32_t ind = 0;
			double lat, lon, height;
			lat = buffer_get_double64(data, D(1e16), &ind);
			lon = buffer_get_double64(data, D(1e16), &ind);
			height = buffer_get_float32(data, 1e3, &ind);
			pos_set_enu_ref(lat, lon, height);

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		default:
			break;
		}
	}
}

void commands_printf(const char* format, ...) {
//	if (!m_init_done) {
//		return;
//	}

	//chMtxLock(&m_print_gps);
	va_list arg;
	va_start (arg, format);
	commands_vprintf(format, arg);
	va_end (arg);
	//chMtxUnlock(&m_print_gps);
}

void commands_vprintf(const char* format, va_list args) {
	int len;
	static char print_buffer[512];

	print_buffer[0] = main_id;
	print_buffer[1] = CMD_PRINTF;
	len = vsnprintf(print_buffer + 2, 509, format, args);

	if(len > 0) {
		commands_send_packet((unsigned char*)print_buffer, (len<509) ? len + 2: 512);
	}
}

void commands_send_nmea(const char *data, unsigned int len) {
	if (main_config.gps_send_nmea) {
		int32_t send_index = 0;
		m_send_buffer[send_index++] = main_id;
		m_send_buffer[send_index++] = CMD_SEND_NMEA_RADIO;
		memcpy(m_send_buffer + send_index, data, len);
		send_index += len;
		commands_send_packet(m_send_buffer, send_index);
	}
}

void commands_init_plot(char *namex, char *namey) {
	int ind = 0;
	m_send_buffer[ind++] = main_id;
	m_send_buffer[ind++] = CMD_PLOT_INIT;
	memcpy(m_send_buffer + ind, namex, strlen(namex));
	ind += strlen(namex);
	m_send_buffer[ind++] = '\0';
	memcpy(m_send_buffer + ind, namey, strlen(namey));
	ind += strlen(namey);
	m_send_buffer[ind++] = '\0';
	commands_send_packet((unsigned char*)m_send_buffer, ind);
}

void commands_plot_add_graph(char *name) {
	int ind = 0;
	m_send_buffer[ind++] = main_id;
	m_send_buffer[ind++] = CMD_PLOT_ADD_GRAPH;
	memcpy(m_send_buffer + ind, name, strlen(name));
	ind += strlen(name);
	m_send_buffer[ind++] = '\0';
	commands_send_packet((unsigned char*)m_send_buffer, ind);
}

void commands_plot_set_graph(int graph) {
	int ind = 0;
	m_send_buffer[ind++] = main_id;
	m_send_buffer[ind++] = CMD_PLOT_SET_GRAPH;
	m_send_buffer[ind++] = graph;
	commands_send_packet((unsigned char*)m_send_buffer, ind);
}

void commands_send_plot_points(float x, float y) {
	int32_t ind = 0;
	m_send_buffer[ind++] = main_id;
	m_send_buffer[ind++] = CMD_PLOT_DATA;
	buffer_append_float32_auto(m_send_buffer, x, &ind);
	buffer_append_float32_auto(m_send_buffer, y, &ind);
	commands_send_packet((unsigned char*)m_send_buffer, ind);
}
