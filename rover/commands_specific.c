/*
 * commands_specific.c
 *
 *  Created on: 17 aug. 2020
 *      Author: marvind
 */

#include "commands_specific.h"
#include "commands.h"
#include "utils.h"
#include "buffer.h"
#include "conf_general.h"
#include "servo_pwm.h"
#include "bldc_interface.h"
#include "comm_can.h"
#include "pos.h"
#include "pos_imu.h"
#include "autopilot.h"
#include "time_today.h"

bool commands_specific_process_packet(CMD_PACKET packet_id, unsigned char *data, unsigned int len,
		int id_ret, void (*func)(unsigned char *data, unsigned int len), uint8_t *send_buffer) {
	(void)len;

	// Rover-specific commands
	switch(packet_id) {
	case CMD_GET_STATE: {
		POS_STATE pos;
		float accel[3];
		float gyro[3];
		float mag[3];
		const mc_values mcval = bldc_interface_get_last_received_values();
		ROUTE_POINT rp_goal;

		commands_set_send_func(func);

		pos_get(&pos);
		pos_imu_get(accel, gyro, mag);
		autopilot_get_goal_now(&rp_goal);

		int32_t send_index = 0;
		send_buffer[send_index++] = id_ret; // 1
		send_buffer[send_index++] = CMD_GET_STATE; // 2
		send_buffer[send_index++] = FW_VERSION_MAJOR; // 3
		send_buffer[send_index++] = FW_VERSION_MINOR; // 4
		buffer_append_float32(send_buffer, pos.roll, 1e6, &send_index); // 8
		buffer_append_float32(send_buffer, pos.pitch, 1e6, &send_index); // 12
		buffer_append_float32(send_buffer, pos.yaw, 1e6, &send_index); // 16
		buffer_append_float32(send_buffer, accel[0], 1e6, &send_index); // 20
		buffer_append_float32(send_buffer, accel[1], 1e6, &send_index); // 24
		buffer_append_float32(send_buffer, accel[2], 1e6, &send_index); // 28
		buffer_append_float32(send_buffer, gyro[0], 1e6, &send_index); // 32
		buffer_append_float32(send_buffer, gyro[1], 1e6, &send_index); // 36
		buffer_append_float32(send_buffer, gyro[2], 1e6, &send_index); // 40
		buffer_append_float32(send_buffer, mag[0], 1e6, &send_index); // 44
		buffer_append_float32(send_buffer, mag[1], 1e6, &send_index); // 48
		buffer_append_float32(send_buffer, mag[2], 1e6, &send_index); // 52
		buffer_append_float32(send_buffer, pos.px, 1e4, &send_index); // 56
		buffer_append_float32(send_buffer, pos.py, 1e4, &send_index); // 60
		buffer_append_float32(send_buffer, pos.speed, 1e6, &send_index); // 64
		buffer_append_float32(send_buffer, mcval.v_in, 1e6, &send_index); // 68
		buffer_append_float32(send_buffer, mcval.temp_mos, 1e6, &send_index); // 72
		send_buffer[send_index++] = mcval.fault_code; // 73
		buffer_append_float32(send_buffer, pos.px_gps, 1e4, &send_index); // 77
		buffer_append_float32(send_buffer, pos.py_gps, 1e4, &send_index); // 81
		buffer_append_float32(send_buffer, rp_goal.px, 1e4, &send_index); // 85
		buffer_append_float32(send_buffer, rp_goal.py, 1e4, &send_index); // 89
		buffer_append_float32(send_buffer, autopilot_get_rad_now(), 1e6, &send_index); // 93
		buffer_append_int32(send_buffer, time_today_get_ms(), &send_index); // 97
		buffer_append_int16(send_buffer, autopilot_get_route_left(), &send_index); // 99
		buffer_append_float32(send_buffer, pos.px, 1e4, &send_index); // 103
		buffer_append_float32(send_buffer, pos.py, 1e4, &send_index); // 107

		commands_send_packet(send_buffer, send_index);
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
		servo_pwm_set_ramped(0, steering);
	} break;

	case CMD_SET_SERVO_DIRECT: {
		int32_t ind = 0;
		float steering = buffer_get_float32(data, 1e6, &ind);
		utils_truncate_number(&steering, 0.0, 1.0);
		servo_pwm_set_ramped(0, steering);
	} break;

	default:
		return false;
		break;
	}

	return true;
}
