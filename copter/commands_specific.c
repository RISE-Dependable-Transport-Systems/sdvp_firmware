/*
 * commands_specific.c
 *
 *  Created on: 17 aug. 2020
 *      Author: marvind
 */

#include "commands_specific.h"
#include "commands.h"
#include "buffer.h"
#include "time_today.h"
#include "conf_general.h"
#include "pos.h"
#include "pos_imu.h"
#include "copter_control.h"
#include "autopilot.h"

bool commands_specific_process_packet(CMD_PACKET packet_id, unsigned char *data, unsigned int len,
		int id_ret, void (*func)(unsigned char *data, unsigned int len), uint8_t *send_buffer) {
	(void) len;

	// Copter-specific commands
	switch(packet_id) {
	case CMD_MR_GET_STATE: {
		POS_STATE pos;
		float accel[3];
		float gyro[3];
		float mag[3];
		ROUTE_POINT rp_goal;

		commands_set_send_func(func);

		pos_imu_get(accel, gyro, mag);
		pos_get(&pos);
		autopilot_get_goal_now(&rp_goal);

		int32_t send_index = 0;
		send_buffer[send_index++] = id_ret; // 1
		send_buffer[send_index++] = CMD_MR_GET_STATE; // 2
		send_buffer[send_index++] = FW_VERSION_MAJOR; // 3
		send_buffer[send_index++] = FW_VERSION_MINOR; // 4
		buffer_append_float32_auto(send_buffer, pos.roll, &send_index); // 8
		buffer_append_float32_auto(send_buffer, pos.pitch, &send_index); // 12
		buffer_append_float32_auto(send_buffer, pos.yaw, &send_index); // 16
		buffer_append_float32_auto(send_buffer, accel[0], &send_index); // 20
		buffer_append_float32_auto(send_buffer, accel[1], &send_index); // 24
		buffer_append_float32_auto(send_buffer, accel[2], &send_index); // 28
		buffer_append_float32_auto(send_buffer, gyro[0], &send_index); // 32
		buffer_append_float32_auto(send_buffer, gyro[1], &send_index); // 36
		buffer_append_float32_auto(send_buffer, gyro[2], &send_index); // 40
		buffer_append_float32_auto(send_buffer, mag[0], &send_index); // 44
		buffer_append_float32_auto(send_buffer, mag[1], &send_index); // 48
		buffer_append_float32_auto(send_buffer, mag[2], &send_index); // 52
		buffer_append_float32_auto(send_buffer, pos.px, &send_index); // 56
		buffer_append_float32_auto(send_buffer, pos.py, &send_index); // 60
		buffer_append_float32_auto(send_buffer, pos.pz, &send_index); // 64
		buffer_append_float32_auto(send_buffer, pos.speed, &send_index); // 68
		buffer_append_float32_auto(send_buffer, 0.0, &send_index); // 72
		buffer_append_float32_auto(send_buffer, pos.px_gps, &send_index); // 76
		buffer_append_float32_auto(send_buffer, pos.py_gps, &send_index); // 80
		buffer_append_float32_auto(send_buffer, rp_goal.px, &send_index); // 84
		buffer_append_float32_auto(send_buffer, rp_goal.py, &send_index); // 88
		buffer_append_int32(send_buffer, time_today_get_ms(), &send_index); // 92
		commands_send_packet(send_buffer, send_index);
	} break;

	case CMD_MR_RC_CONTROL: {
		int32_t ind = 0;
		float throttle = buffer_get_float32_auto(data, &ind);
		float roll = buffer_get_float32_auto(data, &ind);
		float pitch = buffer_get_float32_auto(data, &ind);
		float yaw = buffer_get_float32_auto(data, &ind);
		copter_control_set_input(throttle, roll, pitch, yaw);
	} break;

	case CMD_MR_OVERRIDE_POWER: {
		int32_t ind = 0;
		copter_control_set_motor_override(0, buffer_get_float32_auto(data, &ind));
		copter_control_set_motor_override(1, buffer_get_float32_auto(data, &ind));
		copter_control_set_motor_override(2, buffer_get_float32_auto(data, &ind));
		copter_control_set_motor_override(3, buffer_get_float32_auto(data, &ind));
	} break;

	default:
		return false;
		break;
	}

	return true;
}
