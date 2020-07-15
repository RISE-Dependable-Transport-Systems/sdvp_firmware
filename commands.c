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
			buffer_append_float32(m_send_buffer, 0.0, 1e4, &send_index); // 56
			buffer_append_float32(m_send_buffer, 0.0, 1e4, &send_index); // 60
			buffer_append_float32(m_send_buffer, 0.0, 1e6, &send_index); // 64
			buffer_append_float32(m_send_buffer, 0.0, 1e6, &send_index); // 68
			buffer_append_float32(m_send_buffer, 0.0, 1e6, &send_index); // 72
			m_send_buffer[send_index++] = 0; // 73
			buffer_append_float32(m_send_buffer, 0.0, 1e4, &send_index); // 77
			buffer_append_float32(m_send_buffer, 0.0, 1e4, &send_index); // 81
			buffer_append_float32(m_send_buffer, 0.0, 1e4, &send_index); // 85
			buffer_append_float32(m_send_buffer, 0.0, 1e4, &send_index); // 89
			buffer_append_float32(m_send_buffer, 0.0, 1e6, &send_index); // 93
			buffer_append_int32(m_send_buffer, 0, &send_index); // 97
			buffer_append_int16(m_send_buffer, 0, &send_index); // 99
			buffer_append_float32(m_send_buffer, 0.0, 1e4, &send_index); // 103
			buffer_append_float32(m_send_buffer, 0.0, 1e4, &send_index); // 107

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

