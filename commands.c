#include "commands.h"
#include "packet.h"
#include "buffer.h"
#include "conf_general.h"
#include "pos.h"
#include <stdint.h>

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
		case CMD_GET_STATE: {
			float accel[3];
			float gyro[3];
			float mag[3];

			commands_set_send_func(func);

			pos_get_imu(accel, gyro, mag);

			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret; // 1
			m_send_buffer[send_index++] = CMD_GET_STATE; // 2
			m_send_buffer[send_index++] = 12; // 3
			m_send_buffer[send_index++] = 2; // 4
			buffer_append_float32(m_send_buffer, 0.0, 1e6, &send_index); // 8
			buffer_append_float32(m_send_buffer, 0.0, 1e6, &send_index); // 12
			buffer_append_float32(m_send_buffer, 0.0, 1e6, &send_index); // 16
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
		default:
			break;
		}
	}
}
