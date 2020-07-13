#include "commands.h"
#include "conf_general.h"
#include <stdint.h>

void commands_init(void) {

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
    (void)func;

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
		default:
			break;
		}
	}
}
