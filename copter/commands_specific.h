/*
 * commands_specific.h
 *
 *  Created on: 17 aug. 2020
 *      Author: marvind
 */

#ifndef COPTER_COMMANDS_SPECIFIC_H_
#define COPTER_COMMANDS_SPECIFIC_H_

#include "datatypes.h"

bool commands_specific_process_packet(CMD_PACKET packet_id, unsigned char *data, unsigned int len,
				int id_ret, void (*func)(unsigned char *data, unsigned int len), uint8_t *send_buffer);

#endif /* COPTER_COMMANDS_SPECIFIC_H_ */
