#include "commands.h"

void commands_init(void) {

}

void commands_process_packet(unsigned char *data, unsigned int len,
		void (*func)(unsigned char *data, unsigned int len)) {

    (void)data;
    (void)len;
    (void)func;
}
