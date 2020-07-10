#ifndef COMMANDS_H_
#define COMMANDS_H_

// Functions
void commands_init(void);
void commands_process_packet(unsigned char *data, unsigned int len,
		void (*func)(unsigned char *data, unsigned int len));

#endif /* COMMANDS_H_ */
