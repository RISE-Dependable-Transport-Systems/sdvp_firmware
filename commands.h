#ifndef COMMANDS_H_
#define COMMANDS_H_

// Functions
void commands_init(void);
void commands_set_send_func(void(*func)(unsigned char *data, unsigned int len));
void commands_send_packet(unsigned char *data, unsigned int len);
void commands_process_packet(unsigned char *data, unsigned int len,
		void (*func)(unsigned char *data, unsigned int len));
void commands_printf(const char* format, ...);

#endif /* COMMANDS_H_ */
