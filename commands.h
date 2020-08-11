#ifndef COMMANDS_H_
#define COMMANDS_H_

#include <stdarg.h>

// Functions
void commands_set_send_func(void(*func)(unsigned char *data, unsigned int len));
void commands_send_packet(unsigned char *data, unsigned int len);
void commands_process_packet(unsigned char *data, unsigned int len,
		void (*func)(unsigned char *data, unsigned int len));
void commands_printf(const char* format, ...);
void commands_vprintf(const char* format, va_list args);
void commands_printf_log_serial(char* format, ...);
void commands_send_nmea(const char *data, unsigned int len);
void commands_init_plot(char *namex, char *namey);
void commands_plot_add_graph(char *name);
void commands_plot_set_graph(int graph);
void commands_send_plot_points(float x, float y);

#endif /* COMMANDS_H_ */
