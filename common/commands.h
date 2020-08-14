/*
	Copyright 2016 - 2018 Benjamin Vedder	benjamin@vedder.se
	          2020        Marvin Damschen	marvin.damschen@ri.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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
