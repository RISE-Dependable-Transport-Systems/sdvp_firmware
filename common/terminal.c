/*
	Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se
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

#include "ch.h"
#include "hal.h"
#include "conf_general.h"
#include "terminal.h"
#include <string.h>
#include <stdio.h>

// Settings
#define CALLBACK_LEN						80

// Private types
typedef struct _terminal_callback_struct {
	const char *command;
	const char *help;
	const char *arg_names;
	void(*cbf)(int argc, const char **argv);
} terminal_callback_struct;

// Private variables
static void(*m_vprintf)(const char* format, va_list args) = 0;
static terminal_callback_struct callbacks[CALLBACK_LEN];
static int callback_write = 0;

void terminal_set_vprintf(void(*vprintf)(const char* format, va_list args)) {
	m_vprintf = vprintf;
}

void terminal_printf(const char* format, ...) {
	if (m_vprintf) {
	    va_list vargs;
	    va_start(vargs, format);
	    m_vprintf(format, vargs);
	    va_end(vargs);
	}
}

void terminal_process_string(char *str) {
	enum { kMaxArgs = 64 };
	int argc = 0;
	char *argv[kMaxArgs];

	char *p2 = strtok(str, " ");
	while (p2 && argc < kMaxArgs) {
		argv[argc++] = p2;
		p2 = strtok(0, " ");
	}

	if (argc == 0) {
		terminal_printf("No command received\n");
		return;
	}

	if (strcmp(argv[0], "ping") == 0) {
		terminal_printf("pong\n");
	} else if (strcmp(argv[0], "mem") == 0) {
		size_t n, total, largest;
		n = chHeapStatus(NULL, &total, &largest);
		terminal_printf("core free memory : %u bytes", chCoreGetStatusX());
		terminal_printf("heap fragments   : %u", n);
		terminal_printf("heap free total  : %u bytes", total);
		terminal_printf("heap free largest: %u bytes", largest);
	} else if (strcmp(argv[0], "threads") == 0) {
		thread_t *tp;
		static const char *states[] = {CH_STATE_NAMES};
		terminal_printf("stklimit    stack     addr refs prio     state               name");
		terminal_printf("-----------------------------------------------------------------");
		tp = chRegFirstThread();
		do {
#if (CH_DBG_ENABLE_STACK_CHECK == TRUE) || (CH_CFG_USE_DYNAMIC == TRUE)
			uint32_t stklimit = (uint32_t)tp->wabase;
#else
    		uint32_t stklimit = 0U;
#endif
			terminal_printf("%08lx %08lx %08lx %4lu %4lu %9s %18s",
		             stklimit, (uint32_t)tp->ctx.sp, (uint32_t)tp,
		             (uint32_t)tp->refs - 1, (uint32_t)tp->prio, states[tp->state],
		             tp->name == NULL ? "" : tp->name);
			tp = chRegNextThread(tp);
		} while (tp != NULL);
		terminal_printf(" ");
	}

	// The help command
	else if (strcmp(argv[0], "help") == 0) {
		terminal_printf("Valid commands are:");
		terminal_printf("help");
		terminal_printf("  Show this help");

		terminal_printf("ping");
		terminal_printf("  Print pong here to see if the reply works");

		terminal_printf("mem");
		terminal_printf("  Show memory usage");

		terminal_printf("threads");
		terminal_printf("  List all threads");

		for (int i = 0;i < callback_write;i++) {
			if (callbacks[i].arg_names) {
				terminal_printf("%s %s", callbacks[i].command, callbacks[i].arg_names);
			} else {
				terminal_printf(callbacks[i].command);
			}

			if (callbacks[i].help) {
				terminal_printf("  %s", callbacks[i].help);
			} else {
				terminal_printf("  There is no help available for this command.");
			}
		}

		terminal_printf(" ");
	} else {
		bool found = false;
		for (int i = 0;i < callback_write;i++) {
			if (strcmp(argv[0], callbacks[i].command) == 0) {
				callbacks[i].cbf(argc, (const char**)argv);
				found = true;
				break;
			}
		}

		if (!found) {
			terminal_printf("Invalid command: %s\n"
					"type help to list all available commands\n", argv[0]);
		}
	}
}

/**
 * Register a custom command  callback to the terminal. If the command
 * is already registered the old command callback will be replaced.
 *
 * @param command
 * The command name.
 *
 * @param help
 * A help text for the command. Can be NULL.
 *
 * @param arg_names
 * The argument names for the command, e.g. [arg_a] [arg_b]
 * Can be NULL.
 *
 * @param cbf
 * The callback function for the command.
 */
void terminal_register_command_callback(
		const char* command,
		const char *help,
		const char *arg_names,
		void(*cbf)(int argc, const char **argv)) {

	int callback_num = callback_write;

	for (int i = 0;i < callback_write;i++) {
		// First check the address in case the same callback is registered more than once.
		if (callbacks[i].command == command) {
			callback_num = i;
			break;
		}

		// Check by string comparison.
		if (strcmp(callbacks[i].command, command) == 0) {
			callback_num = i;
			break;
		}
	}

	callbacks[callback_num].command = command;
	callbacks[callback_num].help = help;
	callbacks[callback_num].arg_names = arg_names;
	callbacks[callback_num].cbf = cbf;

	if (callback_num == callback_write) {
		callback_write++;
		if (callback_write >= CALLBACK_LEN) {
			callback_write = 0;
		}
	}
}
