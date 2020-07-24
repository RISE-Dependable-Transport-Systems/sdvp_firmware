/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

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

#include "timeout.h"

// Private variables
static systime_t m_timeout_msec;
static systime_t m_last_update_time;
static void (*m_timeout_action_cb)(void);
static void (*m_timeout_reset_cb)(void);

// Threads
static THD_WORKING_AREA(timeout_thread_wa, 512);
static THD_FUNCTION(timeout_thread, arg);

void timeout_init(systime_t timeoutms, void (*timeout_action_cb)(void), void (*timeout_reset_cb)(void)) {
	m_timeout_msec = timeoutms;
	m_timeout_action_cb = timeout_action_cb;
	m_timeout_reset_cb = timeout_reset_cb;
	m_last_update_time = chVTGetSystemTimeX();

	chThdCreateStatic(timeout_thread_wa, sizeof(timeout_thread_wa), HIGHPRIO, timeout_thread, NULL);
}

void timeout_reset(void) {
	m_last_update_time = chVTGetSystemTimeX();
}

static THD_FUNCTION(timeout_thread, arg) {
	(void)arg;

	chRegSetThreadName("Timeout");

	for(;;) {
		if (m_timeout_msec != 0 && chVTTimeElapsedSinceX(m_last_update_time) > TIME_MS2I(m_timeout_msec))
			m_timeout_action_cb();
		else
			m_timeout_reset_cb();

		chThdSleepMilliseconds(100);
	}
}
