/*
	Copyright 2020        Marvin Damschen	marvin.damschen@ri.se

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

	This file uses code from RC_Controller by Benjamin Vedder (pos).
 */

#include "ch.h"

#define MS_PER_DAY (24 * 60 * 60 * 1000)

// Private variables
static int32_t m_ms_today = -1;
static systime_t last_update = -1;
static int32_t m_pps_time_ref = -1;
static int32_t m_pps_cnt = 0;

void time_today_set_ms(int32_t ms_today) {
	m_ms_today = ms_today;
	last_update = chVTGetSystemTimeX();
}

// updates the time lazily, ignores drift as PPS is assumed
int32_t time_today_get_ms(void) {
	if (m_ms_today == -1)
		return -1;

	systime_t now = chVTGetSystemTimeX();
	m_ms_today += TIME_I2MS(chTimeDiffX(last_update, now));
	last_update = now;

	if (m_ms_today >= MS_PER_DAY) {
		m_ms_today -= MS_PER_DAY;
	}

	return m_ms_today;
}

void time_today_set_pps_time_ref(int32_t pps_time_ref) {
	m_pps_time_ref = pps_time_ref;
}

void time_today_pps_cb(void *arg) {
	(void)arg;
	static int32_t last_time_ref = 0;

	// Only one correction per time stamp.
	if (last_time_ref == m_pps_time_ref) {
		return;
	}

	// Assume that the last time reference (NMEA time stamp) is less
	// than one second old and round to the closest second after it.
	if (m_pps_time_ref != 0) {
		int32_t s_today = m_pps_time_ref / 1000;
		s_today++;
		m_ms_today = s_today * 1000;
	}

	last_time_ref = m_pps_time_ref;
	last_update = chVTGetSystemTimeX();
	m_pps_cnt++;
}

int32_t time_today_get_pps_cnt(void) {
	return m_pps_cnt;
}
