/*
 * time_today.c
 *
 *  Created on: 22 juli 2020
 *      Author: marvind
 */

#include "ch.h"

#define MS_PER_DAY (24 * 60 * 60 * 1000)

// Private variables
static int32_t m_ms_today = -1;
static systime_t last_update = -1;
static int32_t m_pps_time_ref = -1;

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
}
