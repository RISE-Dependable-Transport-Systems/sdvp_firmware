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

#include "log.h"
#include "pos.h"
#include "pos_mc.h"
#include "pos_imu.h"
#include "pos_gnss.h"
#include "commands.h"
#include "servo_pwm.h"
#include "comm_can.h"
#include "ch.h"
#include "time_today.h"

#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <math.h>

// private variables
static int m_log_rate_hz;
static bool m_log_en;
static bool m_write_split;
static char m_log_name[LOG_NAME_MAX_LEN + 1];

// Threads
static THD_WORKING_AREA(log_thread_wa, 2048);
static THD_FUNCTION(log_thread, arg);

// Private functions
static void print_log_ext(void);

void log_init(void) {
	m_log_rate_hz = 10;
	m_log_en = false;
	m_write_split = true;
	strcpy(m_log_name, "Undefined");

	chThdCreateStatic(log_thread_wa, sizeof(log_thread_wa),
			NORMALPRIO, log_thread, NULL);
}

void log_set_rate(int rate_hz) {
	m_log_rate_hz = rate_hz;
}

void log_set_enabled(bool enabled) {
	if (enabled && !m_log_en) {
		m_write_split = true;
	}

	m_log_en = enabled;
}

void log_set_name(char *name) {
	strcpy(m_log_name, name);
}

static THD_FUNCTION(log_thread, arg) {
	(void)arg;

	chRegSetThreadName("Log");

	systime_t time_p = chVTGetSystemTimeX(); // T0

	for(;;) {
		if (m_log_en) {
			if (m_write_split) {
				commands_printf_log_serial("//%s\n"
						"//timestamp (ms),"
						"timestamp pos today (ms),"
						"car x,"
						"car y,"
						"roll,"
						"pitch,"
						"yaw,"
						"roll rate,"
						"pitch rate,"
						"yaw rate,"
						"accel_x,"
						"accel_y,"
						"accel_z,"
						"mag_x,"
						"mag_y,"
						"mag_z,"
						"speed (m/s),"
						"tachometer,"
						"timestamp gps sample today (ms),"
						"lat,"
						"lon,"
						"height,"
						"Travel distance,"
						"Yaw IMU,"
						"speed GNSS (m/s),"
						"GNSS fix type\r\n",
						m_log_name);
				m_write_split = false;
			}

			print_log_ext();
		}

		time_p += CH_CFG_ST_FREQUENCY / m_log_rate_hz;
		systime_t time = chVTGetSystemTimeX();

		if (time_p >= time + 5) {
			chThdSleepUntil(time_p);
		} else {
			chThdSleepMilliseconds(1);
		}
	}
}

static void print_log_ext(void) {
	static mc_values val;
	static POS_STATE pos;
	static GPS_STATE gps;
	float accel[3];
	float mag[3];

	pos_get(&pos);
	pos_mc_get(&val);
	pos_gnss_get(&gps);
	pos_imu_get(accel, 0, mag);
	uint32_t ms_chtime = chTimeI2MS(chVTGetSystemTimeX());
	uint32_t ms_today = time_today_get_ms();

	commands_printf_log_serial(
			"%u,"     // timestamp (ms)
			"%u,"     // timestamp pos today (ms)
			"%.3f,"   // car x
			"%.3f,"   // car y
			"%.2f,"   // roll
			"%.2f,"   // pitch
			"%.2f,"   // yaw
			"%.2f,"   // roll rate
			"%.2f,"   // pitch rate
			"%.2f,"   // yaw rate
			"%.2f,"   // accel_x
			"%.2f,"   // accel_y
			"%.2f,"   // accel_z
			"%.2f,"   // mag_x
			"%.2f,"   // mag_y
			"%.2f,"   // mag_z
			"%.3f,"   // speed (m/s)
			"%d,"     // tachometer
			"%u,"     // timestamp gps sample today (ms)
			"%.7f,"   // lat
			"%.7f,"   // lon
			"%.3f,"  // height
			"%.3f,"  // Travel distance
			"%.2f,"  // Yaw IMU
			"%.3f," // speed GNSS (m/s)
			"%d\r\n", // GNSS fix type

			ms_chtime,
			ms_today,
			(double)pos.px,
			(double)pos.py,
			(double)pos.roll,
			(double)pos.pitch,
			(double)pos.yaw,
			(double)pos.roll_rate,
			(double)pos.pitch_rate,
			(double)pos.yaw_rate,
			(double)accel[0],
			(double)accel[1],
			(double)accel[2],
			(double)mag[0],
			(double)mag[1],
			(double)mag[2],
			(double)pos.speed,
			val.tachometer,
			gps.ms,
			gps.lat,
			gps.lon,
			gps.height,
			(double)(val.tachometer * main_config.car.gear_ratio
			* (2.0 / main_config.car.motor_poles) * (1.0 / 6.0)
			* main_config.car.wheel_diam * M_PI),
			(double)pos.yaw_imu,
			pos_get_gnss_speed(),
			gps.fix_type);
}
