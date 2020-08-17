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

#include "pos.h"

#include "ch.h"
#include "utils.h"
#include "commands.h" // TODO might make sense to factor out
#include "conf_general.h"
#include "time_today.h"
#include "servo_pwm.h" // TODO factor out
#include "terminal.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#define POS_HISTORY_LEN					100

// Private variables
static POS_STATE m_pos;
static POS_POINT m_pos_history[POS_HISTORY_LEN];
static int m_pos_history_ptr;
static mutex_t m_mutex_pos;
static bool m_en_delay_comp;
static bool m_gps_corr_print;
static bool m_pos_history_print;
static void (*m_pos_correction_gnss_hook)(POS_STATE *, float) = NULL;
static void (*m_pos_correction_imu_hook)(POS_STATE *, float) = NULL;
static void (*m_pos_correction_imu_post_hook)(float) = NULL;
// IMU
static float m_imu_yaw_offset;
static float m_yaw_imu_clamp;
static bool m_yaw_imu_clamp_set;

// Private functions
static void cmd_terminal_delay_info(int argc, const char **argv);
static void cmd_terminal_gps_corr_info(int argc, const char **argv);
static void cmd_terminal_delay_comp(int argc, const char **argv);
static void save_pos_history(void);
static POS_POINT get_closest_point_to_time(int32_t time);

void pos_init(void) {
	memset(&m_pos, 0, sizeof(m_pos));
	memset(&m_pos_history, 0, sizeof(m_pos_history));
	m_pos_history_ptr = 0;
	m_pos_history_print = false;
	m_gps_corr_print = false;
	m_en_delay_comp = true;
	m_imu_yaw_offset = 0.0;

	m_yaw_imu_clamp = 0.0;
	m_yaw_imu_clamp_set = false;

	chMtxObjectInit(&m_mutex_pos);

	terminal_register_command_callback(
			"pos_delay_info",
			"Print and plot delay information when doing GNSS position correction.\n"
			"  0 - Disabled\n"
			"  1 - Enabled",
			"[print_en]",
			cmd_terminal_delay_info);

	terminal_register_command_callback(
			"pos_gnss_corr_info",
			"Print and plot correction information when doing GNSS position correction.\n"
			"  0 - Disabled\n"
			"  1 - Enabled",
			"[print_en]",
			cmd_terminal_gps_corr_info);

	terminal_register_command_callback(
			"pos_delay_comp",
			"Enable or disable delay compensation.\n"
			"  0 - Disabled\n"
			"  1 - Enabled",
			"[enabled]",
			cmd_terminal_delay_comp);
}

void pos_get(POS_STATE *p) {
	chMtxLock(&m_mutex_pos);
	*p = m_pos;
	chMtxUnlock(&m_mutex_pos);
}

float pos_get_yaw(void) {
	return m_pos.yaw;
}

float pos_get_speed(void) {
	return m_pos.speed;
}

float pos_get_gnss_speed(void) {
	const float dt_gps = (m_pos.gps_ms - m_pos.gps_ms_last) / 1000.0;
	const float vx_gps = (m_pos.px_gps - m_pos.px_gps_last) / dt_gps;
	const float vy_gps = (m_pos.py_gps - m_pos.py_gps_last) / dt_gps;

	return sqrtf(SQ(vx_gps) + SQ(vy_gps));
}

void pos_set_xya(float x, float y, float angle) {
	chMtxLock(&m_mutex_pos);

	m_pos.px = x;
	m_pos.py = y;
	m_pos.yaw = angle;
	m_imu_yaw_offset = m_pos.yaw_imu - angle;
	m_yaw_imu_clamp = angle;

	chMtxUnlock(&m_mutex_pos);
}

void pos_set_yaw_offset(float angle) {
	chMtxLock(&m_mutex_pos);

	m_imu_yaw_offset = angle;
	utils_norm_angle(&m_imu_yaw_offset);
	m_pos.yaw = m_pos.yaw_imu - m_imu_yaw_offset;
	utils_norm_angle(&m_pos.yaw);
	m_yaw_imu_clamp = m_pos.yaw;

	chMtxUnlock(&m_mutex_pos);
}

static void cmd_terminal_delay_info(int argc, const char **argv) {
	if (argc == 2) {
		if (strcmp(argv[1], "0") == 0) {
			m_pos_history_print = 0;
			terminal_printf("OK\n");
		} else if (strcmp(argv[1], "1") == 0) {
			m_pos_history_print = 1;
			terminal_printf("OK\n");
		} else {
			terminal_printf("Invalid argument %s\n", argv[1]);
		}
	} else {
		terminal_printf("Wrong number of arguments\n");
	}
}

static void cmd_terminal_gps_corr_info(int argc, const char **argv) {
	if (argc == 2) {
		if (strcmp(argv[1], "0") == 0) {
			m_gps_corr_print = 0;
			terminal_printf("OK\n");
		} else if (strcmp(argv[1], "1") == 0) {
			m_gps_corr_print = 1;
			terminal_printf("OK\n");
		} else {
			terminal_printf("Invalid argument %s\n", argv[1]);
		}
	} else {
		terminal_printf("Wrong number of arguments\n");
	}
}

static void cmd_terminal_delay_comp(int argc, const char **argv) {
	if (argc == 2) {
		if (strcmp(argv[1], "0") == 0) {
			m_en_delay_comp = 0;
			terminal_printf("OK\n");
		} else if (strcmp(argv[1], "1") == 0) {
			m_en_delay_comp = 1;
			terminal_printf("OK\n");
		} else {
			terminal_printf("Invalid argument %s\n", argv[1]);
		}
	} else {
		terminal_printf("Wrong number of arguments\n");
	}
}

void pos_correction_imu(const float roll, const float pitch, const float yaw, const float yaw_mag, const float gyro[3], const float quaternions[4], const float dt) {
	chMtxLock(&m_mutex_pos);

	m_pos.roll = roll * 180.0 / M_PI;
	m_pos.pitch = pitch * 180.0 / M_PI;
	m_pos.roll_rate = -gyro[0] * 180.0 / M_PI;
	m_pos.pitch_rate = gyro[1] * 180.0 / M_PI;

	if (main_config.mag_use) {
		static float yaw_now = 0;
		static float yaw_imu_last = 0;

		float yaw_imu_diff = utils_angle_difference_rad(yaw, yaw_imu_last);
		yaw_imu_last = yaw;
		yaw_now += yaw_imu_diff;

		float diff = utils_angle_difference_rad(yaw_mag, yaw_now);
		yaw_now += SIGN(diff) * main_config.yaw_mag_gain * M_PI / 180.0 * dt;
		utils_norm_angle_rad(&yaw_now);

		m_pos.yaw_imu = yaw_now * 180.0 / M_PI;
	} else {
		m_pos.yaw_imu = yaw * 180.0 / M_PI;
	}

	utils_norm_angle(&m_pos.yaw_imu);
	m_pos.yaw_rate = -gyro[2] * 180.0 / M_PI;

	// Correct yaw
	// Prevent IMU drift when vehicle is stationary (only works for land vehicles)
	// TODO: refactor define-dependent code
	if (VEHICLE_TYPE == VEHICLE_TYPE_ROVER) {
		if (!m_yaw_imu_clamp_set) {
			m_yaw_imu_clamp = m_pos.yaw_imu - m_imu_yaw_offset;
			m_yaw_imu_clamp_set = true;
		}

		if (main_config.car.clamp_imu_yaw_stationary && fabsf(m_pos.speed) < 0.05) {
			m_imu_yaw_offset = m_pos.yaw_imu - m_yaw_imu_clamp;
		} else {
			m_yaw_imu_clamp = m_pos.yaw_imu - m_imu_yaw_offset;
		}
	}

	if (VEHICLE_TYPE == VEHICLE_TYPE_ROVER && main_config.car.yaw_use_odometry) {
		if (main_config.car.yaw_imu_gain > 1e-10) {
			float ang_diff = utils_angle_difference(m_pos.yaw, m_pos.yaw_imu - m_imu_yaw_offset);

			if (ang_diff > 1.2 * main_config.car.yaw_imu_gain) {
				m_pos.yaw -= main_config.car.yaw_imu_gain;
				utils_norm_angle(&m_pos.yaw);
			} else if (ang_diff < -1.2 * main_config.car.yaw_imu_gain) {
				m_pos.yaw += main_config.car.yaw_imu_gain;
				utils_norm_angle(&m_pos.yaw);
			} else {
				m_pos.yaw -= ang_diff;
				utils_norm_angle(&m_pos.yaw);
			}
		}
	} else {
		m_pos.yaw = m_pos.yaw_imu - m_imu_yaw_offset;
		utils_norm_angle(&m_pos.yaw);
	}

	m_pos.q0 = quaternions[0];
	m_pos.q1 = quaternions[1];
	m_pos.q2 = quaternions[2];
	m_pos.q3 = quaternions[3];

	// Perform vehicle-type-specific corrections if necessary (should be registered in main)
	if (m_pos_correction_imu_hook)
		m_pos_correction_imu_hook(&m_pos, dt);

	chMtxUnlock(&m_mutex_pos);

	// After corrections, trigger vehicle-type-specific actions if necessary (should be registered in main)
	if (m_pos_correction_imu_post_hook)
		m_pos_correction_imu_post_hook(dt);
}

static void save_pos_history(void) {
	m_pos_history[m_pos_history_ptr].px = m_pos.px;
	m_pos_history[m_pos_history_ptr].py = m_pos.py;
	m_pos_history[m_pos_history_ptr].pz = m_pos.pz;
	m_pos_history[m_pos_history_ptr].yaw = m_pos.yaw;
	m_pos_history[m_pos_history_ptr].speed = m_pos.speed;
	m_pos_history[m_pos_history_ptr].time = time_today_get_ms();

	m_pos_history_ptr++;
	if (m_pos_history_ptr >= POS_HISTORY_LEN) {
		m_pos_history_ptr = 0;
	}
}

static POS_POINT get_closest_point_to_time(int32_t time) {
	if (m_pos_history_ptr == 0 && ((*(uint32_t*)m_pos_history)) == 0) { // return current position when history is empty
		POS_POINT tmp = {m_pos.px, m_pos.py, m_pos.py, m_pos.yaw, m_pos.speed, time_today_get_ms()};
		return tmp;
	}

	int32_t ind = m_pos_history_ptr > 0 ? m_pos_history_ptr - 1 : POS_HISTORY_LEN - 1;
	int32_t min_diff = abs(time - m_pos_history[ind].time);
	int32_t ind_use = ind;

	int cnt = 0;
	for (;;) {
		ind = ind > 0 ? ind - 1 : POS_HISTORY_LEN - 1;

		if (ind == m_pos_history_ptr) {
			break;
		}

		int32_t diff = abs(time - m_pos_history[ind].time);

		if (diff < min_diff) {
			min_diff = diff;
			ind_use = ind;
		} else {
			break;
		}

		cnt++;
	}

	return m_pos_history[ind_use];
}

void pos_correction_gnss(const float gnss_px, const float gnss_py, const float gnss_pz, const int32_t gnss_ms, const int fix_type) {
	if (m_pos.gps_corr_cnt == 0.0)
		m_pos.gps_corr_cnt = sqrtf(SQ(m_pos.px_gps - m_pos.px_gps_last) + SQ(m_pos.py_gps - m_pos.py_gps_last));

	{
		static int sample = 0;
		if (m_pos_history_print) {
			int32_t diff = time_today_get_ms() - gnss_ms;
			terminal_printf("Age: %d gnss_ms, PPS_CNT: %d", diff, time_today_get_pps_cnt());
			if (sample == 0) {
				commands_init_plot("Sample", "Age (gnss_ms)");
				commands_plot_add_graph("Delay");
				commands_plot_set_graph(0);
			}
			commands_send_plot_points(sample++, diff);
		} else {
			sample = 0;
		}
	}

	chMtxLock(&m_mutex_pos);

	// Angle
	if (fabsf(m_pos.speed * 3.6f) > 0.5 || 1) {
		float yaw_gps = -atan2f(gnss_py - m_pos.gps_ang_corr_y_last_gps,
				gnss_px - m_pos.gps_ang_corr_x_last_gps) * 180.0 / M_PI;
		POS_POINT closest = get_closest_point_to_time(
				(gnss_ms + m_pos.gps_ang_corr_last_gps_ms) / 2.0);
		float yaw_diff = utils_angle_difference(yaw_gps, closest.yaw);
		utils_step_towards(&m_imu_yaw_offset, m_imu_yaw_offset - yaw_diff,
				main_config.gps_corr_gain_yaw * m_pos.gps_corr_cnt);
	}

	utils_norm_angle(&m_imu_yaw_offset);

	// Position
	float gain = main_config.gps_corr_gain_stat +
			main_config.gps_corr_gain_dyn * m_pos.gps_corr_cnt;

	POS_POINT closest = get_closest_point_to_time(m_en_delay_comp ? gnss_ms : time_today_get_ms());
	POS_POINT closest_corr = closest;

	{
		static int sample = 0;
		static int ms_before = 0;
		if (m_gps_corr_print) {
			float diff = utils_point_distance(closest.px, closest.py, gnss_px, gnss_py) * 100.0;

			terminal_printf("Diff: %.1f cm, Speed: %.1f km/h, Yaw: %.1f",
					(double)diff, (double)(m_pos.speed * 3.6), (double)m_pos.yaw);

			if (sample == 0) {
				commands_init_plot("Time (s)", "Value");
				commands_plot_add_graph("Diff (cm)");
				commands_plot_add_graph("Speed (0.1 * km/h)");
				commands_plot_add_graph("Yaw (degrees)");
			}

			sample += gnss_ms - ms_before;

			commands_plot_set_graph(0);
			commands_send_plot_points((float)sample / 1000.0, diff);
			commands_plot_set_graph(1);
			commands_send_plot_points((float)sample / 1000.0, m_pos.speed * 3.6 * 10);
			commands_plot_set_graph(2);
			commands_send_plot_points((float)sample / 1000.0, m_pos.yaw);
		} else {
			sample = 0;
		}
		ms_before = gnss_ms;
	}

	utils_step_towards(&closest_corr.px, gnss_px, gain);
	utils_step_towards(&closest_corr.py, gnss_py, gain);
	m_pos.px += closest_corr.px - closest.px;
	m_pos.py += closest_corr.py - closest.py;
	m_pos.pz = gnss_pz - m_pos.gps_ground_level;
	m_pos.gps_corr_time = chVTGetSystemTimeX();
	m_pos.gps_corr_cnt = 0.0;

	m_pos.px_gps_last = m_pos.px_gps;
	m_pos.py_gps_last = m_pos.py_gps;
	m_pos.pz_gps_last = m_pos.pz_gps;
	m_pos.gps_ms_last = m_pos.gps_ms;

	m_pos.px_gps = gnss_px;
	m_pos.py_gps = gnss_py;
	m_pos.pz_gps = gnss_pz;
	m_pos.gps_ms = gnss_ms;
	m_pos.gps_fix_type = fix_type;

	m_pos.gps_ang_corr_x_last_gps = gnss_px;
	m_pos.gps_ang_corr_y_last_gps = gnss_py;
	m_pos.gps_ang_corr_last_gps_ms = gnss_ms;

	// Perform vehicle-type-specific corrections if necessary (should be registered in main)
	if (m_pos_correction_gnss_hook) {
		const float time_now = time_today_get_ms();
		static float time_last = 0;
		float dt = (time_now - time_last) / 1000.0;
		time_last = time_now;

		m_pos_correction_gnss_hook(&m_pos, dt);
	}

	chMtxUnlock(&m_mutex_pos);
}

void pos_correction_mc(float distance, float turn_rad_rear, float angle_diff, float speed) {
	chMtxLock(&m_mutex_pos);

	if (fabsf(distance) > 1e-6) {
		float angle_rad = -m_pos.yaw * M_PI / 180.0;

		m_pos.gps_corr_cnt += fabsf(distance);

		if (!main_config.car.yaw_use_odometry || fabsf(angle_diff) < 1e-6) {
			m_pos.px += cosf(angle_rad) * distance;
			m_pos.py += sinf(angle_rad) * distance;
		} else {
			m_pos.px += turn_rad_rear * (sinf(angle_rad + angle_diff) - sinf(angle_rad));
			m_pos.py += turn_rad_rear * (cosf(angle_rad - angle_diff) - cosf(angle_rad));
			angle_rad += angle_diff;
			utils_norm_angle_rad(&angle_rad);

			m_pos.yaw = -angle_rad * 180.0 / M_PI;
			utils_norm_angle(&m_pos.yaw);
		}
	}

	m_pos.speed = speed;

	save_pos_history();

	chMtxUnlock(&m_mutex_pos);
}

void pos_set_correction_imu_hook(void (pos_correction_imu_hook)(POS_STATE *pos, float dt)) {
	m_pos_correction_imu_hook = pos_correction_imu_hook;
}

void pos_set_correction_imu_post_hook(void (pos_correction_imu_post_hook)(float dt)) {
	m_pos_correction_imu_post_hook = pos_correction_imu_post_hook;
}

void pos_set_correction_gnss_hook(void (pos_correction_gnss_hook)(POS_STATE *pos, float dt)) {
	m_pos_correction_gnss_hook = pos_correction_gnss_hook;
}
