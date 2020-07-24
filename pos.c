#include "pos.h"

#include "ch.h"
#include "utils.h"
#include "commands.h" // TODO might make sense to factor out
#include "conf_general.h"
#include "time_today.h"
#include "ahrs.h"
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
static ATTITUDE_INFO m_att;
static mutex_t m_mutex_pos;
static bool m_en_delay_comp;
static bool m_gps_corr_print;
static bool m_pos_history_print;
// IMU
static bool m_attitude_init_done;
static float m_accel[3];
static float m_gyro[3];
static float m_mag[3];
static float m_mag_raw[3];
static float m_imu_yaw_offset;
static float m_yaw_imu_clamp;
static bool m_yaw_imu_clamp_set;
// Motor controller
static mc_values m_mc_val;


// Private functions
static void cmd_terminal_delay_info(int argc, const char **argv);
static void cmd_terminal_gps_corr_info(int argc, const char **argv);
static void cmd_terminal_delay_comp(int argc, const char **argv);
static void update_orientation_angles(float *accel, float *gyro, float *mag, float dt);
static void save_pos_history(void);
static POS_POINT get_closest_point_to_time(int32_t time);
static void car_update_pos(float distance, float turn_rad_rear, float angle_diff, float speed);

void pos_init(void) {
	ahrs_init_attitude_info(&m_att);
	m_attitude_init_done = false;
	memset(&m_pos, 0, sizeof(m_pos));
	memset(&m_mc_val, 0, sizeof(m_mc_val));
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

	terminal_register_command_callback(
			"pos_reset_att",
			"Re-initialize the attitude estimation",
			NULL,
			cmd_terminal_reset_attitude);
}

void pos_get_imu(float *accel, float *gyro, float *mag) {
	if (accel) {
		accel[0] = m_accel[0];
		accel[1] = m_accel[1];
		accel[2] = m_accel[2];
	}

	if (gyro) {
		gyro[0] = m_gyro[0];
		gyro[1] = m_gyro[1];
		gyro[2] = m_gyro[2];
	}

	if (mag) {
		mag[0] = m_mag_raw[0];
		mag[1] = m_mag_raw[1];
		mag[2] = m_mag_raw[2];
	}
}

void pos_get_pos(POS_STATE *p) {
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

void cmd_terminal_reset_attitude(int argc, const char **argv) {
	(void)argc;
	(void)argv;
	m_attitude_init_done = false;
	terminal_printf("OK");
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

void pos_imu_data_callback(float *accel, float *gyro, float *mag) {
	static systime_t time_last = 0;
	sysinterval_t time_elapsed = chVTTimeElapsedSinceX(time_last);
	time_last = chVTGetSystemTimeX();
	float dt = chTimeI2US(time_elapsed)/1.0e6;

	update_orientation_angles(accel, gyro, mag, dt);
}

static void update_orientation_angles(float *accel, float *gyro, float *mag, float dt) {
	gyro[0] = gyro[0] * M_PI / 180.0;
	gyro[1] = gyro[1] * M_PI / 180.0;
	gyro[2] = gyro[2] * M_PI / 180.0;

	m_mag_raw[0] = mag[0];
	m_mag_raw[1] = mag[1];
	m_mag_raw[2] = mag[2];

	/*
	 * Hard and soft iron compensation
	 *
	 * http://davidegironi.blogspot.it/2013/01/magnetometer-calibration-helper-01-for.html#.UriTqkMjulM
	 *
	 * xt_raw = x_raw - offsetx;
	 * yt_raw = y_raw - offsety;
	 * zt_raw = z_raw - offsetz;
	 * x_calibrated = scalefactor_x[1] * xt_raw + scalefactor_x[2] * yt_raw + scalefactor_x[3] * zt_raw;
	 * y_calibrated = scalefactor_y[1] * xt_raw + scalefactor_y[2] * yt_raw + scalefactor_y[3] * zt_raw;
	 * z_calibrated = scalefactor_z[1] * xt_raw + scalefactor_z[2] * yt_raw + scalefactor_z[3] * zt_raw;
	 */
	if (main_config.mag_comp) {
		float mag_t[3];

		mag_t[0] = mag[0] - main_config.mag_cal_cx;
		mag_t[1] = mag[1] - main_config.mag_cal_cy;
		mag_t[2] = mag[2] - main_config.mag_cal_cz;

		mag[0] = main_config.mag_cal_xx * mag_t[0] + main_config.mag_cal_xy * mag_t[1] + main_config.mag_cal_xz * mag_t[2];
		mag[1] = main_config.mag_cal_yx * mag_t[0] + main_config.mag_cal_yy * mag_t[1] + main_config.mag_cal_yz * mag_t[2];
		mag[2] = main_config.mag_cal_zx * mag_t[0] + main_config.mag_cal_zy * mag_t[1] + main_config.mag_cal_zz * mag_t[2];
	}

	// Swap mag X and Y to match the accelerometer
	{
		float tmp[3];
		tmp[0] = mag[1];
		tmp[1] = mag[0];
		tmp[2] = mag[2];
		mag[0] = tmp[0];
		mag[1] = tmp[1];
		mag[2] = tmp[2];
	}

	// Rotate board yaw orientation
	float rotf = 0.0;

#ifdef BOARD_YAW_ROT // TODO: -> main_config
	rotf += BOARD_YAW_ROT;
#endif

	rotf *= M_PI / 180.0;
	utils_norm_angle_rad(&rotf);

	float cRot = cosf(rotf);
	float sRot = sinf(rotf);

	m_accel[0] = cRot * accel[0] + sRot * accel[1];
	m_accel[1] = cRot * accel[1] - sRot * accel[0];
	m_accel[2] = accel[2];
	m_gyro[0] = cRot * gyro[0] + sRot * gyro[1];
	m_gyro[1] = cRot * gyro[1] - sRot * gyro[0];
	m_gyro[2] = gyro[2];
	m_mag[0] = cRot * mag[0] + sRot * mag[1];
	m_mag[1] = cRot * mag[1] - sRot * mag[0];
	m_mag[2] = mag[2];

	if (!m_attitude_init_done) {
		ahrs_update_initial_orientation(m_accel, m_mag, (ATTITUDE_INFO*)&m_att);
		m_attitude_init_done = true;
	} else {
		//		ahrs_update_mahony_imu(gyro, accel, dt, (ATTITUDE_INFO*)&m_att);
		ahrs_update_madgwick_imu(m_gyro, m_accel, dt, (ATTITUDE_INFO*)&m_att);
	}

	float roll = ahrs_get_roll((ATTITUDE_INFO*)&m_att);
	float pitch = ahrs_get_pitch((ATTITUDE_INFO*)&m_att);
	float yaw = ahrs_get_yaw((ATTITUDE_INFO*)&m_att);

	// Apply tilt compensation for magnetometer values and calculate magnetic
	// field angle. See:
	// https://cache.freescale.com/files/sensors/doc/app_note/AN4248.pdf
	// Notice that hard and soft iron compensation is applied above
	float mx = -m_mag[0];
	float my = m_mag[1];
	float mz = m_mag[2];

	float sr = sinf(roll);
	float cr = cosf(roll);
	float sp = sinf(pitch);
	float cp = cosf(pitch);

	float c_mx = mx * cp + my * sr * sp + mz * sp * cr;
	float c_my = my * cr - mz * sr;

	float yaw_mag = atan2f(-c_my, c_mx) - M_PI / 2.0;

	chMtxLock(&m_mutex_pos);

	m_pos.roll = roll * 180.0 / M_PI;
	m_pos.pitch = pitch * 180.0 / M_PI;
	m_pos.roll_rate = -m_gyro[0] * 180.0 / M_PI;
	m_pos.pitch_rate = m_gyro[1] * 180.0 / M_PI;

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
	m_pos.yaw_rate = -m_gyro[2] * 180.0 / M_PI;

	// Correct yaw
#if MAIN_MODE == MAIN_MODE_CAR
	{
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

	if (main_config.car.yaw_use_odometry) {
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
#else
	m_pos.yaw = m_pos.yaw_imu - m_imu_yaw_offset;
	utils_norm_angle(&m_pos.yaw);
#endif

	m_pos.q0 = m_att.q0;
	m_pos.q1 = m_att.q1;
	m_pos.q2 = m_att.q2;
	m_pos.q3 = m_att.q3;

	chMtxUnlock(&m_mutex_pos);
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


	chMtxUnlock(&m_mutex_pos);
}

void pos_mc_values_received(mc_values *val) {
	m_mc_val = *val;

	static float last_tacho = 0;
	static bool tacho_read = false;
	float tacho = m_mc_val.tachometer;
	float rpm = m_mc_val.rpm;

	// Reset tacho the first time.
	if (!tacho_read) {
		tacho_read = true;
		last_tacho = tacho;
	}

	float distance = (tacho - last_tacho) * main_config.car.gear_ratio
			* (2.0 / main_config.car.motor_poles) * (1.0 / 6.0)
			* main_config.car.wheel_diam * M_PI;
	last_tacho = tacho;

	float angle_diff = 0.0;
	float turn_rad_rear = 0.0;

	float steering_angle = (servo_pwm_get(0) // TODO: generalize
			- main_config.car.steering_center)
			* ((2.0 * main_config.car.steering_max_angle_rad)
					/ main_config.car.steering_range);

	if (fabsf(steering_angle) >= 1e-6) {
		turn_rad_rear = main_config.car.axis_distance / tanf(steering_angle);
		float turn_rad_front = sqrtf(
				main_config.car.axis_distance * main_config.car.axis_distance
				+ turn_rad_rear * turn_rad_rear);

		if (turn_rad_rear < 0) {
			turn_rad_front = -turn_rad_front;
		}

		angle_diff = (distance * 2.0) / (turn_rad_rear + turn_rad_front);
	}

	float speed = rpm * main_config.car.gear_ratio
			* (2.0 / main_config.car.motor_poles) * (1.0 / 60.0)
			* main_config.car.wheel_diam * M_PI;

	car_update_pos(distance, turn_rad_rear, angle_diff, speed);
}

static void car_update_pos(float distance, float turn_rad_rear, float angle_diff, float speed) {
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
