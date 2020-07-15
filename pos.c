#include "pos.h"

#include "ch.h"
#include "utils.h"
#include "conf_general.h"
#include "ahrs.h"
#include <math.h>

// Private variables
static ATTITUDE_INFO m_att;
static POS_STATE m_pos;
static mutex_t m_mutex_pos;
static bool m_attitude_init_done;
static float m_accel[3];
static float m_gyro[3];
static float m_mag[3];
static float m_mag_raw[3];
static float m_imu_yaw_offset;
static float m_yaw_imu_clamp;
static bool m_yaw_imu_clamp_set;

// Private functions
static void update_orientation_angles(float *accel, float *gyro, float *mag, float dt);

void pos_init(void) {
	ahrs_init_attitude_info(&m_att);
	m_attitude_init_done = false;
	memset(&m_pos, 0, sizeof(m_pos));
	m_imu_yaw_offset = 0.0;

	chMtxObjectInit(&m_mutex_pos);
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

void pos_get_pos(POS_STATE *p) {
	chMtxLock(&m_mutex_pos);
	*p = m_pos;
	chMtxUnlock(&m_mutex_pos);
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

void pos_input_nmea(const char *data) {
	// TODO
//	nmea_gga_info_t gga;
//	static nmea_gsv_info_t gpgsv;
//	static nmea_gsv_info_t glgsv;
//	int gga_res = utils_decode_nmea_gga(data, &gga);
//	int gpgsv_res = utils_decode_nmea_gsv("GP", data, &gpgsv);
//	int glgsv_res = utils_decode_nmea_gsv("GL", data, &glgsv);
//
//	if (gpgsv_res == 1) {
//		utils_sync_nmea_gsv_info(&m_gpgsv_last, &gpgsv);
//	}
//
//	if (glgsv_res == 1) {
//		utils_sync_nmea_gsv_info(&m_glgsv_last, &glgsv);
//	}
//
//	if (gga.t_tow >= 0) {
//		m_nma_last_time = gga.t_tow;
//
//#if !(UBLOX_EN && UBLOX_USE_PPS) && !GPS_EXT_PPS
//		m_ms_today = gga.t_tow;
//#endif
//	}
//
//	// Only use valid fixes
//	if (gga.fix_type == 1 || gga.fix_type == 2 || gga.fix_type == 4 || gga.fix_type == 5) {
//		// Convert llh to ecef
//		double sinp = sin(gga.lat * D_PI / D(180.0));
//		double cosp = cos(gga.lat * D_PI / D(180.0));
//		double sinl = sin(gga.lon * D_PI / D(180.0));
//		double cosl = cos(gga.lon * D_PI / D(180.0));
//		double e2 = FE_WGS84 * (D(2.0) - FE_WGS84);
//		double v = RE_WGS84 / sqrt(D(1.0) - e2 * sinp * sinp);
//
//		chMtxLock(&m_mutex_gps);
//
//		m_gps.lat = gga.lat;
//		m_gps.lon = gga.lon;
//		m_gps.height = gga.height;
//		m_gps.fix_type = gga.fix_type;
//		m_gps.sats = gga.n_sat;
//		m_gps.ms = gga.t_tow;
//		m_gps.x = (v + gga.height) * cosp * cosl;
//		m_gps.y = (v + gga.height) * cosp * sinl;
//		m_gps.z = (v * (D(1.0) - e2) + gga.height) * sinp;
//
//		// Continue if ENU frame is initialized
//		if (m_gps.local_init_done) {
//			float dx = (float)(m_gps.x - m_gps.ix);
//			float dy = (float)(m_gps.y - m_gps.iy);
//			float dz = (float)(m_gps.z - m_gps.iz);
//
//			m_gps.lx = m_gps.r1c1 * dx + m_gps.r1c2 * dy + m_gps.r1c3 * dz;
//			m_gps.ly = m_gps.r2c1 * dx + m_gps.r2c2 * dy + m_gps.r2c3 * dz;
//			m_gps.lz = m_gps.r3c1 * dx + m_gps.r3c2 * dy + m_gps.r3c3 * dz;
//
//			float px = m_gps.lx;
//			float py = m_gps.ly;
//
//			// Apply antenna offset
//			const float s_yaw = sinf(-m_pos.yaw * M_PI / 180.0);
//			const float c_yaw = cosf(-m_pos.yaw * M_PI / 180.0);
//			px -= c_yaw * main_config.gps_ant_x - s_yaw * main_config.gps_ant_y;
//			py -= s_yaw * main_config.gps_ant_x + c_yaw * main_config.gps_ant_y;
//
//			chMtxLock(&m_mutex_pos);
//
//			m_pos.px_gps_last = m_pos.px_gps;
//			m_pos.py_gps_last = m_pos.py_gps;
//			m_pos.pz_gps_last = m_pos.pz_gps;
//			m_pos.gps_ms_last = m_pos.gps_ms;
//
//			m_pos.px_gps = px;
//			m_pos.py_gps = py;
//			m_pos.pz_gps = m_gps.lz;
//			m_pos.gps_ms = m_gps.ms;
//			m_pos.gps_fix_type = m_gps.fix_type;
//
//			// Correct position
//			// Optionally require RTK and good ublox quality indication.
//			if (main_config.gps_comp &&
//					(!main_config.gps_req_rtk || (gga.fix_type == 4 || gga.fix_type == 5)) &&
//					(!main_config.gps_use_ubx_info || m_ubx_pos_valid)) {
//
//				m_pos.gps_last_corr_diff = sqrtf(SQ(m_pos.px - m_pos.px_gps) +
//						SQ(m_pos.py - m_pos.py_gps));
//
//				correct_pos_gps(&m_pos);
//				m_pos.gps_corr_time = chVTGetSystemTimeX();
//
//#if MAIN_MODE == MAIN_MODE_CAR
//				m_pos.pz = m_pos.pz_gps - m_pos.gps_ground_level;
//#elif MAIN_MODE == MAIN_MODE_MULTIROTOR
//				// Update height from GPS if ultrasound measurements haven't been received for a while
//				if (ST2MS(chVTTimeElapsedSinceX(m_pos.ultra_update_time)) > 250) {
//					m_pos.pz = m_pos.pz_gps - m_pos.gps_ground_level;
//				}
//#endif
//			}
//
//			m_pos.gps_corr_cnt = 0.0;
//
//			chMtxUnlock(&m_mutex_pos);
//		} else {
//			init_gps_local(&m_gps);
//			m_gps.local_init_done = true;
//		}
//
//		m_gps.update_time = chVTGetSystemTimeX();
//
//		chMtxUnlock(&m_mutex_gps);
//	}
//
//	return gga_res >= 0;
}
