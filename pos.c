#include "pos.h"

#include "ch.h"
#include "utils.h"
#include "ahrs.h"
#include <math.h>

// Private variables
static ATTITUDE_INFO m_att;
static POS_STATE m_pos;
static bool m_attitude_init_done;
static float m_accel[3];
static float m_gyro[3];
static float m_mag[3];
static float m_mag_raw[3];
static float m_imu_yaw_offset;
static mutex_t m_mutex_pos;

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
//	if (main_config.mag_comp) {
//		float mag_t[3];
//
//		mag_t[0] = mag[0] - main_config.mag_cal_cx;
//		mag_t[1] = mag[1] - main_config.mag_cal_cy;
//		mag_t[2] = mag[2] - main_config.mag_cal_cz;
//
//		mag[0] = main_config.mag_cal_xx * mag_t[0] + main_config.mag_cal_xy * mag_t[1] + main_config.mag_cal_xz * mag_t[2];
//		mag[1] = main_config.mag_cal_yx * mag_t[0] + main_config.mag_cal_yy * mag_t[1] + main_config.mag_cal_yz * mag_t[2];
//		mag[2] = main_config.mag_cal_zx * mag_t[0] + main_config.mag_cal_zy * mag_t[1] + main_config.mag_cal_zz * mag_t[2];
//	}

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

//	if (main_config.mag_use) {
//		static float yaw_now = 0;
//		static float yaw_imu_last = 0;
//
//		float yaw_imu_diff = utils_angle_difference_rad(yaw, yaw_imu_last);
//		yaw_imu_last = yaw;
//		yaw_now += yaw_imu_diff;
//
//		float diff = utils_angle_difference_rad(yaw_mag, yaw_now);
//		yaw_now += SIGN(diff) * main_config.yaw_mag_gain * M_PI / 180.0 * dt;
//		utils_norm_angle_rad(&yaw_now);
//
//		m_pos.yaw_imu = yaw_now * 180.0 / M_PI;
//	} else {
		m_pos.yaw_imu = yaw * 180.0 / M_PI;
//	}

	utils_norm_angle(&m_pos.yaw_imu);
	m_pos.yaw_rate = -m_gyro[2] * 180.0 / M_PI;

	// Correct yaw
//#if MAIN_MODE == MAIN_MODE_CAR
//	{
//		if (!m_yaw_imu_clamp_set) {
//			m_yaw_imu_clamp = m_pos.yaw_imu - m_imu_yaw_offset;
//			m_yaw_imu_clamp_set = true;
//		}
//
//		if (main_config.car.clamp_imu_yaw_stationary && fabsf(m_pos.speed) < 0.05) {
//			m_imu_yaw_offset = m_pos.yaw_imu - m_yaw_imu_clamp;
//		} else {
//			m_yaw_imu_clamp = m_pos.yaw_imu - m_imu_yaw_offset;
//		}
//	}
//
//	if (main_config.car.yaw_use_odometry) {
//		if (main_config.car.yaw_imu_gain > 1e-10) {
//			float ang_diff = utils_angle_difference(m_pos.yaw, m_pos.yaw_imu - m_imu_yaw_offset);
//
//			if (ang_diff > 1.2 * main_config.car.yaw_imu_gain) {
//				m_pos.yaw -= main_config.car.yaw_imu_gain;
//				utils_norm_angle(&m_pos.yaw);
//			} else if (ang_diff < -1.2 * main_config.car.yaw_imu_gain) {
//				m_pos.yaw += main_config.car.yaw_imu_gain;
//				utils_norm_angle(&m_pos.yaw);
//			} else {
//				m_pos.yaw -= ang_diff;
//				utils_norm_angle(&m_pos.yaw);
//			}
//		}
//	} else {
//		m_pos.yaw = m_pos.yaw_imu - m_imu_yaw_offset;
//		utils_norm_angle(&m_pos.yaw);
//	}
//#else
	m_pos.yaw = m_pos.yaw_imu - m_imu_yaw_offset;
	utils_norm_angle(&m_pos.yaw);
//#endif

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
