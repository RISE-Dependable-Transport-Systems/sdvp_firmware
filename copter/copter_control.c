/*
	Copyright 2017 Benjamin Vedder	benjamin@vedder.se

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

#include "copter_control.h"
#include "pos.h"
#include "utils.h"
#include "actuator.h"
#include "time_today.h"

#include <math.h>

// Settings
#define INPUT_TIMEOUT_MS				1000
#define POWER_OVERRIDE_TIMEOUT_MS		500
#define AUTOPILOT_TIMEOUT_MS			1000
#define MIN_THROTTLE					0.1
#define THROTTLE_OVERRIDE_LIM			0.85

// Private types
typedef struct {
	float roll_goal;
	float pitch_goal;
	float yaw_goal;
	float roll_integrator;
	float pitch_integrator;
	float yaw_integrator;
	float last_roll_process;
	float last_roll_error;
	float last_pitch_process;
	float last_pitch_error;
	float last_yaw_process;
	float last_yaw_error;
	systime_t gnss_update_time;
} MR_CONTROL_STATE;

typedef struct {
	float roll;
	float pitch;
	float yaw;
	float throttle;
	systime_t last_update_time;
} MR_RC_STATE;

typedef struct {
	float throttle;
	float roll;
	float pitch;
	float yaw;
} MR_OUTPUT;

// Private variables
static MR_CONTROL_STATE m_ctrl;
static MR_RC_STATE m_rc;
static POS_STATE m_pos_last;
static MR_OUTPUT m_output;
static float m_power_override[4];
static float m_power_override_time;

// Private functions
static void update_rc_control(MR_CONTROL_STATE *ctrl, MR_RC_STATE *rc, POS_STATE *pos, float dt);

void copter_control_init(void) {
	memset(&m_rc, 0, sizeof(MR_RC_STATE));
	memset(&m_ctrl, 0, sizeof(MR_CONTROL_STATE));
	memset(&m_pos_last, 0, sizeof(POS_STATE));
	memset(&m_output, 0, sizeof(MR_OUTPUT));
	memset(m_power_override, 0, sizeof(m_power_override));
	m_power_override_time = 0.0;
}

systime_t copter_control_time_since_input_update(void) {
	return TIME_I2MS(chVTTimeElapsedSinceX(m_rc.last_update_time));
}

void copter_control_set_input(float throttle, float roll, float pitch, float yaw) {
	m_rc.roll = roll;
	m_rc.pitch = pitch;
	m_rc.yaw = yaw;
	m_rc.throttle = throttle;
	m_rc.last_update_time = chVTGetSystemTimeX();
}

void copter_control_set_motor_override(int motor, float power) {
	if (motor >= 0 && motor < 4) {
		m_power_override[motor] = power;
		m_power_override_time = (float)POWER_OVERRIDE_TIMEOUT_MS / 1000.0;
	}
}

bool copter_control_is_throttle_over_tres(void) {
	return m_output.throttle > MIN_THROTTLE;
}

void copter_control_run_iteration(float dt) {
	bool lost_signal = true;

	pos_get(&m_pos_last);

	// require rc input and GNSS fix, velocity integration drift (from copter_control_pos_correction_imu) is unbounded otherwise
	if (copter_control_time_since_input_update() < INPUT_TIMEOUT_MS && TIME_I2MS(chVTTimeElapsedSinceX(m_ctrl.gnss_update_time)) < INPUT_TIMEOUT_MS) {
		lost_signal = false;
	} else {
		memset(&m_rc, 0, sizeof(MR_RC_STATE));
	}

	if (m_power_override_time > 0.0) {
		m_power_override_time -= dt;

		for (int i = 0;i < 4;i++) {
			actuator_set_motor(i, m_power_override[i]);
		}

		if (m_power_override_time <= 0.0) {
			memset(m_power_override, 0, sizeof(m_power_override));
		}

		return;
	}

	memset(&m_output, 0, sizeof(MR_OUTPUT));

	if (!lost_signal) {
		// Set roll, pitch and yaw goals to the current value if the throttle
		// just got above the threshold.
		static int was_below_tres = 0;
		if (m_rc.throttle < MIN_THROTTLE) {
			was_below_tres = 1;
			actuator_set_motor(-1, 0.0);
			return;
		} else {
			if (was_below_tres) {
				m_ctrl.roll_goal = m_pos_last.roll;
				m_ctrl.pitch_goal = m_pos_last.pitch;
				m_ctrl.yaw_goal = m_pos_last.yaw;
				m_ctrl.roll_integrator = 0.0;
				m_ctrl.pitch_integrator = 0.0;
				m_ctrl.yaw_integrator = 0.0;
			}

			was_below_tres = 0;
		}

		if (m_rc.throttle < THROTTLE_OVERRIDE_LIM || 1) {
			// Manual control
			update_rc_control(&m_ctrl, &m_rc, &m_pos_last, dt);
			m_output.throttle = m_rc.throttle;
		} else {
			// TODO: autopilot
		}

		// Run attitude control
		float roll_error = utils_angle_difference(m_ctrl.roll_goal, m_pos_last.roll);
		float pitch_error = utils_angle_difference(m_ctrl.pitch_goal, m_pos_last.pitch);
		float yaw_error = utils_angle_difference(m_ctrl.yaw_goal, m_pos_last.yaw);

		roll_error /= 360.0;
		pitch_error /= 360.0;
		yaw_error /= 360.0;

		// Run integration
		m_ctrl.roll_integrator += roll_error * dt;
		m_ctrl.pitch_integrator += pitch_error * dt;
		m_ctrl.yaw_integrator += yaw_error * dt;
		// Prevent wind-up
		utils_truncate_number_abs(&m_ctrl.roll_integrator, 1.0 / main_config.mr.ctrl_gain_roll_i);
		utils_truncate_number_abs(&m_ctrl.pitch_integrator, 1.0 / main_config.mr.ctrl_gain_pitch_i);
		utils_truncate_number_abs(&m_ctrl.yaw_integrator, 1.0 / main_config.mr.ctrl_gain_yaw_i);

		float d_roll_sample_process = -utils_angle_difference(m_pos_last.roll, m_ctrl.last_roll_process) / dt;
		float d_roll_sample_error = (roll_error - m_ctrl.last_roll_error) / dt;
		float d_pitch_sample_process = -utils_angle_difference(m_pos_last.pitch, m_ctrl.last_pitch_process) / dt;
		float d_pitch_sample_error = (pitch_error - m_ctrl.last_pitch_error) / dt;
		float d_yaw_sample_process = -utils_angle_difference(m_pos_last.yaw, m_ctrl.last_yaw_process) / dt;
		float d_yaw_sample_error = (yaw_error - m_ctrl.last_yaw_error) / dt;

		m_ctrl.last_roll_process = m_pos_last.roll;
		m_ctrl.last_roll_error = roll_error;
		m_ctrl.last_pitch_process = m_pos_last.pitch;
		m_ctrl.last_pitch_error = pitch_error;
		m_ctrl.last_yaw_process = m_pos_last.yaw;
		m_ctrl.last_yaw_error = yaw_error;

		d_roll_sample_process /= 360.0;
		d_pitch_sample_process /= 360.0;
		d_yaw_sample_process /= 360.0;

		m_output.roll = roll_error * main_config.mr.ctrl_gain_roll_p +
				m_ctrl.roll_integrator * main_config.mr.ctrl_gain_roll_i +
				d_roll_sample_process * main_config.mr.ctrl_gain_roll_dp +
				d_roll_sample_error * main_config.mr.ctrl_gain_roll_de;
		m_output.pitch = pitch_error * main_config.mr.ctrl_gain_pitch_p +
				m_ctrl.pitch_integrator * main_config.mr.ctrl_gain_pitch_i +
				d_pitch_sample_process * main_config.mr.ctrl_gain_pitch_dp +
				d_pitch_sample_error * main_config.mr.ctrl_gain_pitch_de;
		m_output.yaw = yaw_error * main_config.mr.ctrl_gain_yaw_p +
				m_ctrl.yaw_integrator * main_config.mr.ctrl_gain_yaw_i +
				d_yaw_sample_process * main_config.mr.ctrl_gain_yaw_dp +
				d_yaw_sample_error * main_config.mr.ctrl_gain_yaw_de;

		utils_truncate_number_abs(&m_output.roll, 1.0);
		utils_truncate_number_abs(&m_output.pitch, 1.0);
		utils_truncate_number_abs(&m_output.yaw, 1.0);

		// Compensate throttle for roll and pitch
		const float tan_roll = tanf(m_pos_last.roll * M_PI / 180.0);
		const float tan_pitch = tanf(m_pos_last.pitch * M_PI / 180.0);
		const float tilt_comp_factor = sqrtf(tan_roll * tan_roll + tan_pitch * tan_pitch + 1);

		m_output.throttle *= tilt_comp_factor;
		utils_truncate_number(&m_output.throttle, 0.0, 1.0);
	}

	actuator_set_output(m_output.throttle, m_output.roll, m_output.pitch, m_output.yaw);
}

static void update_rc_control(MR_CONTROL_STATE *ctrl, MR_RC_STATE *rc, POS_STATE *pos, float dt) {
	if (fabsf(rc->yaw) < 0.05) {
		rc->yaw = 0.0;
	}

	float roll = ctrl->roll_goal + rc->roll * main_config.mr.js_gain_tilt * dt * 250.0;
	float pitch = ctrl->pitch_goal + rc->pitch * main_config.mr.js_gain_tilt * dt * 250.0;
	float yaw = ctrl->yaw_goal + rc->yaw * main_config.mr.js_gain_yaw * dt * 250.0;

	if (main_config.mr.js_mode_rate) {
		// Integrate towards the current angle slowly
		roll = utils_weight_angle(roll, pos->roll, 0.995);
		pitch = utils_weight_angle(pitch, pos->pitch, 0.995);
	} else {
		// A trick!
		roll = utils_weight_angle(roll, -pos->roll, 0.995);
		pitch = utils_weight_angle(pitch, -pos->pitch, 0.995);
	}

	utils_norm_angle(&roll);
	utils_norm_angle(&pitch);
	utils_norm_angle(&yaw);

	ctrl->roll_goal = roll;
	ctrl->pitch_goal = pitch;
	ctrl->yaw_goal = yaw;
}

void copter_control_pos_correction_gnss(POS_STATE *pos, float dt) {
	if (dt > 2.0 || dt < 0.01)
		return;

	// Velocity
	const float dt_gps = (pos->gps_ms - pos->gps_ms_last) / 1000.0;
	const float vx_gps = (pos->px_gps - pos->px_gps_last) / dt_gps;
	const float vy_gps = (pos->py_gps - pos->py_gps_last) / dt_gps;
	float error_vx = pos->vx - vx_gps;
	float error_vy = pos->vy - vy_gps;

	utils_truncate_number_abs(&error_vx, main_config.mr.max_corr_error);
	utils_truncate_number_abs(&error_vy, main_config.mr.max_corr_error);

	const float error_vx_diff = (error_vx - pos->error_vx_last);
	const float error_vy_diff = (error_vy - pos->error_vy_last);
	pos->error_vx_last = error_vx;
	pos->error_vy_last = error_vy;

	const float vcx_p = error_vx * main_config.mr.vel_gain_p;
	const float vcy_p = error_vy * main_config.mr.vel_gain_p;

	pos->vel_corr_x_int += error_vx * main_config.mr.vel_gain_i * dt;
	pos->vel_corr_y_int += error_vy * main_config.mr.vel_gain_i * dt;
	utils_truncate_number(&pos->vel_corr_x_int, -1.0, 1.0);
	utils_truncate_number(&pos->vel_corr_y_int, -1.0, 1.0);

	const float vcx_d = error_vx_diff * main_config.mr.vel_gain_d / dt;
	const float vcy_d = error_vy_diff * main_config.mr.vel_gain_d / dt;

	const float vcx_out = vcx_p + pos->vel_corr_x_int + vcx_d;
	const float vcy_out = vcy_p + pos->vel_corr_y_int + vcy_d;

	pos->vx -= vcx_out;
	pos->vy -= vcy_out;

	// Tilt
	const float acx_p = error_vx * main_config.mr.tilt_gain_p;
	const float acy_p = error_vy * main_config.mr.tilt_gain_p;

	pos->tilt_corr_x_int += error_vx * main_config.mr.tilt_gain_i * dt;
	pos->tilt_corr_y_int += error_vy * main_config.mr.tilt_gain_i * dt;
	utils_truncate_number(&pos->tilt_corr_x_int, -1.0, 1.0);
	utils_truncate_number(&pos->tilt_corr_y_int, -1.0, 1.0);

	const float acx_d = error_vx_diff * main_config.mr.tilt_gain_d / dt;
	const float acy_d = error_vy_diff * main_config.mr.tilt_gain_d / dt;

	const float acx_out = acx_p + pos->tilt_corr_x_int + acx_d;
	const float acy_out = acy_p + pos->tilt_corr_y_int + acy_d;

	const float cosy = cosf(-pos->yaw * M_PI / 180.0);
	const float siny = sinf(-pos->yaw * M_PI / 180.0);

	pos->tilt_pitch_err += acx_out * cosy + acy_out * siny;
	pos->tilt_roll_err += acy_out * cosy - acx_out * siny;

	utils_truncate_number_abs(&pos->tilt_roll_err, main_config.mr.max_tilt_error);
	utils_truncate_number_abs(&pos->tilt_pitch_err, main_config.mr.max_tilt_error);

	if (!copter_control_is_throttle_over_tres())
		pos->gps_ground_level = pos->pz_gps;

	m_ctrl.gnss_update_time = chVTGetSystemTimeX();
}

void copter_control_pos_correction_imu(POS_STATE *pos, float dt) {
	if (!copter_control_is_throttle_over_tres())
		return;

	float roll = pos->roll + pos->tilt_roll_err;
	float pitch = pos->pitch + pos->tilt_pitch_err;
	float yaw = pos->yaw;

	// Too much tilt means that this won't work anyway. Return in that case.
	if (fabsf(roll) > 45.0 || fabsf(pitch) > 45.0) {
		pos->vx = 0;
		pos->vy = 0;
		return;
	}

	roll = roll * M_PI / 180.0;
	pitch = pitch * M_PI / 180.0;
	yaw = yaw * M_PI / 180.0;

	const float acc_v = 9.82;
	const float cos_y = cosf(-yaw);
	const float sin_y = sinf(-yaw);

	const float dvx = -acc_v * tanf(pitch) * dt;
	const float dvy = -acc_v * tanf(roll) * dt;

	pos->vx += cos_y * dvx - sin_y * dvy;
	pos->vy += cos_y * dvy + sin_y * dvx;
	pos->px += pos->vx * dt;
	pos->py += pos->vy * dt;

	// Apply position and velocity limits
	if (utils_truncate_number(&pos->px, main_config.mr.map_min_x, main_config.mr.map_max_x)) {
		pos->vx = 0.0;
	} else {
		utils_truncate_number_abs(&pos->vx, main_config.mr.vel_max);
	}

	if (utils_truncate_number(&pos->py, main_config.mr.map_min_y, main_config.mr.map_max_y)) {
		pos->vy = 0;
	} else {
		utils_truncate_number_abs(&pos->vy, main_config.mr.vel_max);
	}

	// Exponential decay
	const float decay_factor = powf(main_config.mr.vel_decay_e, dt);
	pos->vx *= decay_factor;
	pos->vy *= decay_factor;

	// Linear decay
	utils_step_towards(&pos->vx, 0.0, main_config.mr.vel_decay_l * dt);
	utils_step_towards(&pos->vy, 0.0, main_config.mr.vel_decay_l * dt);

	// Update speed sum
	pos->speed = sqrtf(SQ(pos->vx) + SQ(pos->vy));
}
