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

#include "servo_pwm.h"
#include "ch.h"
#include "hal.h"
#include "conf_general.h"
#include "utils.h"

// Settings
#define SERVO_OUT_PULSE_MIN_US		1000 // TODO -> main_config
#define SERVO_OUT_PULSE_MAX_US		2000
#define SERVO_UPDATE_RATE			200	// Hz
#define TIM_CLOCK				1000000 // Hz
#define ALL_CHANNELS			0xFF
#define RAMP_LOOP_HZ			50 // Hz

// Private variables
static bool m_safety_stop;
static float m_safe_stop_pulse_width;
static float m_pulse_width_ramp_towards[4] = {0.0, 0.0, 0.0, 0.0};
static float m_pulse_width_current[4] = {0.0, 0.0, 0.0, 0.0};
static THD_WORKING_AREA(ramp_thread_wa, 128);

// Private functions
static THD_FUNCTION(ramp_thread, arg);
static inline void __servo_pwm_set_io(uint8_t channel, float pulse_width);

static PWMConfig pwmcfg3 = {
		TIM_CLOCK,
		(uint16_t)((uint32_t)TIM_CLOCK / (uint32_t)SERVO_UPDATE_RATE),
		NULL,
		{
				{PWM_OUTPUT_DISABLED, NULL},
				{PWM_OUTPUT_DISABLED, NULL},
				{PWM_OUTPUT_DISABLED, NULL},
				{PWM_OUTPUT_DISABLED, NULL}
		},
		0,
		0,
#if STM32_PWM_USE_ADVANCED
		0
#endif
};

static PWMConfig pwmcfg9 = {
		TIM_CLOCK,
		(uint16_t)((uint32_t)TIM_CLOCK / (uint32_t)SERVO_UPDATE_RATE),
		NULL,
		{
				{PWM_OUTPUT_DISABLED, NULL},
				{PWM_OUTPUT_DISABLED, NULL},
				{PWM_OUTPUT_DISABLED, NULL},
				{PWM_OUTPUT_DISABLED, NULL}
		},
		0,
		0,
#if STM32_PWM_USE_ADVANCED
		0
#endif
};


void servo_pwm_init(uint8_t servo_enable_mask, float safe_stop_pulse_width) {
	m_safety_stop = false;
	m_safe_stop_pulse_width = safe_stop_pulse_width;

	if (servo_enable_mask & (1 << 0))
		pwmcfg3.channels[2].mode = PWM_OUTPUT_ACTIVE_HIGH;

	if (servo_enable_mask & (1 << 1))
		pwmcfg3.channels[3].mode = PWM_OUTPUT_ACTIVE_HIGH;

	if (servo_enable_mask & (1 << 2))
		pwmcfg9.channels[0].mode = PWM_OUTPUT_ACTIVE_HIGH;

	if (servo_enable_mask & (1 << 3))
		pwmcfg9.channels[1].mode = PWM_OUTPUT_ACTIVE_HIGH;

	pwmStart(&PWMD3, &pwmcfg3);
	pwmStart(&PWMD9, &pwmcfg9);

	chThdCreateStatic(ramp_thread_wa, sizeof(ramp_thread_wa), NORMALPRIO, ramp_thread, NULL);
}

void servo_pwm_set_all(float pulse_width) {
	servo_pwm_set(ALL_CHANNELS, pulse_width);
}

/**
 * Set output pulsewidth without ramping.
 *
 * @param channel
 * Channel to use
 * Range: [0 - 3]
 * 0xFF: All Channels
 *
 * @param pulse_width
 * Pulsewidth to use
 * Range: [0.0 - 1.0]
 *
 */
void servo_pwm_set(uint8_t channel, float pulse_width) {
	utils_truncate_number(&pulse_width, 0.0, 1.0);
	if (channel == ALL_CHANNELS) {
		m_pulse_width_ramp_towards[0] = pulse_width;
		m_pulse_width_ramp_towards[1] = pulse_width;
		m_pulse_width_ramp_towards[2] = pulse_width;
		m_pulse_width_ramp_towards[3] = pulse_width;
	} else
		m_pulse_width_ramp_towards[channel] = pulse_width;

	__servo_pwm_set_io(channel, pulse_width);
}

void __servo_pwm_set_io(uint8_t channel, float pulse_width) {
	uint32_t cnt_val;

	if (m_safety_stop) {
		return;
	} else {
		cnt_val = ((TIM_CLOCK / 1e3) * (uint32_t)SERVO_OUT_PULSE_MIN_US) / 1e3 +
				((TIM_CLOCK / 1e3) * (uint32_t)(pulse_width * (float)(SERVO_OUT_PULSE_MAX_US -
						SERVO_OUT_PULSE_MIN_US))) / 1e3;
	}

	switch(channel) {
	case 0:
		pwmEnableChannel(&PWMD3, 2, cnt_val);
		m_pulse_width_current[0] = pulse_width;
		break;

	case 1:
		pwmEnableChannel(&PWMD3, 3, cnt_val);
		m_pulse_width_current[1] = pulse_width;
		break;

	case 2:
		pwmEnableChannel(&PWMD9, 0, cnt_val);
		m_pulse_width_current[2] = pulse_width;
		break;

	case 3:
		pwmEnableChannel(&PWMD9, 1, cnt_val);
		m_pulse_width_current[3] = pulse_width;
		break;

	case ALL_CHANNELS:
		pwmEnableChannel(&PWMD3, 2, cnt_val);
		pwmEnableChannel(&PWMD3, 3, cnt_val);
		pwmEnableChannel(&PWMD9, 0, cnt_val);
		pwmEnableChannel(&PWMD9, 1, cnt_val);
		m_pulse_width_current[0] = pulse_width;
		m_pulse_width_current[1] = pulse_width;
		m_pulse_width_current[2] = pulse_width;
		m_pulse_width_current[3] = pulse_width;
		break;

	default:
		break;
	}
}

/**
 * Set output pulsewidth with ramping.
 *
 * @param channel
 * Channel to use
 * Range: [0 - 3]
 * 0xFF: All Channels
 *
 * @param pulse_width
 * Pulsewidth to use
 * Range: [0.0 - 1.0]
 *
 */
void servo_pwm_set_ramped(uint8_t channel, float pulse_width) {
	utils_truncate_number(&pulse_width, 0.0, 1.0);
	if (channel == ALL_CHANNELS) {
		m_pulse_width_ramp_towards[0] = pulse_width;
		m_pulse_width_ramp_towards[1] = pulse_width;
		m_pulse_width_ramp_towards[2] = pulse_width;
		m_pulse_width_ramp_towards[3] = pulse_width;
	} else
		m_pulse_width_ramp_towards[channel] = pulse_width;
}

float servo_pwm_get(uint8_t id) {
	if (id <= 3)
		return m_pulse_width_current[id];
	else
		return -1.0;
}

void servo_pwm_safety_stop(void) {
	servo_pwm_set_all(m_safe_stop_pulse_width);
	m_safety_stop = true;
}
void servo_pwm_reset_safety_stop(void) {
	m_safety_stop = false;
}

static THD_FUNCTION(ramp_thread, arg) {
	(void)arg;

	chRegSetThreadName("servo_pwm ramp");

	for(;;) {
		for (int i = 0; i < 4; i++) {
			const float pos_prev = m_pulse_width_current[i];
			utils_step_towards(&m_pulse_width_current[i], m_pulse_width_ramp_towards[i], 1.0 / ((float)RAMP_LOOP_HZ * main_config.car.steering_ramp_time));

			if (m_pulse_width_current[i] != pos_prev)
				__servo_pwm_set_io(i, m_pulse_width_current[i]);
		}

		chThdSleep(CH_CFG_ST_FREQUENCY / RAMP_LOOP_HZ);
	}
}
