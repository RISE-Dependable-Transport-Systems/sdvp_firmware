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

// Settings
#define SERVO_OUT_PULSE_MIN_US		1000 // TODO -> main_config
#define SERVO_OUT_PULSE_MAX_US		2000
#define SERVO_UPDATE_RATE			200	// Hz
#define TIM_CLOCK				1000000 // Hz
#define ALL_CHANNELS			0xFF

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

// TODO: figure out how to enable TIM9 on recent ChibiOS
//static PWMConfig pwmcfg9 = {
//		TIM_CLOCK,
//		(uint16_t)((uint32_t)TIM_CLOCK / (uint32_t)SERVO_UPDATE_RATE),
//		NULL,
//		{
//				{PWM_OUTPUT_DISABLED, NULL},
//				{PWM_OUTPUT_DISABLED, NULL},
//				{PWM_OUTPUT_DISABLED, NULL},
//				{PWM_OUTPUT_DISABLED, NULL}
//		},
//		0,
//		0,
//#if STM32_PWM_USE_ADVANCED
//		0
//#endif
//};

void servo_pwm_init(uint8_t servo_enable_mask) {
	if (servo_enable_mask & (1 << 0))
		pwmcfg3.channels[2].mode = PWM_OUTPUT_ACTIVE_HIGH;

	if (servo_enable_mask & (1 << 1))
		pwmcfg3.channels[3].mode = PWM_OUTPUT_ACTIVE_HIGH;

//	if (servo_enable_mask & (1 << 2))
//		pwmcfg9.channels[0].mode = PWM_OUTPUT_ACTIVE_HIGH;
//
//	if (servo_enable_mask & (1 << 3))
//		pwmcfg9.channels[1].mode = PWM_OUTPUT_ACTIVE_HIGH;

	pwmStart(&PWMD3, &pwmcfg3);
//	pwmStart(&PWMD9, &pwmcfg9);
}

void servo_pwm_set_all(float pulse_width) {
	servo_pwm_set(ALL_CHANNELS, pulse_width);
}

/**
 * Set output pulsewidth.
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
	uint32_t cnt_val;

	if (0) {
		// Always set zero if emergency stop is set.
		// TODO: Implement this
		cnt_val = ((TIM_CLOCK / 1e3) * (uint32_t)SERVO_OUT_PULSE_MIN_US) / 1e3;
	} else {
		cnt_val = ((TIM_CLOCK / 1e3) * (uint32_t)SERVO_OUT_PULSE_MIN_US) / 1e3 +
				((TIM_CLOCK / 1e3) * (uint32_t)(pulse_width * (float)(SERVO_OUT_PULSE_MAX_US -
						SERVO_OUT_PULSE_MIN_US))) / 1e3;
	}

	switch(channel) {
	case 0:
		pwmEnableChannel(&PWMD3, 2, cnt_val);
		break;

	case 1:
		pwmEnableChannel(&PWMD3, 3, cnt_val);
		break;

//	case 2:
//		pwmEnableChannel(&PWMD9, 0, cnt_val);
//		break;
//
//	case 3:
//		pwmEnableChannel(&PWMD9, 1, cnt_val);
//		break;

	case ALL_CHANNELS:
		pwmEnableChannel(&PWMD3, 2, cnt_val);
		pwmEnableChannel(&PWMD3, 3, cnt_val);
//		pwmEnableChannel(&PWMD9, 0, cnt_val);
//		pwmEnableChannel(&PWMD9, 1, cnt_val);
		break;

	default:
		break;
	}
}
