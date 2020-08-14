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
 */

#include "ch.h"
#include "hal.h"
#include "portab.h"
#include "usbcfg.h"
#include "comm_serial.h"
#include "packet.h"
#include "commands.h"
#include "terminal.h"
#include "chprintf.h"
#include "conf_general.h"
#include "log.h"
#include "time_today.h"
#include "bmi160_wrapper.h"
#include "pos.h"
#include "pos_mc.h"
#include "pos_imu.h"
#include "pos_gnss.h"
#include "servo_pwm.h"
#include "comm_can.h"
#include "bldc_interface.h"
#include "motor_sim.h"
#include "ublox.h"
#include "timeout.h"
#include "autopilot.h"

// see: USB_CDC in ChibiOS testhal
static void usbSerialInit(void) {
  /*
   * Initializes a serial-over-USB CDC driver.
   */
  sduObjectInit(&PORTAB_SDU1);
  sduStart(&PORTAB_SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);
}

static void timeout_stop_cb(void) {
  palWriteLine(LINE_LED_RED, 1);

  // stop bldc_interface, brake if in motion
  comm_can_set_vesc_id(ID_ALL);
  if (!main_config.car.disable_motor && bldc_interface_get_last_received_values().rpm > TIMEOUT_MIN_RPM_BRAKE)
    bldc_interface_set_current_safety_brake(40.0);
  else
    bldc_interface_safety_stop();

  // set servo_pwm to safe value
  servo_pwm_safety_stop();
}

static void timeout_reset_cb(void) {
  palWriteLine(LINE_LED_RED, 0);
  bldc_interface_reset_safety_stop();
  servo_pwm_reset_safety_stop();
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
  
  /*
   * Board-dependent initialization.
   */
  portab_setup();
  
  // Default values for GPIO
  palWriteLine(LINE_LED_GREEN, 0);
  palWriteLine(LINE_LED_RED, 0);

  usbSerialInit();
  while (PORTAB_SDU1.config->usbp->state != USB_ACTIVE) {
      palWriteLine(LINE_LED_RED, 1);
      chThdSleepMilliseconds(100);
  }
  palWriteLine(LINE_LED_RED, 0); // USB-Serial connection is set up
  comm_serial_init((BaseSequentialStream *)&PORTAB_SDU1);
  terminal_set_vprintf(&commands_vprintf);

  conf_general_init();

  // car: init single servo (SERVO0) incl. safe stop value, set to center
  servo_pwm_init(0b0001, 0.5);
  servo_pwm_set(0, 0.5);

  // init CAN communication (incl. VESC/bldc_interface)
  comm_can_init();

  // Init positioning (pos), BMI160 IMU and u-blox GNSS (F9P).
  // Set bldc_interface (Motor Controller) callback
  // pos input: IMU (500 Hz), GNSS (5 Hz), Motor Controller (50 Hz)
  pos_init();
  pos_mc_init();
  pos_imu_init();
  pos_gnss_init();
  bmi160_wrapper_init(500);
  bmi160_wrapper_set_read_callback(pos_imu_data_cb);
  palWriteLine(LINE_LED_RED, 1);
  ublox_init();
  ublox_set_nmea_callback(&pos_gnss_nmea_cb);
  palWriteLine(LINE_LED_RED, 0); // u-blox init done
  bldc_interface_set_rx_value_func(pos_mc_values_cb);

  // u-blox PPS callback for timekeeping
  palEnableLineEvent(LINE_UBX_PPS, PAL_EVENT_MODE_RISING_EDGE);
  palSetLineCallback(LINE_UBX_PPS, time_today_pps_cb, NULL);

  autopilot_init();

  log_init();
  log_set_rate(main_config.log_rate_hz);
  log_set_enabled(main_config.log_en);
  log_set_name(main_config.log_name);

  motor_sim_init();

  timeout_init(1000, timeout_stop_cb, timeout_reset_cb); // safety timeout

  /*
   * main program loop
   */
  while (true) {
	static unsigned int i = 0;
	i++;

	// visual alive signal
	if (i % 50 == 0)
		palToggleLine(LINE_LED_GREEN);

	// packet communication timeout
    packet_timerfunc();

    // poll motor controller info every 20 ms -> 50 Hz
    if (i % 2 == 0)
    	bldc_interface_get_values();

    chThdSleepMilliseconds(10);
  }
}
