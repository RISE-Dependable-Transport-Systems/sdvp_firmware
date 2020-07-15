/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
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
#include "bmi160_wrapper.h"
#include "pos.h"
#include "servo_pwm.h"
#include "comm_can.h"
#include "ublox.h"

// see: USB_CDC in ChibiOS testhal
void usbSerialInit(void) {
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

  // car: init single servo (SERVO0), set to center
  servo_pwm_init(0b0001);
  servo_pwm_set(0, 0.5);

  comm_can_init();

  // Init positioning (pos), BMI160 IMU and u-blox GNSS (F9P)
  pos_init();
  bmi160_wrapper_init(500);
  bmi160_wrapper_set_read_callback(pos_imu_data_callback);
  palWriteLine(LINE_LED_RED, 1);
  ublox_init();
  palWriteLine(LINE_LED_RED, 0); // u-blox init done

  /*
   * main program loop
   */
  while (true) {
	static int i = 0;
	i++;

	if (i % 50 == 0)
		palToggleLine(LINE_LED_GREEN);

	// packet communication timeout
    packet_timerfunc();

    chThdSleepMilliseconds(10);
  }
}
