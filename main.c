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
#include "chprintf.h"
#include "bmi160_wrapper.h"

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

static void dump_imu(float *accel, float *gyro, float *mag) {
	chprintf((BaseSequentialStream *)&PORTAB_SDU1, "### (%f, %f, %f)  - (%f, %f, %f)  - (%f, %f, %f) ###", accel[0], accel[1], accel[2],
																									       gyro[0], gyro[1], gyro[2],
																									       mag[0], mag[1], mag[2]);
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
  
  /*
   * Default values for GPIO
   */
  palWriteLine(LINE_LED_GREEN, 0);
  palWriteLine(LINE_LED_RED, 0);
  
  usbSerialInit();
  while (PORTAB_SDU1.config->usbp->state != USB_ACTIVE) {
      palWriteLine(LINE_LED_RED, 1);
      chThdSleepMilliseconds(100);
  }
  palWriteLine(LINE_LED_RED, 0); // USB-Serial connection is set up
  comm_serial_init((BaseSequentialStream *)&PORTAB_SDU1);

  bmi160_wrapper_init(2);
  bmi160_wrapper_set_read_callback(dump_imu);

  /*
   * main program loop
   */
  while (true) {
	static int i = 0;
	i++;

	if (i % 50 == 0)
		palToggleLine(LINE_LED_GREEN);

	// packet timeout
    packet_timerfunc();

    chThdSleepMilliseconds(10);
  }
}
