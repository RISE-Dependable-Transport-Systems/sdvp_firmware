/*
 * time_today.h
 *
 *  Created on: 22 juli 2020
 *      Author: marvind
 */

#ifndef TIME_TODAY_H_
#define TIME_TODAY_H_

#include <stdint.h>

void time_today_set_ms(int32_t ms_today);
int32_t time_today_get_ms(void);
void time_today_set_pps_time_ref(int32_t pps_time_ref);
void time_today_pps_cb(void *arg);
int32_t time_today_get_pps_cnt(void);

#endif /* TIME_TODAY_H_ */
