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

	This file uses code from RC_Controller by Benjamin Vedder (pos).
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
