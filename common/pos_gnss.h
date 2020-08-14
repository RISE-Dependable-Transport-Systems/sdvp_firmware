/*
	Copyright 2016 - 2018 Benjamin Vedder	benjamin@vedder.se
	          2020        Marvin Damschen	marvin.damschen@ri.se

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

#ifndef POS_GNSS_H_
#define POS_GNSS_H_

#include "datatypes.h"

// Functions
void pos_gnss_init(void);
void pos_gnss_get(GPS_STATE *p);
void pos_gnss_set_enu_ref(double lat, double lon, double height);
void pos_gnss_get_enu_ref(double *llh);
void pos_gnss_nmea_cb(const char *data);
void pos_gnss_input_rtcm3(const unsigned char *data, const unsigned int len);

#endif /* POS_GNSS_H_ */
