/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se
	          2020 Marvin Damschen	marvin.damschen@ri.se

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

#ifndef TIMEOUT_H_
#define TIMEOUT_H_

#include "ch.h"

// Functions
void timeout_init(systime_t timeoutms, void (*timeout_action_cb)(void), void (*timeout_reset_cb)(void));
void timeout_reset(void);

#endif /* TIMEOUT_H_ */
