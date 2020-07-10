/*
    Copyright 2012-2016 Benjamin Vedder	benjamin@vedder.se
              2020      Marvin Damschen marvin.damschen@ri.se        

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

#ifndef COMM_SERIAL_H_
#define COMM_SERIAL_H_

// Functions
void comm_serial_init(BaseSequentialStream *serialStream);
void comm_serial_send_packet(unsigned char *data, unsigned int len);

#endif /* COMM_SERIAL_H_ */
