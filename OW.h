/*
  @file     ow.h
  @author   Don Thompson (raynham Engineering)
  @license  GNU (see license.txt)

  OW (1-WIRE) library for Arduino
  Copyright (C) 2009 Don Thompson, Raynham Engineering

  Raynham MA

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

#ifndef __OW_H__
#define __OW_H__

#define SERIAL_DEBUG
//#define SERIAL_DEBUG_OW
//#define SERIAL_DEBUG_THERM
//#include <inttypes.h>
#include "DS2482.h"
// move these to an include ??
#define FMLY_2405 0x05
#define FMLY_2406 0x12
#define FMLY_2413 0x3a
#define INCL_FMLY_2405
#define INCL_FMLY_2406
#define INCL_FMLY_2413


/* Need to have shift value defined for A and B
 * by device type:
 * 2406: A=2, B=3
 * 2413: A=0, B=2
 *
 */
#define DS2406_PIOA_SENSE 	(1<<2)
#define DS2406_PIOB_SENSE 	(1<<3)
/* DS2406 CONTROL BYTE
| BIT 7  | BIT 6  | BIT 5 | BIT 4 | BIT 3  | BIT 2  | BIT 1 | BIT 0 |
| Supply | No Ch. | PIOB  | PIOA  |  PIOB  | PIOA   | PIOB  | PIOA  |
| Ind.   |        | Latch | Latch | Sensed | Sensed | F-F   | F-F   |
*/

#define DS2413_PIOA_SENSE 	1
#define DS2413_PIOB_SENSE 	(1<<2)
/* DS2413 CONTROL byte
|   BITS 7 .. 4              | BIT 3     | BIT 2     | BIT 1     | BIT 0    |
|                            | PIOB OUT  | PIOB PIN  | PIOA OUT  | PIOA PIN |
| complement of low bits     | latch     | state     | latch     | state    |
*/
class OW : public DS2482
{
public:
    OW(uint8_t address): DS2482(address){ status_memory_7 = 0x1F; };

    uint8_t readStatus(uint8_t *ROM, uint8_t *);		// read status bytes of ds2406
    uint8_t writeStatus(uint8_t *ROM, uint8_t);
    uint8_t readChannel(uint8_t *ROM, uint8_t *);		// read status bytes of ds2406
    uint8_t channelAccess(uint8_t *ROM, uint8_t channel_control, uint8_t *buf);
    bool readThermometer(uint8_t *ROM, double &rTempF);
    uint8_t readPIO(uint8_t *ROM);		// read PIO state (switch family) (readdevPort ???;;;;)
    uint8_t readPIOA(uint8_t *ROM);
    uint8_t readPIOB(uint8_t *ROM);
    uint8_t readPIOX(uint8_t *ROM, uint8_t port);  //THIS WILL BECOME readPIO
    uint8_t writePIO(uint8_t *ROM, uint8_t, uint8_t );
    //void write8bits(uint8_t val);  //REMOVE
    // ADD writePIOA and B ??
    uint8_t writePIOtest(uint8_t *ROM, uint8_t port, uint8_t val);
    uint8_t condSearch(uint8_t *);
    const uint8_t sense_port_shift[2][2] = {{2,3},{0,2}};
private:
    uint8_t status_memory_7;
};


#endif
