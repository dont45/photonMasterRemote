/*
  hwd.cpp
  hardware watchdog timer
  Implemented for ST Microelectronics STWD100
  Don Thompson
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

#ifndef __HWD_H__
#define __HWD_H__

#define HWD_EN_PIN D5
#define HWD_WDI_PIN D6

class HWD {
public:
  HWD();
  void enable();
  void disable();
  void checkin();
};

#endif
