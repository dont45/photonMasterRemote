/*
  sensor.h
  sensors
  State class manages system status display (LEDs)
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

#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "application.h"
#include "util.h"
#include "parms.h"
#include "OW.h"

//sensor devices will all be loaded from EEPROM
//and added to a std::list
//1-wire devices are checked for presence
class Sensor {
public:
  Sensor(OW *ow) {
    // Move this to function in configuration read
    p_ow = ow;
    pinMode(LOOP_SENSOR_PIN, INPUT);
    pinMode(MOTION_LOOP_SENSOR_PIN,INPUT);
    pinMode(TAMPER_PIN,INPUT);
    pinMode(MESSAGE_PIN,OUTPUT);
  }
  bool readSensor(device_t &d);
  void setSensorIndicator(device_t d, uint8_t val);
  float readTemperature();
  float getLastTemperature();

  char* romFormat(char *buf, uint8_t rom[]);
private:
  OW *p_ow;
  float last_temp;
  float last_oil_level;
};
#endif
