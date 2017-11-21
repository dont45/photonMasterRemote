/*
  @file     sensor.h
  @author   Don Thompson (raynham Engineering)
  @license  GNU (see license.txt)

  Sensor class manages the sensors defined in the device list
  Copyright (C) 2016 Don Thompson, Raynham Engineering

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
    trippedSensor = FALSE;
  }
  bool readSensor(device_t &d);
  void setDeviceActive(device_t *d, bool);
  void setSensorIndicator(device_t d, uint8_t val);
  float readTemperature();
  float getLastTemperature();
  void clearChangedSensors();
  void addChangedSensor(device_t);
  bool hasChangedSensor();
  bool hasTrippedSensor();
  //xxx getChangedSensors();
  String fmtChangedSensors();

  char* romFormat(char *buf, uint8_t rom[]);
private:
  OW *p_ow;
  std::list<device_t> changedList;  //sensor has changed state since last read
  float last_temp;
  float last_oil_level;
  bool trippedSensor;
};
#endif
