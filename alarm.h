/*
  alarm.h
  Alarm class manages core alarm functions
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

#ifndef __ALARM_H__
#define __ALARM_H__
#include "application.h"
#include "parms.h"
#include <queue>
#include <list>

class Alarm {
public:
  Alarm() {
    prevTripState = FALSE;
    firstTrippedSensor=0;  //?? use tripedList
    clearing_cnt = 0;
    p_config = new config_t;
    priority = 0;
  }
  ~Alarm();
  bool setState(uint8_t new_state);
  uint8_t getState();
  bool readSavedState();
  bool writeSavedState();
  bool generateConfiguration();
  bool readConfiguration();
  bool writeTestConfiguration();
  bool updateConfiguration(device_t* dev, int n);
  uint8_t buildDeviceList();
  void dump_device_list();
  bool validate_device_list();
  char* deviceListing(char *buf);
  uint8_t wiredeviceAquire();
  bool wiredeviceValidate(uint8_t *ROM);
  bool isTripped();
  bool prevTripped();
  device_t *firstTripped();
  void tripListString();
  uint8_t formatRemoteTrip(char*);
  bool isArmed();
  void doStatusBlink();
  void clearing_countup();
  void blinkTimeout();
  bool alarmNotifying();
  String getPendingMessage();
  uint8_t getPriority();
  bool checkSensors(void);
  String getLastTemperature();  //testing
  bool clearSensorReported();
  bool allClear();
  int clearingCount();
  device_t* getDevice(uint8_t); //v0.1.6
  device_t* getDeviceByUse(uint8_t); //v0.1.6
  bool setLastRemote(uint8_t, float);
  bool setAlarmRemote(uint8_t, float);
  void setDeviceActive(device_t *d, bool);
  void setDeviceAlertMin(device_t *d, int p3);
  void setDeviceAlertMax(device_t *d, int p3);
private:
  config_t configuration;
  config_t *p_config;
  uint8_t curState;
  device_t device;      //who is this guy ????
  std::list<device_t> device_list;
  std::list<device_t> trippedList;
  bool prevTripState;
  uint8_t firstTrippedSensor;
  int clearing_cnt;
  String message;  //??name
  uint8_t priority;
  String trippedString;
};
#endif
