/*
  @file     alarm.h
  @author   Don Thompson (raynham Engineering)
  @license  GNU (see license.txt)

  Alarm class manages core alarm functions
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

#ifndef __ALARM_H__
#define __ALARM_H__
#include "application.h"
#include "parms.h"
#include <queue>
#include <list>
#include <map>

// Alarm states xxx  FIX:???
// need two states: alarm_state; DISARMED, armed
// and sensor_state: clear, tripped, notified
typedef enum {alarm_off=0,alarm_disarmed=1,alarm_armed=2,alarm_tripped=3,alarm_notifying=4,alarm_clearing=5,alarm_reported=6,remotes_enabled=7,remotes_disabled=8};
const char* alarm_state_name_def[9] = {"SYSTEM OFF", "DISARMED", "ARMED", "TRIPPED", "NOTIFY", "CLEAR", "REPORTED","REM-ENA","REM_DIS"};
const char alarm_state_short[9] = {'F','D','A','T','N','C','R','X','O'};
typedef enum {AT_HOME=0, AWAY=1};
const char* alarm_state_location_def[2] = {"HOME", "AWAY"};
typedef enum {LED_OFF, LED_SOLID, LED_SLOW, LED_FAST};

class Alarm {
public:
  Alarm() {
    prevTripState = FALSE;
    firstTrippedSensor=0;  //?? use tripedList
    clearing_cnt = 0;
    p_config = new config_t;
    priority = 0;
    ledStatusState = 0;
    ledStatusBlink = 0;                       // 0=solid,1=slow blink,2=fast blink
    ledStatusBlinkCount = 0;
  }
  ~Alarm();
  //old: set state explicitly, with rules ??
  bool setState(uint8_t new_state);
  //new: set state -- state machine
  int setState();
  uint8_t getState();
  //uint8_t getbaseState();
  String getStateDescription();
  String getStateDescription(uint8_t);
  bool readSavedState();
  bool writeSavedState();
  bool generateConfiguration();
  bool readConfiguration();
  bool writeTestConfiguration();
  bool updateConfiguration(device_t* dev, int n);
  uint8_t buildDeviceList();
  void setValidNonWire();
  void dump_device_list();
  bool validate_device_list();
  char* deviceListing(char *buf);
  char* thermometerListing(char *buf);
  char* eventListing(char *buf);
  uint8_t wiredeviceAquire();
  bool wiredeviceValidate(uint8_t *ROM);
  bool isTripped();
  bool isReported();
  bool prevTripped();
  device_t *firstTripped();
  String tripListString();
  uint8_t formatRemoteTrip(char*);
  bool isArmed();
  void doStatusBlink();
  void clearing_countup();
  void blinkTimeout();
  bool alarmNotifying();
  String getPendingMessage();
  uint8_t getPriority();
  bool checkSensors(bool);
  String getLastTemperature();  //testing
  bool clearSensorReported();
  bool allClear();
  int clearingCount();
  device_t* getDevice(uint8_t); //v0.1.6
  device_t* getDeviceByUse(uint8_t); //v0.1.6
  bool setLastRemote(uint8_t, float);
  bool setAlarmRemote(uint8_t, uint8_t, uint8_t, float);
  //void setDeviceActive(device_t *d, bool);
  void setDeviceAlertMin(device_t *d, int p3);
  void setDeviceAlertMax(device_t *d, int p3);
  float readTemperature(uint8_t); //read by device idx
  void setCurLocation(uint8_t);
  uint8_t getCurLocation();
  bool devIsTripped(int);
  void resetDevice(uint8_t);
  int addEvent(uint8_t);
  int ackEvent(int);
  int removeEvent(int);
  bool eventsClear();
  void setRemotes(uint8_t);
  void tellRemote(uint8_t, char);
#ifdef PHOTON_REMOTE
  void setLEDRemote(char);
#endif
private:
  void setStatusLED();
  config_t configuration;
  config_t *p_config;
  uint8_t curState;
  uint8_t baseState;  // {off,disarmed,armed}
  uint8_t curLocation;
  device_t device;      //who is this guy ????
  std::list<device_t> device_list;
  std::list<device_t> trippedList;
  std::map <int, uint8_t> event_list;  //<event_sequence, device.idx>
  int event_sequence;
  bool prevTripState;
  uint8_t firstTrippedSensor;
  int clearing_cnt;
  String message;  //??name
  uint8_t priority;
  String trippedString;
  int ledNoticeState;
  int ledStatusState;
  int ledStatusBlink;                       // 0=solid,1=slow blink,2=fast blink
  int ledStatusBlinkCount;
};
#endif
