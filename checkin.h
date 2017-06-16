/*
  @file     checkin.h
  @author   Don Thompson (raynham Engineering)
  @license  GNU (see license.txt)

  Alarm User check-in system
  Generate checkin-time events and generate no-checkin if user does not
  respond.
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

#ifndef __CHECKIN_H__
#define __CHECKIN_H__

#define TIME_BETWEEN_CHECKIN 3600 //MINIMUM TIME BETWEEN
#define RESPOND_TO_CHECKIN 1800   //MUSTe RESPOND WITHIN
#define EMERGENCY_NOTICE_MAX 2    //MAX TIMES EMERGENCY SENT 
#include "application.h"
#include "parms.h"

class Checkin {
public:
  Checkin();
  void setCheckinTime(int);
  void setPanicTime(int);
  void setPanicMode();
  bool inPanicMode();
  void userCheckin();     // perform user checkin
  bool checkinThisHour(uint8_t); // should we perform checkin, based on hour mask
  void setCheckinHour(int h);
  int getCheckinTime();
  int getPanicTime();
  bool timeExpired();     //has eheckin_time arrived?
  bool panicExpired();    //has panic_time arrived?
  void reset();           //reset panic state
  String showCheckinHours();
  void setSuspended(uint8_t); //suspend up to 24 hours
  //DEBUG
  String showCheckinTime();
  String showPanicTime();
private:
  unsigned long hours_to_checkin;  //bitmask of hours
  //zero means NOT set, else Time until event:
  int checkin_time; //time until checkin request sent
  int panic_time; //time to send to emergency list
  int suspend_until;  //suspend all checkins until this time (max xx hours?)
  uint8_t in_panic;   //times panic sent, limit to 2??
};
#endif
