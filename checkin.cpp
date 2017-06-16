/*
  checkin.cpp
  user checkins
  user must checkin upon system request to ensure he is ok
  failure to checkin within defined window will result in alerts
  begin sent to all users in 'emergency' group (defined in Pushover console)
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

//todo: ADD COMMAND TO PUT CHECKIN ON HOLD FOR XX HOURS ??
// e.g. ALL IS WELL, I'M PLAYING GOLF WITH FRIENDS
#include "checkin.h"

Checkin::Checkin() {
  //                   22221111111111
  //hours:             321098765432109876543210
  hours_to_checkin = 0b000010000100010000000000;  //10am 2pm 7pm
  checkin_time = Time.now() + TIME_BETWEEN_CHECKIN;
  panic_time = 0;  // zero means NOT set
  suspend_until = 0;
  in_panic = 0;
}

//TODO: return true if checkin was on-time, else false
void Checkin::userCheckin() {
  checkin_time = Time.now() + TIME_BETWEEN_CHECKIN;
  panic_time = 0;
}
//check if hour h is set in bitmask
bool Checkin::checkinThisHour(uint8_t h) {
  unsigned long hours = (hours_to_checkin >> h);
  return hours & 1;
}
void Checkin::setCheckinTime(int t) {
  checkin_time = t;
  panic_time = 0;
}
void Checkin::setPanicTime(int t) {
  panic_time = t;
}

// set or clear this hour from hours_to_checkin
// set if positive, clear if negative
void Checkin::setCheckinHour(int h) {
  bool toSet = h > 0;
  h = abs(h);
  unsigned long setMask = 1 << h;
  if(toSet) {
     hours_to_checkin |= setMask;
  }
  else {  //clear specified hour
    setMask = ~setMask;
    hours_to_checkin &= setMask;
  }
}

bool Checkin::timeExpired() {
  if(!checkin_time) return FALSE;
  if(suspend_until > Time.now()) {
    checkin_time = suspend_until;
    return FALSE;
  } else suspend_until = 0;
  if((Time.now()) > checkin_time) { //give a couple of minutes grace (120 sec)
    //checkin_time = 0; //here?? NO
    return TRUE;
  }
  return FALSE;
}

bool Checkin::panicExpired() {
  if(!panic_time) return FALSE;
  if(Time.now() > panic_time) {
    //panic_time = 0; //here??
    return TRUE;
  }
  return FALSE;
}
void Checkin::setPanicMode() {
  in_panic++;
}
bool Checkin::inPanicMode() {
  return in_panic > EMERGENCY_NOTICE_MAX;  //allow two panic message sends
}

void Checkin::reset() {
  in_panic = 0;
}
//suspend further checkins up to 12 hours
void Checkin::setSuspended(uint8_t hrs) {
  if(hrs > 24) hrs = 0;
  suspend_until = Time.now() + 3600 * hrs;
}

int Checkin::getCheckinTime() {
  return checkin_time;
}
int Checkin::getPanicTime() {
  return panic_time;
}
String Checkin::showCheckinTime() {
  String nextAt("next checkin at: ");
  //Serial.print("next checkin at: ");
  if(checkin_time)
    //nextAt += Time.format(checkin_time, "%I:%M%p");  //03:21AM
    nextAt += Time.format(checkin_time, TIME_FORMAT_DEFAULT);  //03:21AM
  else
    nextAt += "not set";
  return nextAt;
}
String Checkin::showPanicTime() {
  String nextPt("panic at: ");
  //Serial.print("panic at: ");
  if(panic_time)
    nextPt += Time.format(panic_time, TIME_FORMAT_DEFAULT);  //03:21AM
  else
    nextPt += "not set";
  //Serial.println(nextPt);
  return nextPt;
}
//show hours_to_checkin as 7am,10am,8pm
String Checkin::showCheckinHours() {
  String chkHrs;
  String hr;
  String p;   //am or pm
  int ip = 0; //converted to 12 hr format
  unsigned long hours = hours_to_checkin;
  for(int i=0; i<24; i++) {
    if(hours & 1) {
      if(ip) chkHrs.concat(",");
      if(i<12) {
        p="am";
        if(i==0) ip = 12;
        else ip = i;
      } else {
        p = "pm";
        if(i==12) ip = i;
        else ip = i - 12;
      }
      hr = String::format("%d", ip);
      chkHrs.concat(hr);
      chkHrs.concat(p);
    }
    hours >>= 1;
  }
  return chkHrs;
}
