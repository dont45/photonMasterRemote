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

Checkin::Checkin() {}
Checkin::Checkin(unsigned long hours) {
  //                     22221111111111
  //hours:               321098765432109876543210
  //hours_to_checkin = 0b000010000000000010000000;  //7am  7pm
  hours_to_checkin = hours;
  if(!hours_to_checkin)
    hours_to_checkin = 0b000010000000000010000000;  //10am 2pm 7pm

  checkin_time = Time.now() + TIME_BETWEEN_CHECKIN;
  panic_time = 0;  // zero means NOT set
  notice_time = 0;
  suspend_until = 0;
  in_panic = 0;
}

//TODO: return true if checkin was on-time, else false
void Checkin::userCheckin() {
  checkin_time = Time.now() + TIME_BETWEEN_CHECKIN;
  panic_time = 0;
  notice_time = 0;
}
//check if hour h is set in bitmask
bool Checkin::checkinThisHour(uint8_t h) {
  unsigned long hours = (hours_to_checkin >> h);
  return hours & 1;
}

void Checkin::setCheckinTime(int t) {
  checkin_time = t;
  panic_time = 0;
  notice_time = 0;
}
/*
void Checkin::setPanicTime(int t) {
  panic_time = t;
}
void Checkin::setNoticeTime(int t) {
  notice_time = t;
}
*/
void Checkin::setCheckinHours(unsigned int hours) {
  hours_to_checkin = hours;
}
// set or clear this hour from hours_to_checkin
// set if positive, clear if negative
void Checkin::addCheckinHour(int h) {
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
  if((Time.now()) > checkin_time) {
    checkin_time = Time.now() + TIME_BETWEEN_CHECKIN; //no more for now
    panic_time = Time.now() + TIME_TO_PANIC;   //5 min to do checkin
    return TRUE;
  }
  return FALSE;
}

//TODO: Once expired, send two fast 'last chance to checkin' messages
// before emertency alert ??
bool Checkin::panicExpired() {
  if(!panic_time) return FALSE;
  if(Time.now() > panic_time) {
    //moved from main
    panic_time = Time.now() + TIME_BETWEEN_CHECKIN; //no more for now
    notice_time = Time.now() + TIME_TO_NOTICE;   //2 min to NOTICE
    return TRUE;
  }
  return FALSE;
}
//send actual notice
bool Checkin::noticeExpired() {
  if(!notice_time) return FALSE;
  if(Time.now() > notice_time) {
    setPanicMode(); //count times panic sent, limited to 2??
    //ONLY setCheckinTime if NOT panicMode ??
    checkin_time = Time.now() + TIME_BETWEEN_CHECKIN; //no more for now
    panic_time = 0; //no more for now
    notice_time = 0;
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
  setCheckinTime(Time.now() + TIME_BETWEEN_CHECKIN);
}
//suspend further checkins up to 12 hours
void Checkin::setSuspended(uint8_t hrs) {
  if(hrs > 24) hrs = 0;
  suspend_until = Time.now() + 3600 * hrs;
}

unsigned long Checkin::getCheckinHours() {
  return hours_to_checkin;
}

//combine these three into ONE
String Checkin::showCheckinTime() {
  String nextAt("\\ncheckin_time ");
  //Serial.print("next checkin at: ");
  if(checkin_time)
    nextAt += Time.format(checkin_time, TIME_FORMAT_DEFAULT);  //03:21AM
  else
    nextAt += "not set";
    //nextAt = "";
  return nextAt;
}
String Checkin::showPanicTime() {
  String nextPt("\\npanic_time ");
  //Serial.print("panic at: ");
  if(panic_time)
    nextPt += Time.format(panic_time, TIME_FORMAT_DEFAULT);  //03:21AM
  else
    //nextPt += "not set";
    nextPt = "";
  return nextPt;
}
String Checkin::showNoticeTime() {
  String nextPt("\\nnotice_time ");
  //Serial.print("panic at: ");
  if(notice_time)
    nextPt += Time.format(notice_time, TIME_FORMAT_DEFAULT);  //03:21AM
  else
    //nextPt += "not set";
    nextPt = "";
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
