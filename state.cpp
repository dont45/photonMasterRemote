/*
  state.cpp
  system states
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

#include "state.h"

State::State(){
    status = 0;
    ledNoticeState = 0;
    startTime = Time.now();
}

void State::sysState(uint8_t current) {
  int notificationLED;
  curState = current;
  switch(current) {
    case sys_starting:
    #ifdef SERIAL_DEBUG
        Serial.print("sys_starting ");
        Serial.println("SYSTEM_VERSION");
    #endif
        digitalWrite(SYSTEM_NOTIFICATION_PIN, LOW);
        digitalWrite(SYSTEM_STATUS_PIN, LOW);
        digitalWrite(TRIGGER_PIN, LOW);
        break;
    case sys_configure:
        //GREEN LED Off, RED ON
        #ifdef SERIAL_DEBUG
            Serial.println("sys_configure");
        #endif
        digitalWrite(SYSTEM_STATUS_PIN, HIGH);
        break;
    case sys_fail:
        //HANG with fast red (hardware) or green (configuration) blink
        #ifdef SERIAL_DEBUG
            Serial.print("sys_fail:");
            Serial.println(status,HEX);
        #endif
        if((status & fail_hardware) || (status & fail_ds2482))
          notificationLED = SYSTEM_STATUS_PIN;
        else
          notificationLED = SYSTEM_NOTIFICATION_PIN;
        while(1) {
          digitalWrite(notificationLED, ledNoticeState);
          if(++ledNoticeState > 1) ledNoticeState = 0;
          delay(200);
        }
        break;
    case sys_running:
        //green ON
        #ifdef SERIAL_DEBUG
            Serial.print("sys_running");
            Serial.println(SYSTEM_VERSION);
        #endif
        startTime = Time.now();  //?? in class initializer
        digitalWrite(SYSTEM_STATUS_PIN, LOW);
        digitalWrite(SYSTEM_NOTIFICATION_PIN, HIGH);
        break;
    #ifdef SERIAL_DEBUG
    default:
        Serial.print("sysState-invalid state:");
        Serial.println(curState);
    #endif
  }
}

void State::addStatus(uint8_t sbit) {
  status |= sbit;
}
uint8_t State::sysStatus() {
  return status;
}
int State::upTime() {
  return Time.now() - startTime;
}
//calculate and format system up-time as dd hh:mm
char* State::upTime(char* upTimeStr) {
  int now,uptime,uptimeH,uptimeM;
  int updays,uphours,upminutes,upseconds;

  //now = Time.now();
  //uptime = now-startTime;
  uptime = upTime();
  updays = uptime / SECS_IN_DAY;    //days
  uptimeH = uptime % SECS_IN_DAY;  //uptime hours (minus days)u
  uphours = uptimeH / SECS_IN_HOUR; //hours
  uptimeM = uptimeH % SECS_IN_HOUR;
  upminutes = uptimeM / SECS_IN_MINUTE;         //minutes
  upseconds = uptimeM % SECS_IN_MINUTE;
  sprintf(upTimeStr,"%2d %02d:%02d:%02d",updays,uphours,upminutes,upseconds);
  return upTimeStr;
}
