/*
  @file     util.h
  @author   Don Thompson (raynham Engineering)
  @license  GNU (see license.txt)

  utilities for Photon Alarm
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

  @section  HISTORY

*/

#ifndef __UTIL_H__
#define __UTIL_H__
#include "parms.h"

#define HALT while(1)
//TO DO: reorder these so > xx = always present ??
// dev_use >= SUB_REMOTE_THERMOMETER is REMOTE device
// check for DEV_PRESENT if idx < 10
typedef enum {UNDEFINED=0, USER_KEY=1, MASTER_KEY=2, OW_SENSOR=3, OW_INDICATOR=4, OW_RELAY=5, OW_THERMOMETER=6,
              MCP9808_THERMOMETER=7, SWITCH=8, IO_SENSOR=9, SUB_REMOTE_THERMOMETER=10, SUB_REMOTE_SENSOR=11, SUB_OIL_GAUGE=12, SUB_CAR_MONITOR=13, SUB_SET_DEVICE=14} SENSOR_TYPE;
#define FIRST_SUB_REMOTE 10
#define FIRST_NON_WIRE 7
enum {HOME_DEVICE=0, AWAY_DEVICE=1, ALWAYS_DEVICE=2} ;
enum {DEV_MISSING=0, DEV_PRESENT=1, DEV_ERROR_UNKNOWN};
enum {SENSOR_DISABLED=0, SENSOR_ACTIVE=1, SENSOR_INACTIVE=2};
enum {SENSOR_CLEAR, SENSOR_TRIPPED, SENSOR_RESET, SENSOR_STILL_TRIPPED};
enum {SENSE_NORMAL_CLOSED=0, SENSE_NORMAL_OPEN=1};
const char* sensor_status_def[3] = {"missing", "present", "unknown"};
const char* sensor_state_def[3] = {"deactivated", "active","inactive"};
const char* sensor_use_def[] = {"undefined","user key", "master key","ow switch","ow indicator","ow relay","ow thermometer",
          "mcp9808 therm","switch","port switch","remote thermometer","remote alarm", "oil level", "remote monitor", "remote set device"};
const char* sensor_sense_def[2] = {"normal closed", "normal open"};
const char* sensor_level_def[] = {"home", "away", "always"};
const char* sensor_use_descr[] = {"undefined","user-key","master-key","ow-sensor","ow-indicator","ow-relay","ow-thermometer",
              "mcp9808-thermometer","switch","hdw-port-sensor","sub-remote-therm","sub-remote-alarm","sub-oil-level","remote-car-monitor","remote-set-device" };
//device flag bit definitions
//YELLOW = caution state, i.e. oil level warning; RED = critical state, i.e. OIL critically low
enum {DEVICE_PRIORITY=0, DEVICE_YELLOW=1, DEVICE_RED=2, DEVICE_REMOTE=3};

struct state_t
{
  uint8_t magic;
  uint8_t current_state;
  uint8_t current_location;
  uint8_t remote_state;   //sys.remoteStatus bitmask
  int event_sequence;
  uint8_t alert_hours;
  float oil_gallons; // or level ??
  unsigned long checkin_hours;  //bitmap of hours to perform checkin.
}
alarm_saved_state;
#define ALARM_STATE_ADDRESS 0
#define CONFIGURATION_ADDRESS sizeof(state_t)

//device config data in eeprom array (Saved)
struct config_t
{
  uint8_t magic;
  uint8_t sys_id[MAXDEVICE];
  uint8_t dev_addr[MAXDEVICE][8];
  uint8_t dev_flags[MAXDEVICE];
  uint8_t port[MAXDEVICE];
  uint8_t use[MAXDEVICE];
  uint8_t master_idx[MAXDEVICE];
  uint8_t sense[MAXDEVICE];
  uint8_t alert_level[MAXDEVICE]; //HOME_DEVICE or AWAY_DEVICE
  uint8_t alert_min[MAXDEVICE];  // for thermometers
  uint8_t alert_max[MAXDEVICE];  // for thermometers
  char name[MAXDEVICE][SENSOR_NAME_SIZE+1];
};  //configuration - move to class, not global

//device config in runnint std::list (running)
//ADD alert-level to trip e.g. alert for thermometer
//converting to one device definition for both master and remote
struct device_t
{
    uint8_t idx;          // index: v0.1.6
    //change master_id to:
    //bool is_master;     // else remote
    //bool has_remote;    // how to implement ??
    uint8_t sys_id;       // system number for this sensor
    uint8_t master_idx;   // idx on REMOTE, now just idx (fix/remove)
    uint8_t status;       // missing, present, unknown
    uint8_t dev_flags;    //
    uint8_t state;        //disabled, active
		uint8_t dev_rom[8];
    //port and dev_use will be set according to master/remote
    uint8_t port;         //PIO-A==0, PIO-B==1
    uint8_t dev_use;
    uint8_t sense;        //sensor NC or NO
    uint8_t alert_level;  //HOME_DEVICE or AWAY_DEVICE
    uint8_t alert_min;    // 0 ==> no alert_point, <= this ==> do alert
    uint8_t alert_max;    // 0 ==> no alert_point, >= this ==> do alert
    float dev_reading;    //last PIO, temp, etc.
    system_tick_t dev_last_read;    //timestamp of last reading
    String name;
    uint8_t tripped;      //checkSensor says tripped
    uint8_t reported;     //reported by alert
    uint8_t changed;      //changed since last read
};

//just use put(address, object)
template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
  const byte* p = (const byte*)(const void*)&value;
  unsigned int i;
  for (i = 0; i < sizeof(value); i++)
    EEPROM.write(ee++, *p++);
  return i;
}

//just use EEPROM.get(addr, object)
template <class T> int EEPROM_readAnything(int ee, T& value)
{
  byte* p = (byte*)(void*)&value;
  int i;
  for (i = 0; i < sizeof(value); i++)
    *p++ = EEPROM.read(ee++);
  return i;
}

// Log message to cloud, message is a printf-formatted string
// e.g. debug("ledPin = %d", ledPin);
/* xxxxyy
void debug(String message, int value) {
    char msg [50];
    sprintf(msg, message.c_str(), value);
    Spark.publish("DEBUG", msg);
}
*/
#endif
