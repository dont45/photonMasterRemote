/*
  Master-Remote alarm system for Particle Photon
  @file     Alarm-Master.cpp
  @author   D. Thompson
  @license  GNU General Public License (see license.txt)
  @version  4.1

  Copyright (C) 2016 Donald Thompson, Raynham Engineering

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

/*
 * Working Version 4: 4.0.2, shed and bicycle remotes working fully
 * Final Version 3: 3.2.1, ran zero errors from Jan 1,2017 to Oct 10, 2017
 * Final Version of 3.1 is 3.1.4.f, 3.1.4
 * Ran with zero issues and no restarts for 80 plus days whild
 * away on camping trip.
 */
/*
 * Ver 4.0
 * Add port sensor type
 * Debug remote nodes
 * Testing with Electron, cellular
 * Added ALWAYS on sensor to run in DISABLED state
 * Remote nodes set as ALWAYS run when master is disabled
 * Added OFF statre for really off
/*
 * Ver 3.2
 * Add Checi-In feature
 * Funtions:
 * Provide notification to a group of pushover users in the event that
 * the alarm owner fails to respond within the required time-window to
 * a check-time event generated according to a specific schedule
 * by the Alarm system user.
 * This feature is being implemented as a health / safty check of the
 * alarm user, perhaps eldery, disabled, or just lives alone.  should
 * an accident or health event occur an the Alarm user is unable to 'click'
 * the 'all is well' button on the iPhone app, the user list will received
 * a pushover notification of 'No Timely Checkin by xxx'.
 */

/* TO DO:
 * In 4.1 - .2
 * Bug: Clearing Remote Evt from DISARMED resets master to HOME
 * Design: Remote timout (>DEVICE_TIMEOUT) to mark devices NOT_PRESENT
 * Change status message, remove worry and checkin hours, and add evt info
 * Add Settings message, show worry and checkin hours and ???
 * --
 * Add support for hardware watchdog timer STWD100
 * Add general multi-function alarm node as sub-node (SUB_ALARM_SYSTEM)
 * This 'sensor type' will function like SUB_CAR_MONITOR except that
 * it will handle multiple alert messages.  These messages will be
 * embedded into a string to include sensor name and sensor state.
 * also need positive ack messages back.
 * How do we manage remote configuration?  Like sensor name and sensor min values?
 *
 * readSensor & checkSensor need to return more information about sensor.
 * not just a bool, or perhaps a bool and a second return value.
 *
 * alarm enable and alarm Acknowledge need to verify sensors clear before setup
 * or perhaps not?  Idea:  If you want to enable the alarm even though one sensor is
 * tripped, first disable that tripped sensor, then you can reset alarm.
 * worry notifications should show disabled sensors as a reminder.
 */

 /*
  * Added in v3.2
  * Stopped writing saved state to EEPROM, use SAV command instead.
  * This is to preserve EEProm Write cycle until state is moved.
  * Going away long term, do commands SET AWAY, then SAV.
  * Short term SET / DIS sequence does NOT save state
  *
  * Checkin -- A system to monitor 'users' health via a required periodic
  * checkin.
  *   The genisis of this system was my 8 week camping trip where I was almost
  * constantly alone, often in very remote places.  Rather than require that
  * someone check up on me every day, I added this feature to Master-Remote.
  * While it obviously does not monitor me, it does keep a maximum of a few
  * hour window to notify all my emergency contacts that someting is wrong,
  * i.e. I have not responded to a 'checkin' alert.
  *
  *   Implemented is a 'Time to Checkin' alert followed by a window for the
  * 'user' to press the 'chckin' button.  If user does not check in, an alert
  * is sent to all users on the 'emergency' list.  This feature also includes
  * a 'panic' button, which when pressed, generates an immediate 'Panic Alert'
  * to that same list.
  * Other features include management of checkin hours and suspension of further
  * checkins for up to 24 hours.
  */
 /*
  * Added in  v3.1.0
  * HOME an AWAY modes and commands to set
  * Redesigned alarm event handling (in class Alarm)
  */

/*
 * Added in  v1.9
 * add last updated timestamp on device
 *  to receiver car temperature alerts from sub-sensor
 * remote car sensor JUST needs to send temperature every xx minutes
 * the SUB_CAR_MONITOR device here does everything need to manage alerts
 *
 * create webhooks shed_alarm and shed_notify to utilize pushover quiet hours
 * fix oilHandler to use new Tank class (level)
 * Add priority to queued messages
 * ?? direct interacton (request and confirm) should have priority
 * worry-status is low priority
 * tank level change is low
 * each device should have an alarm-conditon priority assigned
 * therefore, tank has low but a door sensor has high
 *
 */

/* Added in v 1.8
* Add oil level to saved data and load on startup
* writre oil level to saved data on subscription receipt
 * oil level data as subscription
 * execute command then confirm with secret sent
 * Added in v 1.6
 * Sensor Id (sensor#) to device_t (array index in config_t)
 * Find by Id (in sensor LIST)
 * update to config_t at idx and write to EEPROM
 * command NAME to rename sensor (by sensor#)
 * command SEN to display all data of a sensor (by sensor#)
 * Removed short_name
 */

/* Particle Cloud published events:
 * alarmState with name of new state any time state changes
 */
// To Do:
// a) fix sysState clearing to shut off blinking Status
//    USE a timer in sysState
//    Timer ext and in class cause build to fail
// b) fix multiple notifications upon alarm condition
// d) add i-button support as key and as config management
// e) add HTU21 humidity sensor support
// f) second 1-wire buss for i-button support (clear readint)
//    or can we skip-rom to just read class 01 == i-button ??
// g) add ds thermometer support

#define SYSTEM_VERSION 4.1.0
#define SYSTEM_VERSION_MAJOR 4
#define SYSTEM_VERSION_MINOR 1
#define SYSTEM_VERSION_SUB 0

#include "application.h"
#include <queue>
#include <list>
#include "Adafruit_MCP9808.h"
#include "OW.h"
#include "parms.h"
#include "util.h"
#include "sensor.h"
#include "math.h"
#include "stdlib.h"
#include "parse.h"
#ifdef HARDWARE_WATCHDOG
#include "hwd.h"
#endif

#ifdef PHOTON_MASTER
//#define SERIAL_DEBUG
//#define SERIAL_DEBUG_ALARM
//#define SERIAL_DEBUG_SEN
//#define SERIAL_DEBUG_SEN1
//#define DEBUG_ONLY_ONE_SENSOR
//#define SERIAL_DEBUG_REMOTE
//#define SERIAL_DEBUGXX
//#define SERIAL_DEBUG_EVENT
 //#define SERIAL_DEBUGXX
//#define SERIAL_DEBUG_CONFIG
//#define SERIAL_WAIT
//#define SERIAL_LOOP_WAIT
//#define DEBUG_PARSE_ELEMENT
//#define DEBUG_PARSE_ELEMENT1
//#define DEBUG_PARSE_DETAIL
//#define HARDWARE_WATCHDOG
#define HALT_ON_HDW_ERROR
#endif

#ifdef PHOTON_REMOTE
//#define SERIAL_DEBUG
//#define SERIAL_DEBUG_SEN
//#define SERIAL_DEBUG_SEN1
//this disables all but ONE sensor <<<===
//#define SERIAL_DEBUG_ALARM;
//#define SERIAL_DEBUG_CONFIG
//#define SERIAL_DEBUG_REMOTE
//#define SERIAL_DEBUG_CLOUD
//#define SERIAL_WAIT
//#define SERIAL_LOOP_WAIT
//#define HARDWARE_WATCHDOG
//#define DEBUG_PARSE_ELEMENT
//#define DEBUG_PARSE_DETAIL
#endif


//STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));
//retained uint8_t testcurState;

int ledStatus = SYSTEM_STATUS_PIN;            //blink this LED at each loop pass
int ledNotice = SYSTEM_NOTIFICATION_PIN;      //blink this LED at each notification
int trigger3 = TRIGGER_PIN;
int message_countdown = 0;
//#ifdef PHOTON_MASTER
double temperatureF;                          // to publish
double gallonsO;                              // published oil gallons (demo)
double remote_temp;                           // published as temprmt
String stateA;
//#endif
#ifdef PHOTON_REMOTE
uint8_t remoteState;
uint8_t nextState;
char data[80];
String remoteData;
int msgSeq = 0;
#endif

Adafruit_MCP9808 mcp9808;
OW ow(0);       //default 0 is 0x18 !!
// OW ow1(1);   //NO! testing second ds2483 for iButtons
// iButton ib(1);
// we SHOULD make a specific iButton class, similar to OW

#include "state.h"
State sys;

#include "tank.h"
// Tank class performs current tank content in gallons based upon
// oil level present in tank (inches of oil in tank)
Tank::Tank(uint8_t style) {
    tankStyle = style;
    volumeIN2 = 0;
    cur_gallons = 0;
    width = tank_d[style][WIDTH];
    diameter = tank_d[style][DIAMETER];
    length = tank_d[style][LENGTH];
   radius = 28 / 2;
    fullVolume();
    fullGallons();
}

// calculate inches of oil remaining in tank from sensor reading in mm
// sensor reading is from sensor at top of tank to oil surface
int Tank::calcOilLevel(int sensorReading) {
    int inchReading = sensorReading / 25;
    int sensorHeight;
    switch(tankStyle) {  //SHOULD be done int initializer
	      case VERTICAL_275:
	      case VERTICAL_330:
    	   sensorHeight = tank_d[tankStyle][DIAMETER];
	       break;
	      case HORIZONTAL_275:
	      case HORIZONTAL_330:
    	    sensorHeight = tank_d[tankStyle][WIDTH];
	      default:
	        sensorHeight = 0.0;
    }
    return sensorHeight - inchReading;
}

// calculate area of segment of curved section, bottom, top, side
float Tank::calcCurved(int lev) {
    float at;
    float a3;
    float a2;
    float a;
    float ht;
    float lt;
    float alpha;
    float alphadeg;
    float sinBeta;
    float beta;
    float betadeg;
    at = pi * radius * radius;
    ht = radius - lev;
    sinBeta = ht / radius;
    beta = asin(sinBeta);
    betadeg = beta/pi * 180.00;
    lt = cos(beta) * radius;
    a3 = ht * lt;
    alpha = ((pi/2) - beta)*2;
    alphadeg = alpha / pi * 180.00;
    a2 = ((2*pi - alpha) / (2 * pi)) * at;
    a = at - a2 - a3;
    return a;
}

// calculate volume of entire tank
int Tank::fullVolume() {
    float volEnd = pi * radius * radius * length;  //both ends
    float volMid = (diameter - width) * width * length;
    volumeIN2 = volEnd + volMid;
    return volumeIN2;
}

float Tank::fullGallons() {
    cur_gallons = volumeIN2 * CVT_IN2_GALLONS;
    return cur_gallons;
}

// level in inches, return tank volume in cubic inches
int Tank::tankVolume(int level) {
  float topVol = 0;
  float midVol;
  float botVol;
  float endVol;
  float botArea = 0.0;
  float midArea = 0.0;
  float topArea = 0.0;
  float endArea = 0.0;
  float s;
  float l;
  float fullCirc = pi * radius * radius;
  float curvedArea = 0.0; //debug
  switch(tankStyle) {
    case VERTICAL_275:
    case VERTICAL_330:
	      // end curve based upon width , i.e. 27 / 2  = 13
	      // top curve contains oil
        if( level > diameter - radius) {
	          s = diameter - width + 0;	// full straight area
	          l = diameter - level;	//??
	          botArea = fullCirc / 2.0;
	          midArea = s * width;
	          curvedArea = calcCurved(l);  //debug
            topArea = fullCirc/2.0 - calcCurved(l);
	      }
	      // top within straight section
	      else if(level >= radius) {
	          s = level - radius;
	          l = radius;
	          botArea = fullCirc / 2.0;
	          midArea = s * width;
	      }
	      // oil level in bottom curve
	      else {
	          s = 0;
	          l = level;
	          curvedArea = calcCurved(level); //debug
	          botArea = calcCurved(level);
	     }
	     midVol = midArea * length;
	     botVol = botArea * length;
	     topVol = topArea * length;
	     volumeIN2 = (int)(topVol + midVol + botVol);
	     return volumeIN2;
       break;
    case HORIZONTAL_275:
    case HORIZONTAL_330:
	     // calc curved section, use X2, one for each end
	     // calc mid section: level * w * l
	     // when level crosses half full, formula works just fine!
	     s = diameter - width;
	     midArea = level * s;
	     midVol = midArea * length;
	     endArea = calcCurved(level);
	     endVol = endArea * length;  //vol for the two ends
	     volumeIN2 = (int)(endVol + midVol);
	     return volumeIN2;
       break;
    default:
       return -1; //error
  }
  return -1;
}

float Tank::tankGallons(int level) {
   int vol_cubic_in = tankVolume(level);
   return vol_cubic_in * CVT_IN2_GALLONS;
}

// remote data stream parser
//constructor with data
Parse::Parse(String d) {
  remotedata = d;
  idx = 0;
  parsePos = 0;
  elementDelimiter = ELEMENT_DELIMITER;
  endDelimiter = END_DELIMITER;
}
//constructor with data and delimiters
Parse::Parse(char elementd, char endd, String d) {
  remotedata = d;
  idx = 0;
  parsePos = 0;
  elementDelimiter = elementd;
  endDelimiter = endd;
}
//set data to parse
void Parse::setData(String s) {
  idx = 0;
  parsePos = 0;
  //remotedata.assign(s);
  remotedata = String(s);
  elementDelimiter = ELEMENT_DELIMITER;
  endDelimiter = END_DELIMITER;
}
void Parse::setDelimiters(char d_element, char d_end) {
  elementDelimiter = d_element;
  endDelimiter = d_end;
}
//set data to parse with special delimiters
void Parse::setData(char elementd, char endd, String s) {
  idx = 0;
  parsePos = 0;
  //remotedata.assign(s);
  remotedata = String(s);
  elementDelimiter = elementd;
  endDelimiter = endd;
}

// get the input string
String Parse::getData() {
  return remotedata;
}

int Parse::parseNext(int pos, int end) {
  char lastc;
  if(pos) pos++;
  lastc = remotedata.charAt(end-1);
  element[idx] = remotedata.substring(pos, end);
  idx++;
  return idx;
}

//find position of next delimiter in remotedata
int Parse::findDelim(char delimiter, int start) {
  parsePos = remotedata.indexOf(delimiter, start+1);
  return parsePos;
}

String Parse::getElement(int idx) {
  return element[idx];
}

//do parse into parsed_elementU
int Parse::doParse() {
  int start = 0;
  int end = 0;
  int n;
  for(n=0;n<MAX_PARSED_ITEMS;n++) {
    end = findDelim(elementDelimiter, start);
    if(end < 0) end = findDelim(endDelimiter, start);
    if(end < 0) break;
    /*
    Serial.print("doParse: start=");
    Serial.print(start);
    Serial.print(" end=");
    Serial.println(end);
    */
    parseNext(start,end);
    start = end;
  }
  return n;  // parsed n elements
}

#include "notifier.h"

void Sensor::setDeviceActive(device_t *d, bool toActive) {
    if(toActive)
      d->state = SENSOR_ACTIVE;
    else {
      d->state = SENSOR_DISABLED;
      if(d->dev_use >= FIRST_SUB_REMOTE) { //a REMOTE device
        d->dev_reading = 0;
        d->dev_last_read = 0;
        d->tripped = FALSE;
        d->reported = FALSE;
      }
    }
}

// set sensor indicator (on PIO_A)
void Sensor::setSensorIndicator(device_t d, uint8_t indval) {
  #ifdef SERIAL_DEBUGII
  Serial.print("setting indicator:");
  Serial.println(indval);
  #endif
  //#define PIO_A 0
  //#define PIO_B 1
  //d.port is the sensor read port
  //indicator port is the other one ??
  uint8_t indicator_port = d.port ? 0 : 1;
  #ifdef SERIAL_DEBUGII
    Serial.print("setSensorIndicator:prot=");
    Serial.print(d.port);
    Serial.print(" indicator port=");
    Serial.println(indicator_port);
  #endif
  p_ow->writePIOtest(d.dev_rom, indicator_port, indval);  //WRITE TO PIO-x
}

// return TRUE if tripped
// TO DO: FIX reaing of MCP9808 master vs remote sensor
// master reades device, remote just returns dev_reading

bool Sensor::readSensor(device_t &d) {
    uint8_t senval;
    uint8_t prev_senval;
    uint8_t sensor_priority = 0;  //reporting priority
    double tempF;
    double oilL;
    bool senret;
    bool rok;
    bool toSetSensor;
    float reading;
    if(d.status != DEV_PRESENT) {
  #ifdef SERIAL_DEBUG_SEN11
      Serial.print("Device Missing:");
      Serial.print(d.idx);
      Serial.print("  :");
      Serial.println(d.name);
  #endif
      return FALSE;
    }
    if(d.state == SENSOR_DISABLED) {
  #ifdef SERIAL_DEBUG_SEN1
      Serial.print("Device deactivated - ROM:");
      Serial.println(d.dev_rom[7],HEX);
  #endif
      return FALSE;
    }
    d.changed = FALSE;
    #ifdef SERIAL_DEBUG_SEN
            Serial.print("readSensor name=");
            Serial.print(d.name);
            Serial.print(" port=");
            Serial.print(d.port);
            Serial.print(" use=");
            Serial.print(d.dev_use);
            Serial.print(" master=");
            Serial.println(d.master_idx);
    #endif

    switch(d.dev_use) {
      case SWITCH:    //USE IO_SENSOR
  #ifdef SERIAL_DEBUG_SEN
          Serial.print("readSensor name=");
          Serial.print(d.name);
          Serial.print(" port=");
          Serial.print(d.port);
          Serial.print(" use=");
          Serial.print(d.dev_use);
  #endif
         senval = digitalRead(d.port);
         reading = (float)senval;
         //move this out of switch
         if(abs(d.dev_reading - reading) < 0.1) d.changed = TRUE;
         d.dev_reading = reading;
         d.dev_last_read = Time.now();
  #ifdef SERIAL_DEBUG_SEN
          Serial.print(" readvalue=");
          Serial.println(senval);
  #endif
          return senval==d.sense;
          break;
      case IO_SENSOR:  //SAME AS SWITCH ??
          // read hardware port d.dev_addr
          prev_senval = d.dev_reading;
          senval = digitalRead(d.port);
          reading = (float)senval;
          d.changed = senval != prev_senval;
          d.dev_reading = reading;
          d.dev_last_read = Time.now();
          senret = senval==d.sense;
#ifdef SERIAL_DEBUG_SEN
          if(d.changed) {
            Serial.print("readSensor name=");
            Serial.print(d.name);
            Serial.print(" port=");
            Serial.print(d.port);
            Serial.print(" use=");
            Serial.print(d.dev_use);
            Serial.print(" sensel=");
            Serial.print(d.sense);
            Serial.print(" senval=");
            Serial.print(senval);
            Serial.print(" prev_senval=");
            Serial.print(prev_senval);
            Serial.print(" senret=");
            Serial.print(senret);
            Serial.print(" changed=");
            Serial.println(d.changed);
          }
#endif
          return senret;
          break;
      case OW_SENSOR:
      case OW_INDICATOR:  //one-wire device on DS2482-100
          senval = p_ow->readPIOX(d.dev_rom, d.port);    //??just readPIO and use d.port
          toSetSensor = (uint8_t)d.dev_reading != senval;
          prev_senval = d.dev_reading;
          reading = (float)senval;
          d.changed = senval != prev_senval;
          d.dev_reading = reading;
          //............
          d.dev_last_read = Time.now();
          senret = senval==d.sense;
          #ifdef SERIAL_DEBUG_SEN1
                    if(d.changed) {
                      Serial.print("readSensor OW_SENSOR name=");
                      Serial.print(d.name);
                      Serial.print(" port=");
                      Serial.print(d.port);
                      Serial.print(" use=");
                      Serial.print(d.dev_use);
                      Serial.print(" sense=");
                      Serial.print(d.sense);
                      Serial.print(" senval=");
                      Serial.print(senval);
                      Serial.print(" prev_senval=");
                      Serial.print(prev_senval);
                      Serial.print(" senret=");
                      Serial.println(senret);
                    }
          #endif
          if(d.dev_use==OW_INDICATOR)
            //if(toSetSensor) setSensorIndicator(d, senret);
            setSensorIndicator(d, senret);
          return senret;
          break;
      // TODO: Implement??
      // This should set 'this' remote active, not just this one device
      // Bug: it is NOT this device to set active
      // how do we tell it what device ??
      // use master_idx ??
      case SUB_SET_DEVICE: // DESIGN: set remote active
        if(d.dev_last_read > Time.now() - DEVICE_TIMEOUT) {
              if(d.dev_reading > 0.5){
                #ifdef SERIAL_DEBUG_REMOTE
                    Serial.print("SUB_SET_DEVICE: remotesys=");
                    Serial.println(d.master_idx);
                #endif
                sys.setRemoteStatus(d.master_idx,TRUE);
                 // and send message it's set ???
                 //BUG:need alarm1 in scope ?????
                //alarm1.tellRemote(d.master_idx,'R');
              }
              d.dev_reading = 0.0;
            }
          break;
      case SUB_REMOTE_SENSOR:
          // validat dev_last_read: DEVICE_TIMEOUT = 1/2 HR TOO long??
          if(!sys.getRemoteStatus(d.master_idx)) return FALSE; //REMOTE not enabled (id??)
          if(d.dev_last_read > Time.now() - DEVICE_TIMEOUT) {
            senval = d.dev_reading;
            senret = senval==d.sense;
            #ifdef SERIAL_DEBUG_REMOTE1
                Serial.print("SUB_REMOTE_SENSOR: remotesys=");
                Serial.print(d.master_idx);
                Serial.print(" senretl=");
                Serial.println(senret);
            #endif
              return senret;
          }
          else {
#ifdef SERIAL_DEBUG_SEN
              Serial.println("SUB_REMOTE:timne expired ");
#endif
              return FALSE;
          }
          break;
      case SUB_REMOTE_THERMOMETER:
          //TODO: NEED TO VALIDATE dev_last_read ??
          // was return d.dev_reading > d.alert_max;
          if(!sys.getRemoteStatus(d.master_idx)) return FALSE; //REMOTE not enabled (id??)
          if(d.dev_last_read > Time.now() - DEVICE_TIMEOUT) {
            senval = d.dev_reading;
            senret = 0;   //senval==d.sense; //should check MIN's MAX
            return senret;
          }
          else {
#ifdef SERIAL_DEBUG_SEN
              Serial.println("SUB_REMOTE_TH:timne expired ");
#endif
              return FALSE;
          }
            break;
      case SUB_OIL_GAUGE:        //demo particle.subscribe value
              //last_oil_level is updated by subscription
              //d.dev_reading = last_oil_level;
              if(d.dev_reading < -1000.0) return FALSE;
              if(d.dev_last_read < Time.now() - DEVICE_TIMEOUT) {
                d.dev_reading = -1001.0;
                d.state=SENSOR_INACTIVE;
                return FALSE;
              }
              if(d.alert_min !=0 && d.dev_reading <= d.alert_min)
                return TRUE;
              //sensor can trip based upon notification levels
              //green condition = no notification or -1??
              //yellow condition = 0
              //red condition = priority = 1
              return FALSE;
          break;
      case SUB_CAR_MONITOR:    //sub-sensor via particle.subscribe value
              //dev_reading is updated by subscription (remoteHandler)
              // 1) temp received compared to max setting
              if(d.dev_reading < -1000.0) return FALSE;
              if(d.dev_last_read < Time.now() - DEVICE_TIMEOUT) {
                d.dev_reading = -1001.0;
                d.state=SENSOR_INACTIVE;
                //notifier.queueMessage(SHORT_HEADER,1,"Car Monitor Inactive");
                return FALSE;
              }
              if(d.alert_max !=0 && d.dev_reading >= d.alert_max)
                return TRUE;
              // 2) time-stamp of last data received
              // 3) return TRUE if alert condition
              return FALSE;     // TO DO: alert if temp < ??
          break;

      case MCP9808_THERMOMETER:  //precision thermometer on I2C
              // readTempF should use config parameters to get device info
              // i.e. i2c address etc.  This allows multiple devices on bus
              last_temp = mcp9808.readTempF();
  #ifdef SERIAL_DEBUG_SEN
              Serial.print("READ MCP...");
              Serial.print(last_temp);
              Serial.print(" dev:");
              Serial.println(d.dev_reading);
  #endif
              reading=d.dev_reading;
              d.dev_reading = last_temp;
              d.dev_last_read = Time.now();
              // return value should  be tripped if outside temp range ??
              // report using priority of sensor ??
#ifdef PHOTON_REMOTE
              if(abs(last_temp - reading)>0.1) {
                  Serial.println("temp diff!");
                  return TRUE;
               }
#else
              if((int)last_temp <= d.alert_min) return TRUE;
              // readTemp should return an error condition ??
#endif
              return FALSE;
        break;
        case OW_THERMOMETER:    //ds1820 thermometer
            #ifdef SERIAL_DEBUG_THERM
              Serial.print("ds1820 testing-rom:");
              for(int i=0;i<8;i++) {
                Serial.print(d.dev_rom[i],HEX);
                Serial.print("-");
              }
              Serial.println();
            #endif
              d.dev_reading = -999.0; //debug marker ??
              rok = p_ow->readThermometer(d.dev_rom, tempF);
              if(rok) {
                d.dev_reading = tempF;
                d.dev_last_read = Time.now();
            #ifdef SERIAL_DEBUG_THERM
                Serial.print("temp Fahrenheit x100:");
                Serial.println((int)tempF*100);
            #endif
                if((int)tempF <= d.alert_min) return TRUE;
              }
              return FALSE;  //not tripped
        break;
      default:
        break;
  }
  return FALSE;
}

//clear changedSenbsors
void Sensor::clearChangedSensors() {
  changedList.clear();
  trippedSensor = FALSE;
}

//add sensor to changedList
void Sensor::addChangedSensor(device_t sp) {
    changedList.push_back(sp);
    trippedSensor = TRUE;
}

//are there any changed sensors?
bool Sensor::hasChangedSensor() {
    return !changedList.empty();
  }

  //is any sensor tripped (or all clear)
bool Sensor::hasTrippedSensor() {
      return trippedSensor;
    }

// return formatted string of changedSensors
String Sensor::fmtChangedSensors() {
  //FORMAT:   @<sensor>^<sensor>^...<sensor>^$
  // sensor:  <name>:<master_idx>:<dev_reading>:<tripped>
  std::list<device_t>::iterator k;
  char tempbuf[30];
  char changedbuf[100];
  changedbuf[0]=0;
  #ifdef SERIAL_DEBUG_SEN
  Serial.print("fmtChangedSensor: items=");
  Serial.println(changedList.size());
  #endif
  for( k=changedList.begin(); k != changedList.end(); ++k) {
    #ifdef SERIAL_DEBUG_SEN
    Serial.print("adding sensor: master=");
    Serial.print(k->master_idx);
    Serial.print(" name=");
    Serial.println(k->name);
    #endif
    strcat(changedbuf,"@");
    strncat(changedbuf, k->name.c_str(), SENSOR_NAME_SIZE);
    sprintf(tempbuf,":%d:%6.4f:%d^",k->master_idx,k->dev_reading,k->tripped);  //<<??
    strcat(changedbuf, tempbuf);
   }
   String changed = String(changedbuf);
   return changed;
}

//REMOVE THIS...use readSensor()
float Sensor::readTemperature() {
  // last_temp should be stored by sensor,
  // part of sensor data ??
  last_temp = mcp9808.readTempF();
  return last_temp;
}

float Sensor::getLastTemperature() {
  return last_temp;
}

char* Sensor::romFormat(char *buf, uint8_t rom[]) {
  char *bp = buf;
  int n = 0;
  for(int i=0;i<8;i++) {
    bp = &buf[n];
    int rb = rom[i];
    if(rb<16) *bp++='0';
    itoa((int)rb, bp, 16);
    n+=2;
    buf[n++]='-';
  }
  buf[n-1]=0;
  return buf;
}

Sensor sensor(&ow);

// notify hours quiet hours, alarm does not
#include "checkin.h"
Checkin checkin(alarm_saved_state.checkin_hours);
Notifier notify("low_priority","shed_notice","shed_alarm","emergency", &sensor);
//Notifier notify("shed_notic_e","shed_notice","shed_alarm","emergency", &sensor);

#include "alarm.h"
Alarm alarm1;

// lets make a special config class to handle EEPROM config
// allocate configuration with new and dealoc in destructor
// use instance of this class in Alarm routines which handle
// configuration. ???
Alarm::~Alarm(void) {
  delete p_config;
}

//scan 1-wire bus and build configuration of devices found
//preserve data (use, name, etc. ) from existing configuration
//if device was previously present
bool Alarm::generateConfiguration() {

}

//update a specific device based upon user input changes (NAME)
bool Alarm::updateConfiguration(device_t* dev, int n) {
  strncpy(configuration.name[n], dev->name.c_str(), SENSOR_NAME_SIZE);
  EEPROM_writeAnything(CONFIGURATION_ADDRESS, configuration);
}

int Alarm::clearingCount() {
  return clearing_cnt;
}

//return FALSE if some sensor still tripped
//return TRUE if all now clear
bool Alarm::clearSensorReported() {
  // run thru sensor list and clear reporting if not tripped
  // send error message if still tripped and leave reported
  std::list<device_t>::iterator k;
  bool all_clear = TRUE;
  for(k=device_list.begin(); k != device_list.end(); ++k) {
     //device_t sp = *k;
     if(k->reported) {
       if (k->tripped) {
         all_clear = FALSE;
       }
       else
         k->reported = FALSE;
     }
   }
   return all_clear;
}

//move it to Alarm...
//read temperature sensor from device_list by index
float Alarm::readTemperature(uint8_t idx) {
  std::list<device_t>::iterator k;
  float sen_temp = -999.0;
  for(k=device_list.begin(); k !=device_list.end(); ++k) {
    if(k->idx != idx) continue;
    //NOTE, for REMOTE device, readSensor does nothing, dev_reading is already set
    //for actual device, MCP9808 device is read and dev_reading is updated
    sensor.readSensor(*k);
    sen_temp = k->dev_reading;
    break;
  }
  return sen_temp;
}

String Alarm::getLastTemperature() {
  return String(sensor.getLastTemperature());
}

uint8_t Alarm::getState() {
  return curState;
}
//descriptive text of alarm state
String Alarm::getStateDescription(uint8_t st) {
  String stateDescr = alarm_state_name_def[st];
  if(curState == alarm_armed)
    stateDescr = alarm_state_location_def[curLocation];
  return stateDescr;
}
String Alarm::getStateDescription() {
  return getStateDescription(curState);
}

// new set state based upon new conditions, state machine
// returns -1 if NO state change
int Alarm::setState() {
  uint8_t new_state;
  /* if alarm_disarmed ==> no change
   * if any isTripped and not reported ==>  alarm_armed
   */
   //BUG..INCLUDE alarm_disarmed in else case, delete here
   //need new pvt member baseState ??
   //and method to return baseState

  if(curState == alarm_off) return -1; //no change
  if(isTripped()) {
    //can't be all clear with a sensor tripped
    if(isReported()) {
      new_state = alarm_notifying;
      prevTripState = FALSE;
      clearing_cnt=0;
    }
    else
      new_state = alarm_tripped;
  }
  else {  //nothing tripped
    if(allClear())  //nothing unacknowledged
      new_state = baseState;  //here use getbaseState()
    else  //still need to Acknowledge
      //sensors all clear but events not Acknowledge
      new_state = alarm_clearing;
  }
  if(curState == new_state) return -1;
  #ifdef SERIAL_DEBUG_EVENT
  Serial.print("setState() from:");
  Serial.print(curState);
  Serial.print(" to:");
  Serial.println(new_state);
  #endif
  curState = new_state;
  setStatusLED();
  stateA = getStateDescription(curState);
  Particle.publish("alarmState",stateA,60);
  tellRemote(0,curState);
  return curState;
}

// rewrite of setState(x)
bool Alarm::setState(uint8_t new_state) {
  bool retval = FALSE;
  if(curState == new_state) //NO State Change
    return FALSE;
#ifdef SERIAL_DEBUG_EVENT
  Serial.print("setState(x) to ");
  Serial.print(alarm_state_name_def[new_state]);
  Serial.print(" :curState =");
  Serial.println(alarm_state_name_def[curState]);
#endif
  if(new_state == alarm_disarmed || new_state == alarm_off) {
      //don't allow if any sensor tripped -- trippedList.empty();
      if(!trippedList.empty()) {
      //send warning message about outstanding events --event_list.empty();
      String msg="sensors tripped!";
      notify.queueMessage(SHORT_HEADER,1,msg);
      return FALSE;
      }
      if(!event_list.empty()) {
        //message...
        String msg = "Warning: unacknowledged events!";
        notify.queueMessage(SHORT_HEADER,1,msg);
      }
//      curState = alarm_disarmed;
      curState = new_state;
      baseState = curState;
      //then clear event_list
      event_list.clear();
      retval = TRUE;  //???
  }
  else if(new_state == alarm_armed) {
      curState = alarm_armed;
      baseState = curState;
      //write to eeprom ??
      retval = TRUE;
  }
  else {
    retval = FALSE;
  }
  setStatusLED();
  stateA = getStateDescription(curState);
  Particle.publish("alarmState",stateA,60);
  //DESIGN: HERE WE need to publish masterdata for remotes:
  //remote sysid to be zero (effect all remotes)
  //format: <seq?>@<sysid>@<event>@<timestamp>$
  //eg: 0@0@H@01:45:0000$
  //do this in Alarm::tellRemote(curState);
  tellRemote(0,curState);
  return retval;
}

#ifdef OLD_SET_STATE
// review all this (states)
// TODO : fix stateA to include AT_HOME, AWAY
bool Alarm::oldsetState(uint8_t new_state) {
  bool result = TRUE;
  if(curState == new_state) //NO State Change
    return FALSE;
#ifdef SERIAL_DEBUGXX
  Serial.print("setState(x) to ");
  Serial.print(alarm_state_name_def[new_state]);
  Serial.print(" :curState =");
  Serial.println(alarm_state_name_def[curState]);
#endif
  switch(new_state) {
      case alarm_disarmed:
        curState = alarm_disarmed;
        //write to EEPROM
    #ifdef SERIAL_DEBUG
        Serial.println("alarm disarmed");
    #endif
        message = "DISARMED";
        ledStatusState = 0;
        ledStatusBlink = 0;
        break;
    case alarm_armed:
        //write to EEPROM
    #ifdef SERIAL_DEBUG
        Serial.println("alarm armed");
    #endif
        // add concept of prevTripped
        if(isTripped()) {
            //fast blink status
            ledStatusBlink = 2;
            message = "NOT CLEAR";
            result = FALSE;
            break;
        }
        else {
          ledStatusState = 1;
          ledStatusBlink = 0;
        }
        prevTripState = FALSE;
        clearing_cnt=0;
        if(isArmed())
          message = "ALARM RESET";
        else
          message = "ALARM SET";
        curState = new_state;
        break;
    case alarm_tripped:
        curState = alarm_tripped; //aaaaaa
    #ifdef SERIAL_DEBUG
        Serial.println("alarm tripped");
    #endif
        ledStatusBlink = 1;
        prevTripState = TRUE;
        //tripListString();
        //message = trippedString; //??FIX
        message = tripListString();
        break;
    case alarm_notifying:
        curState = new_state; //aaaaaa
        prevTripState = FALSE;
    #ifdef SERIAL_DEBUG
        Serial.print("alarm notifying:");
        Serial.println(clearing_cnt);
    #endif
        char buf[20];
        sprintf(buf,"NOTIFYING:%d", clearing_cnt);
        message = String(buf);
        ledStatusState = 1;
        ledStatusBlink = 2;
        /*
        if(prevTripped()) {
          if(clearing_cnt > CLEARING_COUNT_MAX) { //?? count
            debug("clearing_cnt=%d\n",clearing_cnt);
            prevTripState = FALSE;
            clearing_cnt = 0;
            setState(alarm_clearing);
          }
        }
        */
        break;
    case alarm_clearing:
        //only if sensors clear??
        message = "CLEARED";
        #ifdef SERIAL_DEBUGXX
        Serial.println("set state alarm_clearing...");
        #endif
        //let user ACK before setting to armed unless REMOTE
        #ifdef PHOTON_REMOTE
        setState(alarm_armed);  //recursive maybe not still armed??
        #endif
        break;
    default:
    #ifdef SERIAL_DEBUG
        Serial.println("invalid status");
    #endif
        message = "INVALID ALARM STATUS";
  }
  digitalWrite(ledStatus, ledStatusState);
  // Send a publish of new alarm sate to your devices...
  stateA = getStateDescription(new_state);
  Particle.publish("alarmState",stateA,60);
  return result;
}
#endif //OLD_SET_STATE

// set state of all remotes
void Alarm::setRemotes(uint8_t remotes) {
  uint8_t rmask = remotes;
  char rstate;
  for(int i=0; i< 8; i++) {
    if(rmask & 1) rstate = 'X';
    else rstate = 'O';
    tellRemote(i, rstate);   //{O,X}
    rmask << 1;
  }
}
//update all remotes with new alarm state
void Alarm::tellRemote(uint8_t remoteid, char newstate) {
  char data[20];
  int msgseq=1;  //FROM WHERE??
  String time = Time.format("%H:%M:%S");
  sprintf(data,"%d@%d@%c@%s$",msgseq++,remoteid,alarm_state_short[newstate],time.c_str());
  String mData = String(data);
  Particle.publish("masterdata",mData);
}

void Alarm::setStatusLED() {
  switch(curState) {
    case alarm_disarmed:
        ledStatusState = 0;
        ledStatusBlink = LED_OFF;
        break;
    case alarm_armed:
        ledStatusState = 1;
        ledStatusBlink = LED_SOLID;
    break;
    case alarm_notifying:
        ledStatusState = 1;
        ledStatusBlink = LED_FAST;
    break;
    case alarm_clearing:
        ledStatusState = 1;
        ledStatusBlink = LED_SLOW;
    break;
    default:
        ledStatusState = 0;
        ledStatusBlink = LED_OFF;
  }
  digitalWrite(ledStatus, ledStatusState);
}

//----------------------
#ifdef PHOTON_REMOTE
void Alarm::setLEDRemote(char rs) {
#ifdef SERIAL_DEBUG_REMOTE
  Serial.print("setLEDRemote:");
  Serial.println(rs);
#endif
//const char alarm_state_short[9] = {'F','D','A','T','N','C','R','X','O'};

  switch(rs) {
    case 'F': // System OFF
        ledNoticeState = 0;
        ledStatusState = 0;
        ledStatusBlink = LED_OFF;
    break;
    case 'D': //DISARMED
        ledStatusState = 0;
        ledStatusBlink = LED_OFF;
    break;
    case 'H': //Home
    case 'A': //Away
        ledStatusState = 1;
        ledStatusBlink = LED_SOLID;
    break;
    case 'T':
        ledStatusState = 1;
        ledStatusBlink = LED_FAST;
    break;
    case 'C':
    case 'N':
        ledStatusState = 1;
        ledStatusBlink = LED_SLOW;
    break;
    case 'O': //Remote Off
        ledNoticeState = 0;
    break;
    case 'X': //Remote On
        ledNoticeState = 1;
    break;
    default:
        ledStatusState = 0;
        ledStatusBlink = LED_OFF;
  }
  digitalWrite(ledStatus, ledStatusState);
  digitalWrite(ledNotice, ledNoticeState);
}
#endif

uint8_t Alarm::buildDeviceList() {
  uint8_t dev_cnt = 0;
  char nbuf[30];  //at least SENSOR_NAME_SIZE   and SIZE for ROMHEX ??
  if(configuration.magic != EE_MAGIC_CONFIG) return 0;
#ifdef SERIAL_DEBUG_CONFIG
  Serial.println("buildDeviceList:");
#endif
  for(int j = 0; j< MAXDEVICE; j++) {
    if(!configuration.use[j]) continue;
    #ifdef SERIAL_DEBUG_CONFIG
      sensor.romFormat(&nbuf[0], configuration.dev_addr[j]);
      Serial.print(j);
      Serial.print(":config rom=");
      Serial.print(nbuf);
      Serial.print(" master=");
      Serial.print(configuration.master_idx[j]);
    #endif
    device.idx=j;
    device.master_idx = configuration.master_idx[j];
    for(int i=0;i<8;i++)
      device.dev_rom[i] = configuration.dev_addr[j][i];
    device.port = configuration.port[j];
    device.dev_use = configuration.use[j];
    device.state = SENSOR_ACTIVE;
    if(device.dev_use >= 8 )  //TO DO: make #define
      device.status = DEV_PRESENT;
    else
      device.status = DEV_MISSING;
    device.sense = configuration.sense[j];
    device.alert_level = configuration.alert_level[j];
    device.alert_min = configuration.alert_min[j];
    device.alert_max = configuration.alert_max[j];
    strncpy(nbuf,configuration.name[j],SENSOR_NAME_SIZE);
    nbuf[SENSOR_NAME_SIZE]=0;
    device.name = String(nbuf);
#ifdef SERIAL_DEBUG_CONFIG
    Serial.print("adding device: name=");
    Serial.println(device.name);
#endif
    device.dev_reading = 0.0;
    device.dev_last_read = 0;
    device.tripped = 0;
    device.reported = 0;
    device_list.push_back(device);
    dev_cnt++;
  }
#ifdef SERIAL_DEBUG_CONFIG
  Serial.print("added to device_list:");
  Serial.println(dev_cnt);
#endif
  return dev_cnt;
}

char* Alarm::deviceListing(char *buf) {
  std::list<device_t>::iterator k;
  char temp[10];
  buf[0]=0;
  strcat(buf,"\n"); //json escape line feed
  for(k=device_list.begin(); k !=device_list.end(); ++k) {
     device_t sp = *k;
     sprintf(temp,"%02d ", sp.idx);
     strcat(buf,temp);
     strncat(buf,sp.name,SENSOR_NAME_SIZE);
     strcat(buf,":");
     if(k->status==0) strcat(buf,"M");      //missing
     else if(k->status==1) strcat(buf,"P"); //present
     else strcat(buf,"U");                  //unknown
     if(k->state == SENSOR_ACTIVE) {
       strcat(buf,"A");                     //active
       if(k->tripped) strcat(buf,"T");      //tripped
       else strcat(buf,"C");                //clear
       if(k->reported) strcat(buf,"R");     //reported
     } else
        strcat(buf,"I");                    //inactive
     strcat(buf,"\n");
   }
#ifdef SERIAL_DEBUGXX
   Serial.println("deviceListing");
   Serial.println(buf);
#endif
   return buf;
}

char* Alarm::thermometerListing(char *buf) {
  std::list<device_t>::iterator k;
  char temp[80];
  buf[0]=0;
  strcat(buf,"\n"); //json escape line feed
  for(k=device_list.begin(); k !=device_list.end(); ++k) {
     if(k->dev_use != SUB_REMOTE_THERMOMETER &&
        k->dev_use != OW_THERMOMETER &&
        k->dev_use != MCP9808_THERMOMETER)  continue;
     device_t sp = *k;
     strncat(buf,sp.name,SENSOR_NAME_SIZE);

     int sec_since_update = Time.now() - sp.dev_last_read;
     /*
     if(sec_since_update > 7200) //2 hours
        strcat(temp," ---- ");
     else
        sprintf(temp,"  %4d  ",sec_since_update);
     strcat(buf, temp);  //dev_last_read
     */
     String lastUpd = Time.format(sp.dev_last_read, "%I:%M%p");  //03:21AM
     sprintf(temp," %s  %5.1fF\n", lastUpd.c_str(), sp.dev_reading);
     strcat(buf,temp);
   }
   #ifdef DEBUGXX
   Serial.println("thermometerListing");
   Serial.println(buf);
   #endif
   return buf;
}

// event queue listing
char* Alarm::eventListing(char *buf) {
  std::map<int, uint8_t>::iterator e;
  char temp[SENSOR_NAME_SIZE+20];
  char name[SENSOR_NAME_SIZE+1];
  buf[0]=0;
  name[0]=0;
  strcat(buf,"\nevent list");
  if(event_list.empty()) strcat(buf,"no events");
  else
    for(e=event_list.begin(); e !=event_list.end(); ++e) {
      device_t* dp=getDevice(e->second);
      if(dp!=0) {
        strncpy(name,dp->name,SENSOR_NAME_SIZE);
        name[SENSOR_NAME_SIZE]=0;
      }
      sprintf(temp, "\n%02d %s %d", e->second, name, e->first);
      strcat(buf, temp);
    }
    return buf;
  }

// trippedList to String
String Alarm::tripListString() {
  std::list<device_t>::iterator k;
  int i = 0;
  bool has_tripped = FALSE;
  char trippedbuf[60];
  trippedbuf[0]=0;
  strcpy(trippedbuf,"TRIPPED:\n");
  for(k=trippedList.begin(); k != trippedList.end(); ++k) {
     device_t sp = *k;
     //??TODO: do something with reported ??
     strncat(trippedbuf,sp.name,SENSOR_NAME_SIZE);
     strcat(trippedbuf,"\n");
     i++;
     has_tripped=TRUE;
   }
   if(!has_tripped) strcat(trippedbuf,"none");
   #ifdef SERIAL_DEBUGXX
   Serial.print("trippedListString: no=");
   Serial.print(i);
   Serial.print(" List=");
   Serial.print(trippedbuf);
   #endif
   trippedString = String(trippedbuf);
   return trippedString;
}

//encode from tripped sensor xxxxyy
//must do in Alarm class:
//returns C, T, R
uint8_t Alarm::formatRemoteTrip(char* data) {
  std::list<device_t>::iterator k;
  char buf[20];
  data[0]=0;
  #ifdef SERIAL_DEBUGXX
    Serial.print("formatRemoteTrip:");
    Serial.println(" from device_list");
  #endif
  uint8_t sensorState = SENSOR_CLEAR;
  //for(k=trippedList.begin(); k != trippedList.end(); ++k) {
  for(k=device_list.begin(); k != device_list.end(); ++k) {
    #ifdef SERIAL_DEBUGXX
      Serial.print("device idx=");
      Serial.print(k->idx);
      Serial.print("  master_idx idx=");
      Serial.print(k->master_idx);
      Serial.print("  name=");
      Serial.print(k->name);
      Serial.print("  dev_reading=");
      Serial.print(k->dev_reading);
      Serial.print("  tripped=");
      Serial.print(k->tripped);
      Serial.print("  reported=");
      Serial.println(k->reported);
    #endif
    //check tripped based upon sensor use xxxxzz
    if(k->dev_reading > k->alert_max) {
      if(k->tripped) sensorState = SENSOR_RESET;
      else sensorState = SENSOR_CLEAR;
    }
    else {
      if(k->tripped) {
        sensorState = SENSOR_STILL_TRIPPED;
        k->reported = FALSE;
      }
      else sensorState = SENSOR_TRIPPED;
    }
    #ifdef SERIAL_DEBUGXX
      Serial.print("sensorState=");
      Serial.println(sensorState);
    #endif
    if(k->master_idx) { //only sensors with assoc. master
      //what about checking sensor type ??
      device_t sp = *k;
      strcat(data,"@");
      strncat(data,sp.name,SENSOR_NAME_SIZE);
      sprintf(buf,":%d:%f:%d",sp.master_idx,sp.dev_reading,sensorState);
      strcat(data,buf);
    }
  }
  return sensorState;  //return VALUE ??
}

//set non-wire devices to present
void Alarm::setValidNonWire() {
  std::list<device_t>::iterator k;
  for(k=device_list.begin(); k !=device_list.end(); ++k) {
    if(k->dev_use >= FIRST_NON_WIRE) {
      k->status = DEV_PRESENT;
    }
  }
}

//debug dump of sensor list
void Alarm::dump_device_list() {
  std::list<device_t>::iterator k;
  char romhex[30];

for(k=device_list.begin(); k !=device_list.end(); ++k) {
   device_t sp = *k;
   sensor.romFormat(romhex, sp.dev_rom);
   Serial.print(sp.idx);
   Serial.print("  Status:");
   Serial.print(sp.status);
   Serial.print(" master:");
   Serial.print(sp.master_idx);
   Serial.print(" Port:");
   Serial.print(sp.port);
   Serial.print(" Use:");
   Serial.print(sp.dev_use);
   Serial.print(" [");
   Serial.print(sensor_use_descr[sp.dev_use]);
   Serial.print("] ROM:");
   Serial.print(romhex);
   Serial.print(" Name:");
   Serial.print(sp.name);
   Serial.println();
  }
Serial.println();
}

bool Alarm::validate_device_list() {
  //run thru device list and check rom present
  std::list<device_t>::iterator k;
  bool all_present = TRUE;
  int dev_present_cnt = 0;
  Serial.println("device_list:");
  for(k=device_list.begin(); k !=device_list.end(); ++k) {
    device_t sp = *k;
    Serial.print(sp.idx);
    Serial.print(":");
    Serial.print(sp.name);
    Serial.print("midx:");
    Serial.print(sp.master_idx);
    Serial.print(" st:");
    Serial.println(sp.status);
    //OW_SENSOR=2, OW_RELAY=3, OW_THERMOMETER=4,
    if(sp.dev_use >= OW_SENSOR && sp.dev_use <= OW_THERMOMETER)
      if(sp.status != 1) {
        all_present = FALSE;
        Serial.print("DEV not present:");
        Serial.println(sp.name);
      }
      else dev_present_cnt++;
  }
  //if ow devices configured, must have at least one to run
  if(!dev_present_cnt && !all_present) {
    // no ow dev found
    sys.addStatus(fail_owdevice);
    notify.sendMessage(SHORT_HEADER,1,"NO OWDEV");   //send this message now
#ifdef HALT_ON_HDW_ERROR
    sys.sysState(sys_fail); //this will do HANG
#endif
  }
  if(all_present)
    Serial.print("all dev present: cnt=");
    Serial.println(dev_present_cnt);
  return all_present;
}

void Alarm::setDeviceAlertMin(device_t *d, int min) {
  //device_t* sp=&(*k);
    d->alert_min = min;
}
void Alarm::setDeviceAlertMax(device_t *d, int max) {
    d->alert_max = max;
    //write it to config ??TO DO:
}

//set AT_HOME or AWAY
void Alarm::setCurLocation(uint8_t loc) {
  if(loc < 0 || loc > 1) loc = 0;
  //just use loc &= 1;  //??
  curLocation = loc;
}
uint8_t Alarm::getCurLocation() {
  if(curLocation < 0 || curLocation > 1) return 0;
  return curLocation;
}

//This is set master device from remote data
bool Alarm::setAlarmRemote(uint8_t sysid, uint8_t idx, uint8_t senv, float setv) {
  std::list<device_t>::iterator k;
  bool fountIt = FALSE;
  for(k=device_list.begin(); k != device_list.end(); ++k) {
    if(k->idx != idx) continue;
    //  DESIGN; need dev_remote boolean ??
#ifdef SERIAL_DEBUG_REMOTE
    Serial.print("setAR: idx=");
    Serial.print(idx);
#endif
    if(k->dev_use == SUB_SET_DEVICE || k->dev_use == SUB_REMOTE_SENSOR)
        k->dev_reading = senv;
    else if(k->dev_use == SUB_REMOTE_THERMOMETER) k->dev_reading = setv;
    else k->dev_reading = -1.0;
    k->master_idx = sysid; //in MASTER, this is remote's sys id
    k->dev_last_read = Time.now();
    tellRemote(sysid, remotes_enabled);
    fountIt = TRUE;
#ifdef SERIAL_DEBUG_REMOTE
    Serial.print(" DEVICE set from remote");
    Serial.print("senValue:");
    Serial.print(senv);
    Serial.print("setValue:");
    Serial.print(setv);
    Serial.print(" stamp:");
    Serial.print(k->dev_last_read);
    Serial.print(" use=");
    Serial.println(k->dev_use);
#endif
  }
  Serial.println();
  return fountIt; //FALSE
}
//ONLY sensor with 'SUB..' use type
bool Alarm::setLastRemote(uint8_t use, float val) {
  std::list<device_t>::iterator k;
  if(use < FIRST_SUB_REMOTE) return FALSE;
  for(k=device_list.begin(); k != device_list.end(); ++k) {
    if(k->dev_use == use) {
      k->dev_reading = val;
      k->dev_last_read = Time.now();
      k->state = SENSOR_ACTIVE;
      return TRUE;
    }
  }
  return FALSE;
}

//TODO: remove THIS
bool Alarm::alarmNotifying() {
  return curState == alarm_notifying;
}

//?? changed
bool Alarm::isTripped() {
  //return curState ==alarm_tripped;  // || curState ==alarm_notifying;
  return !trippedList.empty();
}

//all tripped devices have been reported
bool Alarm::isReported() {
  std::list<device_t>::iterator k;
  bool has_unreported = FALSE;
  for(k=trippedList.begin(); k != trippedList.end(); ++k)
     if(!k->reported) has_unreported  = TRUE;
  return !has_unreported;
}

bool Alarm::prevTripped() {
  return prevTripState;  //??
}
device_t* Alarm::firstTripped() {
  if(trippedList.empty()) return NULL;
  return &trippedList.front();
}
bool Alarm::isArmed() {
  return curState != alarm_off;
}
String Alarm::getPendingMessage() {
  return message;
}
uint8_t Alarm::getPriority() {
  return priority;
}

//blink ledStatusState
//ledStatusBlink is style of blink: none,slow,fast
#define FAST_LED_BLINK_CNT 0
#define SLOW_LED_BLINK_CNT 3

/*
ledStatusState: 1 or 0, value writen to LED
ledStatusBlink: blink speed (off,on,fast,slow)
ledStatusBlinkCount: countdown to led change state
*/
void Alarm::doStatusBlink() {
  ledStatusBlinkCount++;
  switch(ledStatusBlink) {
    case LED_OFF:
            break;
    case LED_SOLID: //solid
            break;
    case LED_SLOW: //slow
            if(ledStatusBlinkCount > SLOW_LED_BLINK_CNT) {
              ledStatusBlinkCount = 0;
              //switch led state
              if(++ledStatusState > 1) ledStatusState = 0;
            }
            break;
    case LED_FAST: //fast
            if(ledStatusBlinkCount > FAST_LED_BLINK_CNT) {
              ledStatusBlinkCount = 0;
              if(++ledStatusState > 1) ledStatusState = 0;
            }
            break;
  }
  digitalWrite(ledStatus, ledStatusState);
//DESIGN BUG: Notice should be controled by class State ??
#ifdef PHOTON_REMOTE
  digitalWrite(ledNotice, ledNoticeState);
#endif
}

void Alarm::blinkTimeout() {
#ifdef SERIAL_DEBUG
  Serial.println("Alarm blinkTimeout");
#endif
  ledStatusBlink = 0;
  ledStatusBlinkCount = 0;
}

// IS this ROM in device_list?
bool Alarm::wiredeviceValidate(uint8_t *ROM) {
  std::list<device_t>::iterator k;
  for(k=device_list.begin(); k !=device_list.end(); ++k) {
    device_t sp = *k;
    bool matching = TRUE;
    for(int i=0;i<8;i++)
      if(ROM[i] != sp.dev_rom[i]) matching = FALSE;
    if(matching) {
  #ifdef SERIAL_DEBUG
      Serial.print("use:");
      Serial.print(k->dev_use);
      Serial.print(" ROM CD:");
      Serial.print(ROM[7],HEX);
      Serial.print(" dev_rom:");
      Serial.print(k->dev_rom[7],HEX);
      Serial.println(" seting status PRESENT");
  #endif
      k->status = DEV_PRESENT;  //why does this NOT set status ????
      return TRUE;
   }
  }
  return FALSE;
}

// Scan wire for all DEVICES
// MOVE TO SENSOR
// DEBUG, for now just validate them as present
uint8_t Alarm::wiredeviceAquire() {
  uint8_t ROM[8];
  uint8_t i,j,k;
  ow.wireResetSearch();
  for(i=0;i<MAX_ALLOW_OWDEV;i++) {
    j=ow.wireSearch(ROM); //CRASHES HERE !!!
    if(j==0) break;
    //how to handle iButton keys ??
    /*
     * if iButton is present =>
     * we are generating a NEW configuration
     * if not present, validate what is found
     * NOTE: must do iButton family search
     * to get ONLY ibuttons, then so
     * wiredeviceAquire to validate or generate ??
     */
    //validate that this sensor is in device_list
    //DESIGN: should not valide if NOT wire
    //side effect: setting PRESENT
    uint8_t valid = wiredeviceValidate(ROM);
#ifdef SERIAL_DEBUG
    Serial.print("sensor add:");
    for(k=0;k<8;k++) {
      Serial.print( ROM[k],HEX);
      Serial.print(":");
    }
    Serial.print(" valid:");
    Serial.println(valid);
#endif
  }
  ow.wireResetSearch();
  return i;
}

// Write simple test configuration to EEPROM
bool Alarm::writeTestConfiguration() {
  int i = 0;
  configuration.magic = EE_MAGIC_CONFIG;
  //using XXX_COFIGURATION from parms.h
  //Sensor 1: mcp_9808
#ifdef TEST_SENSOR_1
  configuration.dev_addr[i][0] = MCP9808_I2CADDR;
  configuration.master_idx[i] = 0;
  for(int k=1;k<8;k++)
    configuration.dev_addr[i][k] = 0xff;
  configuration.dev_flags[i] = 0;
  configuration.port[i] = 0;
  configuration.use[i] = MCP9808_THERMOMETER;
  configuration.sense[i] = SENSE_NORMAL_OPEN;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 0;
  configuration.alert_max[i] = 0;
  strncpy(configuration.name[i], "INSIDE TEMP", SENSOR_NAME_SIZE);
  i++;
#endif
  //Sensor 2: NE Shed Door new sensor pcbWAY
#ifdef TEST_SENSOR_2
  configuration.master_idx[i] = 0;
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = 0xff;
  configuration.dev_flags[i] = 0;
  configuration.port[i] = 0;
  configuration.use[i] = SUB_REMOTE_SENSOR;
  configuration.sense[i] = SENSE_NORMAL_OPEN;
  configuration.alert_level[i] = ALWAYS_DEVICE;
  configuration.alert_min[i] = 0;
  configuration.alert_max[i] = 0.9;
  strncpy(configuration.name[i], "SHED DOOR", SENSOR_NAME_SIZE);
  i++;
#endif
  //Sensor 3
#ifdef TEST_SENSOR_3
const uint8_t testrom2[8] = { 0x05,0xe9,0xef,0x05,0x00,0x00,0x00,0x42 };
  configuration.master_idx[i] = 0;
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom2[k];
  configuration.dev_flags[i] = 1 << DEVICE_PRIORITY;
  configuration.port[i] = PIO_B;
  configuration.use[i] = OW_SENSOR;
  configuration.sense[i] = SENSE_NORMAL_CLOSED;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 0;
  configuration.alert_max[i] = 0;
  strncpy(configuration.name[i], "OWS SWITCH3", SENSOR_NAME_SIZE);
  i++;
#endif
  //Sensor 4
#ifdef TEST_SENSOR_4
const uint8_t testrom3[8] = { 0x10,0xc8,0xb6,0x1e,0x00,0x00,0x00,0x3b };
  configuration.master_idx[i] = 0;
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom3[k];
  configuration.dev_flags[i] = 0;
  configuration.port[i] = PIO_B;
  configuration.use[i] = OW_THERMOMETER;
  configuration.sense[i] = 0;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 0;
  configuration.alert_max[i] = 0;
  strncpy(configuration.name[i], "OUTSIDE TEMP4", SENSOR_NAME_SIZE);
  i++;
#endif
  //Sensor 5
#ifdef TEST_SENSOR_5
const uint8_t testrom4[8] = { 0x10,0x5d,0xab,0x4c,0x01,0x08,0x00,0xf3 };
  configuration.master_idx[i] = 0;
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom4[k];
  configuration.dev_flags[i] = 0;
  configuration.port[i] = PIO_B;
  configuration.use[i] = OW_THERMOMETER;
  configuration.sense[i] = 0;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 70;
  configuration.alert_max[i] = 0;
  strncpy(configuration.name[i], "OUTSIDE TEMP", SENSOR_NAME_SIZE);
  i++;
#endif
#ifdef TEST_SENSOR_6 //ds2407 green star
const uint8_t testrom5[8] = { 0x12,0x6d,0x25,0x0a,0x00,0x00,0x00,0x39 };
  configuration.master_idx[i] = 0;
  for(int k=0;k<8;k++)
      configuration.dev_addr[i][k] = testrom5[k];
  configuration.dev_flags[i] = 1 << DEVICE_PRIORITY;
  configuration.port[i] = PIO_B;
  configuration.use[i] = OW_SENSOR;
  configuration.sense[i] = SENSE_NORMAL_OPEN;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 0;
  configuration.alert_max[i] = 0;
  strncpy(configuration.name[i], "MOTION", SENSOR_NAME_SIZE);
  i++;
#endif
#ifdef TEST_SENSOR_7
//add:12:73:25:A:0:0:0:71:  ds2407 red star
const uint8_t testrom6[8] = { 0x12,0x73,0x25,0x0a,0x00,0x00,0x00,0x71 };
  configuration.master_idx[i] = 0;
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom6[k];
  configuration.dev_flags[i] = 1 << DEVICE_PRIORITY;
  configuration.port[i] = PIO_B;
  configuration.use[i] = OW_SENSOR;
  configuration.sense[i] = SENSE_NORMAL_CLOSED;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 0;
  configuration.alert_max[i] = 0;
  strncpy(configuration.name[i], "O7 DS2407", SENSOR_NAME_SIZE);
  i++;
#endif
#ifdef TEST_SENSOR_8
//add:5:67:F8:5:0:0:0:96:
const uint8_t testrom7[8] = { 0x05,0x67,0xf8,0x05,0x00,0x00,0x00,0x96 };
  configuration.master_idx[i] = 0;
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom7[k];
  configuration.dev_flags[i] = 1 << DEVICE_PRIORITY;
  configuration.port[i] = PIO_B;
  configuration.use[i] = OW_SENSOR;
  configuration.sense[i] = SENSE_NORMAL_CLOSED;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 0;
  configuration.alert_max[i] = 0;
  strncpy(configuration.name[i], "O8 DS2407", SENSOR_NAME_SIZE);
  i++;
#endif
#ifdef TEST_SENSOR_9
//add:3A:7:30:18:0:0:0:BA:  ds2413
const uint8_t testrom8[8] = { 0x3A,0X07,0X30,0X18,0x00,0x00,0x00,0xBA };
  configuration.master_idx[i] = 2;   //pseudo sensor idx in MASTER
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom8[k];
  configuration.dev_flags[i] = 1 << DEVICE_PRIORITY;
  configuration.port[i] = PIO_B;
  configuration.use[i] = OW_SENSOR;
  configuration.sense[i] = SENSE_NORMAL_OPEN;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 0;
  configuration.alert_max[i] = 0.90;
  strncpy(configuration.name[i], "FRONT DOOR", SENSOR_NAME_SIZE);
  i++;
#endif
#ifdef TEST_SENSOR_10
//sensor add:12:D5:7F:46:0:0:0:45: WatchDog Water Alarm
const uint8_t testrom9[8] = { 0x12,0xd5,0x7f,0x46,0x00,0x00,0x00,0x45 };
  configuration.master_idx[i] = 4;
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom9[k];
  configuration.dev_flags[i] = 1 << DEVICE_PRIORITY;
  configuration.port[i] = PIO_A;
  configuration.use[i] = OW_SENSOR;
  configuration.sense[i] = SENSE_NORMAL_OPEN;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 0;
  configuration.alert_max[i] = 0;
  strncpy(configuration.name[i], "WATER DETECTOR", SENSOR_NAME_SIZE);
  i++;
#endif
#ifdef TEST_SENSOR_11
//subscription sensor -- oil level data
const uint8_t testrom10[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
  configuration.master_idx[i] = 0;
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom10[k];
  configuration.dev_flags[i] = 1 << DEVICE_RED;  //priority alert at RED level
  configuration.port[i] = 0;
  configuration.use[i] = SUB_OIL_GAUGE;
  configuration.sense[i] = 0;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 0;  //use for red/yellow alerts ??
  configuration.alert_max[i] = 0;
  strncpy(configuration.name[i], "OIL LEVEL", SENSOR_NAME_SIZE);
  i++;
#endif
#ifdef TEST_SENSOR_12
//subscription sensor -- car temperature alarm (pets)
const uint8_t testrom11[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
  configuration.master_idx[i] = 0;
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom11[k];
  configuration.dev_flags[i] = 0;
  configuration.port[i] = 0;
  configuration.use[i] = SUB_CAR_MONITOR;
  configuration.sense[i] = 0;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 50.0;
  configuration.alert_max[i] = 81.0;
  strncpy(configuration.name[i], "CAR MONITOR", SENSOR_NAME_SIZE);
  i++;
#endif
#ifdef TEST_SENSOR_13
//subscription sensor -- REMOTE ALARM pseudo
const uint8_t testrom12[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
  configuration.master_idx[i] = 0;  //Only defined on remote device
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom12[k];
  configuration.dev_flags[i] = 0;
  configuration.port[i] = 0;
  configuration.use[i] = SUB_REMOTE_SENSOR;
  configuration.sense[i] = SENSE_NORMAL_OPEN;
  configuration.alert_level[i] = ALWAYS_DEVICE;
  configuration.alert_min[i] = 0.0;
  configuration.alert_max[i] = 0.90;
  strncpy(configuration.name[i], "BICYCLE SEED", SENSOR_NAME_SIZE);
  i++;
#endif
#ifdef TEST_SENSOR_14
  //subscription sensor -- MASTER pseudo device
    configuration.master_idx[i] = 0;
    for(int k=0;k<8;k++)
      configuration.dev_addr[i][k] = 0xff;
    configuration.dev_flags[i] = 0;
    configuration.port[i] = 0;
    configuration.use[i] = SUB_SET_DEVICE;
    configuration.sense[i] = 0;
    configuration.alert_level[i] = ALWAYS_DEVICE;
    configuration.alert_min[i] = 0.0;
    configuration.alert_max[i] = 0.5;
    strncpy(configuration.name[i], "REMOTE ENABLE", SENSOR_NAME_SIZE);
    i++;
#endif
#ifdef TEST_SENSOR_15
    //Temperature MCP9808 -- REMOTE Actual Device
      configuration.master_idx[i] = 0;
      configuration.dev_addr[i][0] = 0X00;
      for(int k=1;k<8;k++)
        configuration.dev_addr[i][k] = 0Xff;
      configuration.dev_flags[i] = 0;
      configuration.port[i] = 0;
      configuration.use[i] = SUB_REMOTE_THERMOMETER;
      configuration.sense[i] = 0;
      configuration.alert_level[i] = ALWAYS_DEVICE;
      configuration.alert_min[i] = 0.0;
      configuration.alert_max[i] = 999.0;
      strncpy(configuration.name[i], "OUTSIDE TEMP", SENSOR_NAME_SIZE);
      i++;
#endif
#ifdef TEST_SENSOR_16
const uint8_t testrom15[8] = { 0x28,0xbb,0xe8,0x30,0x07,0x00,0x00,0x37 };
configuration.master_idx[i] = 5;
for(int k=0;k<8;k++)
  configuration.dev_addr[i][k] = testrom15[k];
configuration.dev_flags[i] = 0;
configuration.port[i] = PIO_B;
configuration.use[i] = OW_THERMOMETER;
configuration.sense[i] = 0;
configuration.alert_level[i] = ALWAYS_DEVICE;
configuration.alert_min[i] = 0;
configuration.alert_max[i] = 0;
strncpy(configuration.name[i], "BOILER TEMP", SENSOR_NAME_SIZE);
i++;
#endif
#ifdef TEST_SENSOR_17
//subscription sensor -- REMOTE BASEMENT WATER
//const uint8_t testrom12[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
  configuration.master_idx[i] = 0;
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = 0xff;;
  configuration.dev_flags[i] = 0;
  configuration.port[i] = 0;
  configuration.use[i] = SUB_REMOTE_SENSOR;
  configuration.sense[i] = SENSE_NORMAL_OPEN;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 0.0;
  configuration.alert_max[i] = 0.90;
  strncpy(configuration.name[i], "BASEMENT WATER", SENSOR_NAME_SIZE);
  i++;
#endif

#ifdef TEST_SENSOR_18
  //subscription sensor -- MASTER pseudo device
    configuration.master_idx[i] = 0;
    for(int k=0;k<8;k++)
      configuration.dev_addr[i][k] = 0xff;
    configuration.dev_flags[i] = 0;
    configuration.port[i] = 0;
    configuration.use[i] = SUB_REMOTE_THERMOMETER;
    configuration.sense[i] = 0;
    configuration.alert_level[i] = AWAY_DEVICE;
    configuration.alert_min[i] = 0.0;
    configuration.alert_max[i] = 999.0;
    strncpy(configuration.name[i], "BOILER TEMP", SENSOR_NAME_SIZE);
    i++;
#endif
#ifdef TEST_SENSOR_19 //ds2407 green star
const uint8_t testrom18[8] = { 0x12,0x6d,0x25,0x0a,0x00,0x00,0x00,0x39 };
  configuration.master_idx[i] = 0;
  for(int k=0;k<8;k++)
      configuration.dev_addr[i][k] = testrom18[k];
  configuration.dev_flags[i] = 1 << DEVICE_PRIORITY;
  configuration.port[i] = PIO_B;
  configuration.use[i] = OW_SENSOR;
  configuration.sense[i] = SENSE_NORMAL_OPEN;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 0;
  configuration.alert_max[i] = 0;
  strncpy(configuration.name[i], "MOTION", SENSOR_NAME_SIZE);
  i++;
#endif
#ifdef TEST_SENSOR_20
//add:12:73:25:A:0:0:0:71:  ds2407 red star
const uint8_t testrom19[8] = { 0x12,0x73,0x25,0x0a,0x00,0x00,0x00,0x71 };
  configuration.master_idx[i] = 0;
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom19[k];
  configuration.dev_flags[i] = 1 << DEVICE_PRIORITY;
  configuration.port[i] = PIO_B;
  configuration.use[i] = OW_SENSOR;
  configuration.sense[i] = SENSE_NORMAL_OPEN;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 0;
  configuration.alert_max[i] = 0;
  strncpy(configuration.name[i], "GARAGE DOOR", SENSOR_NAME_SIZE);
  i++;
#endif
//Sensor 2: Test Parasitic Board
#ifdef TEST_SENSOR_21
const uint8_t testrom20[8] = { 0x12,0x50,0xF2,0x72,0x00,0x00,0x0,0x25 };
configuration.master_idx[i] = 0;
for(int k=0;k<8;k++)
  configuration.dev_addr[i][k] = testrom20[k];
configuration.dev_flags[i] = 1 << DEVICE_PRIORITY;
configuration.port[i] = PIO_B;
configuration.use[i] = OW_SENSOR;
configuration.sense[i] = SENSE_NORMAL_OPEN;
configuration.alert_level[i] = HOME_DEVICE;
configuration.alert_min[i] = 0;
configuration.alert_max[i] = 0;
strncpy(configuration.name[i], "DINING RM IR", SENSOR_NAME_SIZE);
i++;
#endif

//new sensor PCB board, pcbWAY SENSOR
//sensor add:12:D0:F:6D:0:0:0:27: valid:0
#ifdef TEST_SENSOR_22
const uint8_t testrom21[8] = { 0x12,0xD0,0x0F,0x6D,0x00,0x00,0x0,0x27 };
configuration.master_idx[i] = 0;
for(int k=0;k<8;k++)
  configuration.dev_addr[i][k] = testrom21[k];
configuration.dev_flags[i] = 1 << DEVICE_PRIORITY;
configuration.port[i] = PIO_B;
configuration.use[i] = OW_INDICATOR;
configuration.sense[i] = SENSE_NORMAL_OPEN;
configuration.alert_level[i] = AWAY_DEVICE;
configuration.alert_min[i] = 0;
configuration.alert_max[i] = 0;
strncpy(configuration.name[i], "NEW PCB SENSOR", SENSOR_NAME_SIZE);
i++;
#endif

// ds2413 BREAKOUT
//sensor add:3A:16:27:18:0:0:0:FB: valid:0
#ifdef TEST_SENSOR_23
const uint8_t testrom22[8] = { 0x3a,0x16,0x27,0x18,0x00,0x00,0x0,0xfb };
configuration.master_idx[i] = 0;
for(int k=0;k<8;k++)
  configuration.dev_addr[i][k] = testrom22[k];
configuration.dev_flags[i] = 1 << DEVICE_PRIORITY;
configuration.port[i] = PIO_B;
configuration.use[i] = OW_SENSOR;
configuration.sense[i] = SENSE_NORMAL_OPEN;
configuration.alert_level[i] = HOME_DEVICE;
configuration.alert_min[i] = 0;
configuration.alert_max[i] = 0;
strncpy(configuration.name[i], "INSIDE DOORS", SENSOR_NAME_SIZE);
i++;
#endif

//new sensor ds2406p, smoke detector
//sensor add:12:8E:15:6D:0:0:0:DB: valid:0
#ifdef TEST_SENSOR_24
const uint8_t testrom23[8] = { 0x12,0x8E,0x15,0x6D,0x00,0x00,0x0,0xDB };
configuration.master_idx[i] = 0;
for(int k=0;k<8;k++)
  configuration.dev_addr[i][k] = testrom23[k];
configuration.dev_flags[i] = 1 << DEVICE_PRIORITY;
configuration.port[i] = PIO_A;
configuration.use[i] = OW_SENSOR;
configuration.sense[i] = SENSE_NORMAL_OPEN;
configuration.alert_level[i] = ALWAYS_DEVICE;
configuration.alert_min[i] = 0;
configuration.alert_max[i] = 0;
strncpy(configuration.name[i], "FR SMOKE DETECTOR", SENSOR_NAME_SIZE);
i++;
#endif

//sensor on hardward port D5
#ifdef TEST_SENSOR_25
for(int k=0;k<8;k++)
  configuration.dev_addr[i][k] = 0xff;
configuration.master_idx[i] = 4;
configuration.dev_flags[i] = 1 << DEVICE_PRIORITY;
configuration.port[i] = D5;
configuration.use[i] = IO_SENSOR;
configuration.sense[i] = SENSE_NORMAL_OPEN; //DO define INPUT_PULLUP??
configuration.alert_level[i] = ALWAYS_DEVICE;
configuration.alert_min[i] = 0;
configuration.alert_max[i] = 0;
strncpy(configuration.name[i], "SEED SENSOR", SENSOR_NAME_SIZE);
i++;
#endif

//force-enable on hardward port D6
#ifdef TEST_SENSOR_26
for(int k=0;k<8;k++)
  configuration.dev_addr[i][k] = 0xff;
configuration.master_idx[i] = 5;
configuration.dev_flags[i] = 1 << DEVICE_PRIORITY;
configuration.port[i] = D6;
configuration.use[i] = IO_SENSOR;
configuration.sense[i] = SENSE_NORMAL_OPEN; //DO define INPUT_PULLUP??
configuration.alert_level[i] = ALWAYS_DEVICE;
configuration.alert_min[i] = 0;
configuration.alert_max[i] = 0;
strncpy(configuration.name[i], "ENABLE BUTTON", SENSOR_NAME_SIZE);
i++;
#endif

//SHED Door Physical
#ifdef TEST_SENSOR_27
//  const uint8_t testrom1[8] = {  0x12,0xD0,0x0F,0x6D,0x00,0x00,0x0,0x27 };
  const uint8_t testrom1[8] = {  0x12,0x3A,0x84,0x72,0x00,0x00,0x0,0xD8 };
  configuration.master_idx[i] = 1;
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom1[k];
  configuration.dev_flags[i] = 1 << DEVICE_PRIORITY;
  configuration.port[i] = PIO_B;
  configuration.use[i] = OW_SENSOR;
  configuration.sense[i] = SENSE_NORMAL_CLOSED;
  configuration.alert_level[i] = ALWAYS_DEVICE;
  configuration.alert_min[i] = 0;
  configuration.alert_max[i] = 0.9;
  strncpy(configuration.name[i], "SHED DOOR", SENSOR_NAME_SIZE);
  i++;
#endif

//sHED Outside Temperature MCP9808 -- Master Pseuco device
#ifdef TEST_SENSOR_28
      configuration.master_idx[i] = 6;
      configuration.dev_addr[i][0] = MCP9808_I2CADDR;
      for(int k=1;k<8;k++)
        configuration.dev_addr[i][k] = 0xff;
      configuration.dev_flags[i] =  1 << DEVICE_PRIORITY;
      configuration.port[i] = 0;
      configuration.use[i] = MCP9808_THERMOMETER;
      configuration.sense[i] = 0;
      configuration.alert_level[i] = ALWAYS_DEVICE;
      configuration.alert_min[i] = 0.0;
      configuration.alert_max[i] = 999.0;
      strncpy(configuration.name[i], "OUTSIDE TEMP", SENSOR_NAME_SIZE);
      i++;
#endif
//aDD 27 AND 28 ?? FOR Shed

  // and Write it
  EEPROM_writeAnything(CONFIGURATION_ADDRESS, configuration);
#ifdef SERIAL_DEBUG
  Serial.print("WriteTestConfiguration: DEV Count=");
  Serial.println(i);
#endif
  return TRUE;
}
bool Alarm::readConfiguration() {
  // READ configuration for updates
  // How to read eeprom on photon ??
  //EEPROM_readAnything(CONFIGURATION_ADDRESS, configuration);    //we will read unless button pressed to init network??
  EEPROM.get(CONFIGURATION_ADDRESS, configuration);
  if(configuration.magic==EE_MAGIC_CONFIG)
  {
  #ifdef SERIAL_DEBUG
    Serial.println("VALID CONFIG MAGIC");
  #endif
    return TRUE;
  }
  // MUST setup actual sensors, i.e. pinmodes etc.
  // Where?
#ifdef SERIAL_DEBUG
  Serial.println("INVALID CONFIG MAGIC");
#endif
  return FALSE;
}

bool Alarm::readSavedState() {
  EEPROM.get(ALARM_STATE_ADDRESS, alarm_saved_state);
  if(alarm_saved_state.magic==EE_MAGIC_STATE)
  {
#ifdef SERIAL_DEBUG
    Serial.println("VALID MAGIC STATE");
    Serial.print("sysState=");
    Serial.println(alarm_saved_state.current_state);
    Serial.print("curLocation=");
    Serial.println(alarm_saved_state.current_location);
    Serial.print("remotes=");
    Serial.println(alarm_saved_state.remote_state, BIN);
#endif
    curState = alarm_saved_state.current_state;
    if(curState > alarm_armed) baseState = alarm_armed;
    else baseState = curState;
    curLocation = alarm_saved_state.current_location;
    sys.setRemotes(alarm_saved_state.remote_state);
    //stateA = String(alarm_state_name_def[curState]);
    stateA = getStateDescription();
    ledStatusState = alarm_saved_state.current_state != alarm_disarmed;
    gallonsO = alarm_saved_state.oil_gallons;
    prevTripState = FALSE;
    event_sequence = alarm_saved_state.event_sequence;
    return TRUE;
  }
#ifdef SERIAL_DEBUG
  Serial.println("INVALID MAGIC STATE");
#endif
  return FALSE;
}
// TODO: we need to minimize eeprom writes to redue ware !!!
// In testing mode, just don't writes
// Don't write event
// Just write alarm State (set or disarmed, not sub-states)
bool Alarm::writeSavedState() {
  //limit these at time of call, not here ??
  //if(alarm_saved_state.current_state == isArmed()) return FALSE;
  alarm_saved_state.magic = EE_MAGIC_STATE;
  alarm_saved_state.current_state = isArmed();
  alarm_saved_state.current_location = curLocation;
#ifdef PHOTON_MASTER
  alarm_saved_state.alert_hours = notify.getAlertHours();
  alarm_saved_state.checkin_hours = checkin.getCheckinHours();
  alarm_saved_state.oil_gallons = gallonsO;
  alarm_saved_state.event_sequence = event_sequence;
  alarm_saved_state.remote_state = sys.getRemotes();
#endif
#ifdef SERIAL_DEBUG
  Serial.print("writing SavedState: current_state=");
  Serial.println(alarm_saved_state.current_state);
#endif
  EEPROM.put(ALARM_STATE_ADDRESS, alarm_saved_state);
  return TRUE;
}

bool Alarm::allClear() {
  return event_list.empty();
}
// returns TRUE if any sensor unreported tripped
// TO DO: change return value to priority, -1 = none tripped
// 0 = report as notify, 1 = report as alarm ???
// TO DO: use device's priority, i.e. in dev_flags to
// determine priority of alert.  Use highest found
// checkSensors should return priority, not bool
// -1 = not tripped, else priority of alert to send
bool Alarm::checkSensors(bool all) {
  //bool temptrip = FALSE;
  char buf[30];
  uint8_t alert_priority = 0;  //readSensor needs to update this
  bool sensorIsTripped = FALSE;
  bool sensorIsReporting = FALSE;
  bool reporting = FALSE;      //reporting a new tripped sensor
  firstTrippedSensor = 0;
  trippedList.clear();        //list of those sensor tripped
  sensor.clearChangedSensors();        //list of sensors which have changed state
  std::list<device_t>::iterator k;
  for(k=device_list.begin(); k !=device_list.end(); ++k) {
#ifdef DEBUG_ONLY_ONE_SENSOR
    if(k->idx != 4) continue;
#endif
    device_t sp = *k;
  #ifdef SERIAL_DEBUG_SEN
    Serial.print("checkSensor:device_list idx=");
    Serial.print(k->idx);
    Serial.print(" name=");
    Serial.print(k->name);
    Serial.print(" master_idx=");
    Serial.println(k->master_idx);
    Serial.print("port=");
    Serial.print(k->port);
    Serial.print(" rom=");
    for(int n=0;n<8;n++) {
      Serial.print(k->dev_rom[n],HEX);
      Serial.print(":");
    }
    Serial.print(" use=");
    Serial.print(k->dev_use);
    Serial.print(':');
    Serial.println(sensor_use_descr[k->dev_use]);
    Serial.print("tripped=");
    Serial.print(k->tripped);
    Serial.print(" reported=");
    Serial.print(k->reported);
    Serial.print(" dev_reading");
    Serial.println(k->dev_reading);
  #endif
    //if(sp.status == DEV_PRESENT) {  <=== add this ???
    if(sensor.readSensor(*k)) {
      if(curState == alarm_disarmed && k->alert_level != ALWAYS_DEVICE) return FALSE;
      //at_home sensor is disabled if currently at_home
      if(curLocation==AT_HOME && k->alert_level==HOME_DEVICE) return FALSE;
      if(k->tripped) {
        /* New design:
          1) generate a seq number for this event: 1000001 : event_sequence
             add this sequence to run data so it is continuous over restarts
          2) add event to map with seq and sensor idx
          3) send message for this event
          4) log event to memory calcCurved??
          5) ACK response with seq# will match to this event and remove from map queue
          6) message will be resent periodically for all events in queue
        */
        #ifdef SERIAL_DEBUG_ALARM
        Serial.print("tripped dev idx=");
        Serial.println(k->idx);
        #endif
        if(addEvent(k->idx)) {
          //added new event, so notify
          String event_message = "sensor tripped\n";
          event_message.concat(k->name.c_str());
          sprintf(buf,"\nevent seq:%d",event_sequence);
          event_message.concat(buf);
          notify.queueMessage(FULL_HEADER,1,event_message);
          #ifdef SERIAL_DEBUG_ALARM
          Serial.print("event_message:");
          Serial.println(event_message);
          #endif
          k->reported = TRUE;
          reporting = TRUE;
          sensorIsTripped = TRUE;
        }
        else {
          sensorIsReporting = TRUE;
        }
      }
      k->tripped = TRUE;
      //if(!k->reported) reporting = TRUE;
      trippedList.push_back(sp);    //if we don't clear, need to add only if not there
      #ifdef SERIAL_DEBUG_ALARM
      Serial.println("added to trippedList");
      #endif
      #ifdef PHOTON_REMOTE
      reporting = TRUE;
      #endif
    }
    else {
      k->tripped = FALSE;
    }
    #ifdef SERIAL_DEBUG_ALARM
      if(k->dev_use==IO_SENSOR) {
          Serial.print("checkSensors:");
          Serial.print(" name=");
          Serial.print(sp.name);
          Serial.print("  tripped=");
          Serial.print(k->tripped);
          Serial.print("  reported=");
          Serial.print(k->reported);
          Serial.print("  reporting:");
          Serial.println(reporting);
      }
    #endif
    if(k->changed || all) {    //DESING: or ALL for 'I' reporting
#ifdef SERIAL_DEBUG_ALARM
      Serial.print("sensor changed:");
      Serial.println(k->name);
#endif
      sensor.addChangedSensor(*k);
    }
  }

  return reporting;
}

void Alarm::clearing_countup() {
  if(allClear()) {
    if(clearing_cnt++ > CLEARING_COUNT_MAX) { //?? count
      //debug("clearing_countup:cnt=%d\n",clearing_cnt);
      prevTripState = FALSE;
      clearing_cnt = 0;
      setState(alarm_clearing);
    }
  }
  else clearing_cnt = 0;
}

//get device by index
device_t* Alarm::getDevice(uint8_t n) {
  std::list<device_t>::iterator k;
  for(k=device_list.begin(); k != device_list.end(); ++k)
    if(k->idx == n) {
      device_t* sp=&(*k);
      return sp;
    }
  return 0;
}
//get device by dev_use
device_t* Alarm::getDeviceByUse(uint8_t use) { //v3.0.1
  std::list<device_t>::iterator k;
  for(k=device_list.begin(); k != device_list.end(); ++k)
    if(k->dev_use == use) {
      device_t* sp=&(*k);
      return sp;
    }
  return 0;
}

// initialize with sensor ???
//Alarm alarm1;

void remoteAlarm(const char *eventName, const char *data) {

  /* format of remotedata alarmData)
  6@1@T@17:07:4848@SEED SENSOR:0:1.0000:1^$
  7@1@T@17:07:5353@ENABLE alarm1:0:1.0000:1^$
  8@1@C@17:07:5757@SEED SENSOR:0:0.0000:0^@ENABLE alarm1:0:0.0000:0^$
  */

  device_t *d;
  String alarmData = String(data);
  //parse this data
  String msgText;
  char temp[15];
  Parse p(alarmData);
  int parse_cnt;
#ifdef SERIAL_DEBUG_REMOTE
  //notify.sendMessage(SHORT_HEADER,1,alarmData);
  Serial.println("Parse alarmData:");
  Serial.print("rData: ");
  Serial.println(p.getData());
#endif

  parse_cnt = p.doParse();
#ifdef DEBUG_PARSE_ELEMENT
  Serial.print("parsed ");
  Serial.print(parse_cnt);
  Serial.println(" items");
  for(int i=0; i< parse_cnt; i++) {
    Serial.println(p.getElement(i));
  }
  Serial.println("end of items");
#endif
  uint8_t system_identification = p.getElement(1).toInt();
  char msgType = p.getElement(2).charAt(0);
  uint8_t setValue = 0;
  //DEBUG shows message received from REMOTE
  if(msgType=='I') msgText = "Info";
  else if(msgType == 'T') {
    msgText = "Tripped";
    setValue = 1; //get from sensor...
  }
  else if(msgType == 'N') msgText = "Notify";
  else if(msgType == 'R') msgText = "Reset";
  else if(msgType == 'C') msgText = "Clear";
  else if(msgType == '?') msgText = "?Unk";
  else msgText = "Unknown";
  // specify which remote here (for multiple remotes), sys.sysId
  sprintf(temp,"remote %d:",system_identification);
  String tmsg = String(temp);
  //tmsg.concat(temp);  //duplicated, remove line
  tmsg.concat(msgText);
//DEBUG...here we need to print out parse_cnt and each element of parse p
#ifdef DEBUG_PARSE_ELEMENT
  //notify.sendMessage(SHORT_HEADER,1,tmsg);
  Serial.println(tmsg);
  Serial.println("sensors:");
#endif
  if(parse_cnt > 3) {
    for(int i=4; i< parse_cnt; i++) {
      Parse ps(FIELD_DELIMITER, FIELD_END, p.getElement(i));
#ifdef DEBUG_PARSE_ELEMENT
      Serial.print("el=");
      Serial.println(p.getElement(i));
#endif
      int sensor_cnt = ps.doParse();
      if(sensor_cnt >= 3) {
          uint8_t masterIdx = ps.getElement(1).toInt();
          float setValue = ps.getElement(2).toFloat();
          uint8_t senValue = ps.getElement(3).toInt();
#ifdef DEBUG_PARSE_ELEMENT1
          Serial.print("remotesen: ");
          Serial.print(ps.getElement(0));
          Serial.print(" midx=");
          Serial.print(masterIdx);
          Serial.print(" val=");
          Serial.print(setValue);
          Serial.print(" sen=");
          Serial.println(senValue);
#endif
          alarm1.setAlarmRemote(system_identification, masterIdx, senValue, setValue);
      }
    }
  }
//  #endif
}

#ifdef PHOTON_MASTER
// TO DO: implemented
// get car temperature from remote sensor, via subscription
// should recieve frequent calls from this sensor when enabled
// remote just sends temperature
// this guy checks minimum temps and performs alerts
// also performs alerts if no alerts are sent after xx min
// standard device enable / disable from iPhone app
void remoteHandler(const char *eventName, const char *data) {
  String temp = String(data);
  char msg[80];
  remote_temp=temp.toFloat();
  alarm1.setLastRemote(SUB_CAR_MONITOR,remote_temp);
  sprintf(msg,"Received Car Temp: %5.1f F", remote_temp);
  notify.queueMessage(SHORT_HEADER,1,msg);
  Serial.printf("remoteHandler: temperature: %fF", remote_temp);
}
// get oil level from remote sensor, via subscription
void oilHandler(const char *eventName, const char *data) {
  Tank tank(HORIZONTAL_275);  //TO DO: needs to be a variable in ???
  String level = String(data);
  char msg[80];
  int level_mm=level.toFloat();
  int level_in=tank.calcOilLevel(level_mm);
  gallonsO = tank.tankGallons(level_in);    //debug exposed cloud variable

  alarm1.setLastRemote(SUB_OIL_GAUGE, gallonsO);
  //alarm1.writeSavedState();  //removed: eeprom ware
  sprintf(msg,"Received Oil Level: %dmm (%din) =%5.1f gallons", level_mm, level_in, gallonsO);
  notify.queueMessage(SHORT_HEADER,0,msg);
  Serial.printf("oilHandler: reveived new level: %f", gallonsO);
}


//gxxxxyy debug to test various oil levels (setoil)
int setTestOil(String newLevel) {
  alarm1.setLastRemote(SUB_OIL_GAUGE, newLevel.toFloat());
}
// initialize notify with alarm1 too ??

//timer use CAUSES build to FAIL ???
//Timer blinkTimer(1000, &Alarm::blinkTimeout, alarm1, TRUE);
//Since timer in class does not build
//void clear_blinking() {
//  alarm1.blinkTimeout();
//}

#endif
//Timer blink2Timer(5000, clear_blinking);

// set time at initialization
void Notifier::setStartTime() {
  hours_between_alert = alarm_saved_state.alert_hours;
  hour=Time.hour();
  lasthour = hour - (hour % hours_between_alert);
  if(lasthour <0) lasthour += 24;
  #ifdef SERIAL_DEBUG
    Serial.print("hour=");
    Serial.println(hour);
    Serial.print("lasthour=");
    Serial.println(lasthour);
    #endif
    lastminute=Time.minute();
}

//NEED a special version for REMOTE ?? xxxxyy
bool Notifier::checkTime() {
  bool check = FALSE;
  String worry_message;
  int tempHour = hour;
  int min_hours_between;
  hour=Time.hour();
  minute=Time.minute();

  // 10  18  18-10 = 8
  elapsedminutes = minute - lastminute;
  //50 10 10 - 50 + 60 = 20
  if(elapsedminutes < 0) elapsedminutes += 60;
  if(elapsedminutes >= WORRY_MINUTES) {
    elapsedminutes = minute;
  }
  min_hours_between = max(1,hours_between_alert);
  tempHour = hour;
  if(tempHour < lasthour) tempHour += 24;
#ifdef PHOTON_MASTER
  if(checkin.panicExpired()) {
    queueMessage(NO_HEADER, 1, "CHECKIN IMMEDIATELY");
    queueMessage(NO_HEADER, 1, "CHECKIN IMMEDIATELY");
    queueMessage(NO_HEADER, 1, "CHECKIN IMMEDIATELY");
  }
  if(!checkin.inPanicMode() && checkin.noticeExpired()) {
    queueMessage(NO_HEADER, 2, CHECKIN_EMERGENCY_MESSAGE);
  }
  //check for checkin timeout every hour
  if(tempHour - lasthour >= 1) {
    if(checkin.checkinThisHour(tempHour)) {
      if(checkin.timeExpired()) {
        queueMessage(NO_HEADER, 1, "CHECKIN NOW");
      }
    }
  }
#endif
  //if hours_between_alert = 4  {4,8,12}
  //FIRST TIME EARLY: LAST_HOUR 3
  //if hours_between_alert is zero, don't alert but do log temp hourly
  #ifdef PHOTON_MASTER
  if(tempHour - lasthour >= min_hours_between) { //NEEDS TO BE ON BOUNDARY
    if(lasthour!=-1){
      if(hours_between_alert != 0) {
         check = TRUE;
         worry_message = updData();
         queueMessage(FULL_HEADER,0,worry_message);
      }
      char tempF[10];
      //CHANGE v3.0 -- read specified thermometer device
      //sprintf(tempF,"%4.1fF",p_sensor->readTemperature());
      sprintf(tempF,"%4.1fF",alarm1.readTemperature(OUTSIDE_THERMOMETER_IDX));
      Particle.publish("temperature2", tempF);
      hourlyReset();
      #ifdef SERIAL_DEBUG
        Serial.print("hour changed! ");
        Serial.print("HOUR:");
        Serial.println(hour);
      #endif
    }
    lasthour = hour;
  }
  #endif
  #ifdef PHOTON_REMOTE
  // DESING?  compare value from where
  // how ofter remote reportz sensor values
  if(elapsedminutes >=  REMOTE_SENSOR_CHECK_MINUTES) {  //testing with 1
    lastminute = minute;
    check = TRUE;
#ifdef SERIAL_DEBUG_REMOTE
    Serial.println("minute tick..");
#endif
  }
  #endif
  return check;
}

// push status message on demand
int Notifier::upd(String command) {
  String msg;
  msg = updData();  //nameing ??
  msg_limit = 0;    //force clear with status
  queueMessage(FULL_HEADER,0,msg);
  return 1;
}
// format common update message
String Notifier::updData() {
  device_t* ts;
  char uptime[40];
  char buf[40];
  //make this a method in Notifier
  //to share with worry
  sys.upTime(uptime);
  String msg = String("\nsys up time: ");
  msg.concat(uptime);
  //remote data
  if(remote_temp > -1000.0) {
    sprintf(buf,"\ncar temp: %5.1fF",remote_temp);
    msg.concat(buf);
  }
  sprintf(buf,"\nworry hours %d",hours_between_alert);
  msg.concat(buf);
#ifdef PHOTON_MASTER
  msg.concat("\ncheckin hours ");
  String hrs = checkin.showCheckinHours();
  msg.concat(hrs.c_str());
  sprintf(buf,"\nremotes %x", sys.getRemotes());
  msg.concat(buf);
#ifdef SERIAL_DEBUG
  String tfmt = checkin.showCheckinTime();
  msg.concat(tfmt.c_str());
  tfmt = checkin.showPanicTime();
  if(tfmt.length() > 1) msg.concat(tfmt.c_str());
  tfmt = checkin.showNoticeTime();
  if(tfmt.length() > 1) msg.concat(tfmt.c_str());
#endif
#endif
  ts = alarm1.firstTripped();
  if(ts != NULL) {
    //SHOULD do all tripped, not just first
    sprintf(buf,"\ntripped sensors:\n");
    strcat(buf,ts->name);
  } else
    sprintf(buf,"\nall sensor clear");
  msg.concat(buf);
  //sprintf(buf,"\nclearing: %d",alarm1.clearingCount());
  //msg.concat(buf);
  return msg;
}
// ver 0.1.8:
// change command order to 1) command, auto receive secret,
// then 2) confirm (execute) command by passing back secret

// request a command to be performed, if format is correct,
// enter command into command_list, keyed by new secret
// new secred is generated and sent, then used as key for
// command_list
// This typdef order must follow order of commands which follow
typedef enum {CMD_HLP, CMD_ABT, CMD_TMP, CMD_CIN, CMD_CAN,
              CMD_PAN, CMD_CIR, CMD_CIS, CMD_CIH, CMD_OFF,
              CMD_SET, CMD_DIS, CMD_ACK, CMD_HOM, CMD_AWA,
              CMD_HOU, CMD_LIS, CMD_NAM, CMD_SEN, CMD_ACT,
              CMD_DEA, CMD_MIN, CMD_MAX, CMD_EVT, CMD_CFG,
              CMD_SAV, CMD_ROF, CMD_RON};
const String Notifier::commands =
      String("HLP.ABT.TMP.CIN.CAN.PAN.CIR.CIS.CIH.OFF.SET.DIS.ACK.HOM.AWA.HOU.LIS.NAM.SEN.ACT.DEA.MIN.MAX.EVT.CFG.SAV.ROF.RON.");
//            0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27
#define COMMAND_NO_CONFIRM 4      //THESE COMMANDS DO NOT REQUIRE A CONFIRMATION
#define COMMAND_ZERO_CONFIRM 5    //THESE FORCE A ZERO CONFIRMATION NUMBER

int Notifier::request(String cmd_line) {
  char msg[20];
  int cidx;
  int new_secret;
  String delim = String(".");
  //sscanf (cmd_line,"%s %s %s %s",msg,args1,args2,args3);
  sscanf (cmd_line,"%s",msg);
  String parsed_cmd = String(msg);
  parsed_cmd.toUpperCase();
  parsed_cmd = parsed_cmd.substring(0,3);
  parsed_cmd.concat(delim);
  Serial.print("parsed cmd:");
  Serial.println(parsed_cmd);
  cidx=commands.indexOf(parsed_cmd);
  if(cidx==-1) {
    // invalid commands, message it
    return 0;
  }
  else cidx /=4;
  //execute command immediately without confirmation
  if(cidx <= COMMAND_NO_CONFIRM) {
      return do_command(cmd_line);
  }
  else if(cidx <= COMMAND_ZERO_CONFIRM) {
    new_secret = 0;
  }
  else new_secret = random(RANDOM_MIN,RANDOM_MAX);
// enter into command_list
  command_list[new_secret] = cmd_line;
// message secret to confirm
  sprintf(msg, " Confirmation: %d", new_secret);
  //int tpri = 2;
  //sprintf(msg, "Confirmation:\"%d,\"priority\":%d", new_secret, tpri);  //\n
  queueMessage(NO_HEADER,-1,msg);
  return 1;
}

#ifdef DEBUGXX
//debug dump of command list
int Notifier::dump(String t) {
  char buf[40];
  char msg[400];
  std::map <int, String>::iterator mi;
  if(command_list.empty())
      queueMessage("\ncommand_list is empty");
  else {
      strcpy(msg,"\ncommand_list:");
      for(mi=command_list.begin();mi != command_list.end(); ++mi) {
        sprintf(buf,"\ncommand: %s secret: %d", mi->second.c_str(), mi->first );
        strcat(msg,buf);
      }
      queueMessage(msg);
  }
  return 1;
}
#endif

// perform command queued earliey, keyed by secret
// lookup command by secret in command_list
// if secrets match, i.e. lookup worked, then perform
// command using set()
int Notifier::confirm(String secret) {
  char buf[60];
  std::map <int, String>::iterator mi;
  int secret_index = secret.toInt();
  if(bad_secret_cnt >= BAD_SECRET_MAX) {
    queueMessage(NO_HEADER,0,"TOO MANY BAD SECRETS");  //debug message
    command_list.clear();
    return 0;
  }
  for(mi=command_list.begin();mi != command_list.end(); ++mi) {
    if(mi->first == secret_index) {
      int res = do_command(mi->second);
      command_list.erase(secret_index);
      return res;
    }
  }
  // this is really bad secret
  sprintf(buf, "\nInvalid Confirmation #: %d\n", secret_index );
  bad_secret_cnt++;
  queueMessage(NO_HEADER,0,buf);
  return 0;
}

// set alarm state remotely
int Notifier::do_command(String cmd_line) {
  int evtidx;
  int cidx;
  int p1,p2,p3;
  int i,h,n;
  char msg[50];
  char args1[20];
  char args2[20];
  char listing_buffer[400];   //size ????
  String dev_msg; //debug testing
  String cmd_line_rest;
  device_t *d;
  String delim = String(".");
  if(bad_secret_cnt >= BAD_SECRET_MAX) {
    queueMessage(NO_HEADER,1,"TOO MANY BAD SECRETS");  //debug message
    return 0;
  }
  args1[0]=0;
  args2[0]=0;
  sscanf (cmd_line,"%s %s %s",msg,args1,args2);
  i = cmd_line.indexOf(" ");
  cmd_line_rest = cmd_line.substring(i+1); //with 'cmd' removed
  cmd_line_rest = cmd_line_rest.trim();
  i = cmd_line_rest.indexOf(" ");
  cmd_line_rest = cmd_line_rest.substring(i+1); //with p1 removed
  cmd_line_rest = cmd_line_rest.trim();

#ifdef SERIAL_DEBUGXX
  Serial.printf("set: cmd=%s args1=%s args2=%s\n",msg,args1,args2);
#endif
  String parsed_cmd = String(msg);
  parsed_cmd.toUpperCase();
  parsed_cmd = parsed_cmd.substring(0,3);
  parsed_cmd.concat(delim);
  //Serial.print("parsed cmd:");
  //Serial.println(parsed_cmd);
  cidx=commands.indexOf(parsed_cmd);
  if(cidx==-1) {
    //debug w/ cmd
    sprintf(msg, "INVALID COMMAND:%s", parsed_cmd.c_str());
    queueMessage(NO_HEADER,1,msg);
    return 0;
  }
  else cidx /=4;
  String arg_list1 = String(args1);
  String arg_list2 = String(args2);
  String arg_list2_4 = String(args2);
  p1 = arg_list1.toInt();
  n = arg_list1.indexOf(' ');
  p2 = arg_list2.toInt();
#ifdef SERIAL_DEBUGXX
  Serial.print("p1=");
  Serial.print(p1);
  Serial.print(" p2=");
  Serial.println(p2);
  Serial.printf("cmd index=%d\n",cidx);
  Serial.printf("secret=%d\n",p1);
  Serial.printf("arg=%d\n",p2);
#endif
  switch(cidx) {
    case CMD_HLP : // Command HELP
      strcpy(listing_buffer,"\nCommands:");
      strcat(listing_buffer,"\nABT - About");
      strcat(listing_buffer,"\nHLP - This Help");
      strcat(listing_buffer,"\nSET On");
      strcat(listing_buffer,"\nDISable");
      strcat(listing_buffer,"\nHOMe");
      strcat(listing_buffer,"\nAWAy");
      strcat(listing_buffer,"\nACKnowledge n");
      strcat(listing_buffer,"\nTMPerature");
      strcat(listing_buffer,"\nHOU btw Worry to n");
      queueMessage(NO_HEADER,1,listing_buffer);

      strcpy(listing_buffer,"\nLISt Sensors");
      strcat(listing_buffer,"\nNAMe Sensor n");
      strcat(listing_buffer,"\nSENsor Detail");
      strcat(listing_buffer,"\nACTivate sensor n");
      strcat(listing_buffer,"\nDEActivate sensor n");
      strcat(listing_buffer,"\nEVT - event list");
      strcat(listing_buffer,"\nMAXimum Alert Temp to x");
      strcat(listing_buffer,"\nMAXimum Alert Temp to x");
      strcat(listing_buffer,"\nCFG - gen device config");
      queueMessage(NO_HEADER,1,listing_buffer);
      break;
    case CMD_ABT : // ABT: CHANGED
      strcpy(listing_buffer,"photonAlarm\n");
      strcat(listing_buffer,"\ncopyright (c) re:Engineering 2016");
      strcat(listing_buffer,"\nDonald Thompson");
      sprintf(msg,"\nVersion %d.%d.%d", SYSTEM_VERSION_MAJOR, SYSTEM_VERSION_MINOR, SYSTEM_VERSION_SUB);
      strcat(listing_buffer, msg);
      queueMessage(NO_HEADER,0,listing_buffer);
      break;
    case CMD_TMP : //TMP Display All Thermometers
        alarm1.thermometerListing(listing_buffer);
        queueMessage(NO_HEADER, 1, listing_buffer);
        return 1;
        break;
    case CMD_CIN :  //check-in
        // PERFOMR user checkin, i.e. cancal pending 'panic'
        checkin.userCheckin();
        // should have return status and show if completed or too late ??
        queueMessage(NO_HEADER,0,"CHECKIN COMPLETED");
        return 1;
        break;
    case CMD_CAN :  //cancel 'panic' and all other pending commands
        command_list.erase(0);
        queueMessage(NO_HEADER,1,"COMMAND CANCELLED");
        return 1;
        break;
    case CMD_PAN :  //PANIC: send immediate message to emergancy contact list (pri=2)
        queueMessage(SHORT_HEADER,2,"PANIC BUTTON -- CONTACT IMMEDIATELY");
        return 1;
        break;
    case CMD_CIR :  //checkin reset, go out of panic mode
        checkin.reset();
        queueMessage(SHORT_HEADER,0,"CHECKIN REENABLED");
        return 1;
        break;
  case CMD_CIS :  //SUSPEND CHECKINS for XX HOURS
        checkin.setSuspended(p1); //suspend for 12 hours
        queueMessage(SHORT_HEADER,0,"CHECKIN SUSPENDED");
        return 1;
        break;
    case CMD_CIH :  //set or clear checkin hours
        if(p1 > 23 || p1 < -23 || p1==0) return 0;
        checkin.addCheckinHour(p1);
        queueMessage(NO_HEADER,1,"CHECKIN HOUR SET");
        alarm_saved_state.checkin_hours = checkin.getCheckinHours();
        //alarm1.writeSavedState();
        return 1;
        break;
    case CMD_SET : // SET Alarm state to enabled
        //command = 'SET <secret>'
          // set unless tripped
        if(alarm1.setState(alarm_armed)) {
            //alarm1.writeSavedState();
            #ifdef SERIAL_DEBUG
              Serial.print('set armed');
            #endif
            queueMessage(SHORT_HEADER, alarm1.getPriority(),alarm1.getPendingMessage());
        }
        return 1;
        break;
    case CMD_DIS : // Disarm Alarm if Secret matches
        //command = 'DISARM <secret>'
        // DISABLE alarm here!!
        if(alarm1.setState(alarm_disarmed)) {
          //alarm1.writeSavedState();
          queueMessage(SHORT_HEADER,alarm1.getPriority(),alarm1.getPendingMessage());
        }
        return 1;
        break;
    case CMD_OFF : // Shut Alarm Off if Secret matches
        if(alarm1.setState(alarm_off)) {
          //alarm1.writeSavedState();
          queueMessage(SHORT_HEADER,alarm1.getPriority(),alarm1.getPendingMessage());
        }
        return 1;
        break;

    case CMD_ACK : // Acknowledge tripped Alarm
        //TODO: REDO THIS
        //xx find sequence specified in ACK message and Acknowledge
        //xx remove from event_list
        //if event_list empty, reset alarm
        // clear blink state
        evtidx=alarm1.ackEvent(p1);
        if(evtidx != -1) {
          #ifdef SERIAL_DEBUG_EVENT
          Serial.print("ACKed event# ");
          Serial.print(p1);
          Serial.print(" dev idx=");
          Serial.println(evtidx);
          #endif
          // here we clear things for device idx ??
          //event_list.erase(seq);
          if(!alarm1.devIsTripped(evtidx)) {
            alarm1.removeEvent(p1);
            //clear device
            alarm1.resetDevice(evtidx);
            //message that acked ok
            sprintf(msg,"event # %d cleared", p1);
          }
          else {
            //message on ACK...dev still tripped
            sprintf(msg,"event # %d NOT cleared, dev tripped", p1);
          }
        }
        else {
          sprintf(msg,"event # %d not found", p1);
        }
        #ifdef SERIAL_DEBUG_EVENT
        Serial.println(msg);
        #endif

        queueMessage(FULL_HEADER,1,msg);
        if(alarm1.eventsClear()) {
          alarm1.setState(alarm_armed);
          queueMessage(SHORT_HEADER, 1, "Alarm Reset");
        }
        //if(alarm1.clearSensorReported())
        /*
        else
          queueMessage(FULL_HEADER,1,"SENSOR STILL TRIPPED");
        queueMessage(SHORT_HEADER, alarm1.getPriority(),alarm1.getPendingMessage());
        */
        return 1;
        break;
    case CMD_HOM : // SET alarm mode to at HOME
        if(alarm1.getCurLocation() != AT_HOME) {
            alarm1.setCurLocation(AT_HOME);
            stateA = alarm1.getStateDescription();
            Particle.publish("alarmState",stateA,60);
            queueMessage(NO_HEADER,1,"SET to HOME");
        }
        //AND SET
        if(alarm1.setState(alarm_armed)) {
            //alarm1.writeSavedState();
            #ifdef SERIAL_DEBUG
              Serial.print('set armed');
            #endif
            queueMessage(SHORT_HEADER, alarm1.getPriority(),alarm1.getPendingMessage());
        }
        break;
    case CMD_AWA : // SET alarm mode to AWAY
        if(alarm1.getCurLocation() != AWAY) {
            alarm1.setCurLocation(AWAY);
            stateA = alarm1.getStateDescription();
            Particle.publish("alarmState",stateA,60);
            queueMessage(NO_HEADER,1,"Set to AWAY");
        }
        //AND SET
        if(alarm1.setState(alarm_armed)) {
            //alarm1.writeSavedState();
            #ifdef SERIAL_DEBUG
              Serial.print('set armed');
            #endif
            queueMessage(SHORT_HEADER, alarm1.getPriority(),alarm1.getPendingMessage());
        }
        break;
    case CMD_HOU : // SET alert Hours if Secret matches
          //command = 'HOUR <hours> <secret>'
          //sscanf (cmd_line,"%s %d %d",msg,&i, &h);
          // set hours_between_alert if valid
          if(p1 >= 0 and p1 <= 12) {
              setAlertHours(p1);
              if(p1)
                sprintf(msg, "HOURS BTW WORRY SET TO %d", p1);
              else
                sprintf(msg, "WORRY ALERTS DISABLED");
              queueMessage(SHORT_HEADER,1,msg);
              #ifdef SERIAL_DEBUG
                Serial.print('seting hours_between_alert');
              #endif
          } else
            queueMessage(NO_HEADER,1,"INVALID ALERT HOURS");
          return 1;
          break;
    case CMD_LIS : // List all Devices
          //command = 'LIST <secret>'
          // LIST ALL Sensor Devices
          alarm1.deviceListing(listing_buffer);
          queueMessage(NO_HEADER, 1, listing_buffer);
          return 1;
          break;

    case CMD_NAM : // Name a Sensor
          //command = 'NAM <secret> <sen#> <name>'
          msg[0]=0;
          // NAME a Sensor Devices
          d = alarm1.getDevice(p1);
          if(d) {
              d->name = cmd_line_rest;
              //now update configuration in EEPROM
              alarm1.updateConfiguration(d, p1);
              sprintf(msg, "Rename Sen# %d to %s", p1, cmd_line_rest.c_str());
          }
          else
              sprintf(msg, "NO SENSOR# %d", p1);
          queueMessage(SHORT_HEADER, 1,msg);
          return 1;
          break;
    case CMD_SEN : // List a Devices in Full
          //command = 'SEN <sen#>'
          listing_buffer[0]=0;
          d = alarm1.getDevice(p1);
          if(d) {
              sprintf(listing_buffer,"\nIndex: %d",d->idx);
              strcat(listing_buffer,"\nrom: ");
              char buf[26];
              p_sensor->romFormat(&buf[0], d->dev_rom);
              strcat(listing_buffer,buf);
              strcat(listing_buffer,"\nStatus: ");
              strcat(listing_buffer,sensor_status_def[d->status]);
              strcat(listing_buffer,"\nState: ");
              strcat(listing_buffer,sensor_state_def[d->state]);
              strcat(listing_buffer,"\nUse: ");
              strcat(listing_buffer,sensor_use_def[d->dev_use]);
              strcat(listing_buffer,"\nSense: ");
              strcat(listing_buffer,sensor_sense_def[d->sense]);
              strcat(listing_buffer,"\nAlert: ");
              strcat(listing_buffer,sensor_level_def[d->alert_level]);
              if(d->alert_min) {
                sprintf(buf,"\nAlert Min: %d", d->alert_min);
                strcat(listing_buffer, buf);
              }
              if(d->alert_max) {
                sprintf(buf,"\nAlert Max: %d", d->alert_max);
                strcat(listing_buffer, buf);
              }
              strcat(listing_buffer,"\nName: ");
              strncat(listing_buffer,d->name,SENSOR_NAME_SIZE);
              strcat(listing_buffer,"\nTripped: ");
              if(d->tripped)
                  strcat(listing_buffer,"Y");
              else
                  strcat(listing_buffer,"N");
              strcat(listing_buffer,"\nReported: ");
              if(d->reported)
                  strcat(listing_buffer,"Y");
              else
                  strcat(listing_buffer,"N");
              sprintf(buf,"\nReading: %5.2f",d->dev_reading);
              strcat(listing_buffer,buf);
              if(d->dev_last_read > Time.now() - 3600 * 24) {
                strcat(listing_buffer,"\nLast Read: ");
                strcat(listing_buffer,Time.format(d->dev_last_read, " %I:%M%p."));
              }
          }
          else
              sprintf(listing_buffer, "NO SENSOR# %d", p1);
          dev_msg = String(listing_buffer);
          //queueMessage(1,listing_buffer);
          queueMessage(NO_HEADER,1,dev_msg);
          return 1;
          break;
      case CMD_ACT : // Activate a sensor
          //command = 'ACT <sen#>'
          //TO DO: only set if present ??
          d = alarm1.getDevice(p1);
          if(d) {
              sensor.setDeviceActive(d, TRUE);
              //sensor.readTemperature();
              sprintf(msg, "SENSOR ACTIVATED: %d", p1);
          }
          else
              sprintf(msg, "NO SENSOR# %d", p1);
          queueMessage(SHORT_HEADER,1,msg);
          return 1;
          break;
      case CMD_DEA : // Deactivate a sensor
          //command = 'DEA <secret> <sen#>'
          d = alarm1.getDevice(p1);
          if(d) {
              sensor.setDeviceActive(d, FALSE);
              sprintf(msg, "SENSOR DEACTIVATEDxx: %d", p1);
          }
          else
              sprintf(msg, "NO SENSOR# %d", p1);
          queueMessage(SHORT_HEADER,1,msg);
          return 1;
          break;
      case CMD_MIN : // Set Sensor Minimum Alarm Temperature
          //command = 'MIN <sen#> <min>'
          d = alarm1.getDevice(p1);
          if(d) {
              alarm1.setDeviceAlertMin(d, p2);
              sprintf(msg, "SENSOR# %d ALERT MINX : %d", p1, p2);
          }
          else
              sprintf(listing_buffer, "NO SENSOR# %d", p1);
          queueMessage(SHORT_HEADER,1,msg);
          return 1;  //ret values ??
          break;

      case CMD_MAX : // Set Sensor MAXimum Alarm Temperature
          //command = 'MAX <sen#> <min>'
          d = alarm1.getDevice(p1);
          if(d) {
              alarm1.setDeviceAlertMax(d, p2);
              sprintf(msg, "SENSOR# %d ALERT MAX : %d", p1, p2);
          }
          else
              sprintf(listing_buffer, "NO SENSOR# %d", p1);
          queueMessage(SHORT_HEADER,1,msg);
          return 1;  //ret values ??
          break;

      case CMD_EVT : // Display Event List
          //command = 'EVT'
          alarm1.eventListing(listing_buffer);
          queueMessage(NO_HEADER, 1, listing_buffer);
          return 1;

          break;
      case CMD_CFG : // Auto-Configure Sensors
              //command = 'CFG'
              // To be Implemented...
              //1) scan 1-wire sensors and I2C devices
              //2) build config of all devices found into test-configuration
              //3) for devices found in running configuration,
              //   copy device detail (name, etc.) from running to test
              //4) display new configuration to user (Pushover alert)
              //5) send user another confirmation number
              //6) upon valid confirmation,
              //   copy test-configuration to configuration (in EEPROM)
              //7) restart alarm_state
              strcpy(msg, "CFG Not Implemented");
              queueMessage(NO_HEADER,1,msg);
          break;
      case CMD_SAV : // Write alarm_saved_state to EEPROM
          if(alarm1.writeSavedState()) queueMessage(NO_HEADER, 0, "STATE SAVED");
          return 1;
          break;
      case CMD_ROF : // disable ALL remotes, or remote p1
          sys.setRemoteStatus(p1,FALSE);
          alarm1.tellRemote(p1,remotes_disabled);
          //BUG: fix message if p1 <> 0
          queueMessage(SHORT_HEADER,0,"REMOTES DISABLED");
          return 1;
          break;
      case CMD_RON : // enable ALL remotes, or remote p1
          sys.setRemoteStatus(p1,TRUE);
          alarm1.tellRemote(p1,remotes_enabled);
          //BUG: fix message if p1 <> 0
          queueMessage(SHORT_HEADER,0,"REMOTES ENABLED");
          return 1;
          break;
      default:
          queueMessage(NO_HEADER,1,"UNRECOGNIZED COMMAND");
          return 0;
  }
  return 1;
}

// return TRUE if found AND tripped
bool Alarm::devIsTripped(int idx) {
  // too bad device_list is NOT a map!!!
  std::list<device_t>::iterator k;
  //bool all_clear = TRUE;
  for(k=device_list.begin(); k != device_list.end(); ++k) {
    if(k->idx != idx) continue;
    return k->tripped;
  }
  return FALSE; //??
}

void Alarm::resetDevice(uint8_t idx) {
  std::list<device_t>::iterator k;
  for(k=device_list.begin(); k != device_list.end(); ++k) {
    if(k->idx != idx) continue;
    if(!k->tripped) k->reported = FALSE;
  }
}


// add event to event_list and return sequence
int Alarm::addEvent(uint8_t idx) {
  std::map <int, uint8_t>::iterator ei;
  if(!event_list.empty()) {  //??WHY THIS line ?? empty ??
    //first make sure this device is not already in event list (by idx)
    for(ei=event_list.begin();ei != event_list.end(); ++ei) {
      if(ei->second == idx) {
        #ifdef SERIAL_DEBUG_EVENT
        Serial.print("addEvent: found:idx=");
        Serial.println(idx);
        #endif
        return 0;  // found it here already
      }
    }
  }
  alarm_saved_state.event_sequence = ++event_sequence;
  #ifdef SERIAL_DEBUG_EVENT
  Serial.print("addEvent: event_sequence=");
  Serial.print(event_sequence);
  #endif
  //don't keep evt due to eeprom ware
  //alarm1.writeSavedState();
  event_list[event_sequence] = idx;
  #ifdef SERIAL_DEBUG_EVENT
  Serial.print("added dev idx=");
  Serial.println(idx);
  #endif
  return event_sequence;
}

// find event by sequence, return device idx or -1
int Alarm::ackEvent(int seq) {
  std::map <int, uint8_t>::iterator ei;
  if(event_list.empty()) return -1;
  for(ei=event_list.begin();ei != event_list.end(); ++ei) {
    if(ei->first == seq) {
      //int res = do_command(mi->second);
      //TODO: where does device reported get cleared ???
      //check if still tripped! ==> return -1
      #ifdef SERIAL_DEBUG_EVENT
      Serial.print("ackEvent found:id=");
      Serial.println(seq);
      #endif
      return ei->second;
    }
  }
  return -1;
}

//remove event for event_list
int Alarm::removeEvent(int seq) {
  //make sure it exists
  if(event_list.find(seq) == event_list.end()) return -1;
  event_list.erase(seq);
  return seq;
}

bool Alarm::eventsClear() {
  return event_list.empty();
}

//function to count down secret_timeout, and clear secret
void Notifier::secret_countdown() {
  if(!secret_timeout--) secret = 0;
}
void Notifier::setAlertHours(int hours) {
  lasthour = Time.hour() - 1;
  if(lasthour < 0) lasthour = 23;
  hours_between_alert = hours;
  //alarm1.writeSavedState();  //use CMD SAV to write to eeprom
}
uint8_t Notifier::getAlertHours() {
  return hours_between_alert;
}

bool Notifier::msgqueueEmpty() {
    return message_queue.empty();
}
void Notifier::queueMessage(uint8_t header, int pri, String msg) {
  pMessage message;
  message.header_style = header;
  message.priority = pri;
  message.message_text = msg;
  message_queue.push(message);
}
void Notifier::hourlyReset() {
  bad_secret_cnt = 0;
  msg_limit = 0;
  digitalWrite(MESSAGE_PIN, LOW);
  if (millis() - lastSync > ONE_DAY_MILLIS) {
    // Request time synchronization from the Particle Cloud
    Particle.syncTime();
    lastSync = millis();
  }
}
void Notifier::dequeMessage() {
  pMessage next_message;
  if(!message_queue.empty()){
    next_message = message_queue.front();
    //don't flood messages, but if they are not sent\
    //here we need to try again when hour changes...
    //just call dequeMessage..it checks if empty
    //Serial.print(" not empty");
    if(msg_limit++ < MAX_MESSAGE_PER_HOUR) {
      message_queue.pop();
      sendMessage(next_message.header_style, next_message.priority, next_message.message_text);
    }
    else
      digitalWrite(MESSAGE_PIN, HIGH);
  }
#ifdef SERIAL_DEBUGXX
  else {
    Serial.print("queue empty!");
  }
  Serial.println(" ");
#endif
}

void Notifier::sendMessage(uint8_t hdr, int pri, String msg) {
  //TO DO: shorten tmessage to 200 ?? messages can not be this long
  char tmessage[620];  //debug testing size limit / was 600
  strncpy(tmessage, msg.c_str(), 618);
  tmessage[619]=0;
  sendMessage(hdr, pri, tmessage);
}

// fixed double-backslash - cr
void Notifier::sendMessage(uint8_t hdr, int pri, char* msg) {
#ifdef SERIAL_DEBUG
  Serial.print("sendMessage: pri=");
  Serial.print(pri);
#endif
  event_message[0] = 0;
  if(hdr != NO_HEADER) {
    String alarmStateName = alarm1.getStateDescription();
    String time = Time.format("%m/%d/%y %H:%M:%S");
    sprintf(event_message,"%s %4.1fF\n%s",time.c_str(),
      //p_sensor->readTemperature(),
      //CHANGED THERMOMETER to specified device
      alarm1.readTemperature(OUTSIDE_THERMOMETER_IDX),
      //here we need to add HOME / AWAY
      alarmStateName.c_str());
      //alarm_state_name_def[alarm1.getState()]);
    strcat(event_message," ");
  }
  if(hdr==FULL_HEADER) {
      if(alarm1.isTripped()) {
        strcat(event_message," sensors tripped:\n");
        device_t* ts;
        /*------------------must do in Alarm::
        std::list<device_t>::iterator k;
        bool all_clear = TRUE;
        for(k=device_list.begin(); k != device_list.end(); ++k) {
           //device_t sp = *k;
           if(k->reported) {
        ------------------*/
        //NULL if trippedList empty
        //iterate  thru trippedList
        ts = alarm1.firstTripped();
        if(ts != NULL) {
          strncat(event_message,ts->name,12);
          strcat(event_message," ");
        }
      }
  }

  strcat(event_message, msg);
//testing priority as webhook parameters
//currently ONLY IMPLEMENTED in webhook elmst-alarm as MESSAGE_PRIORITY
//DOES NOT WORK
/*
String mdata = String::format(
  "{\"message\":\"%s\", \"priority\":%d}",
  event_message, pri);
*/
#ifdef PUSHOVER_SEND_MESSAGES
  //  SELECT webhook_id based upon priority of message
  // TODO - make  webhook_id an array rather than this switch
  switch(pri) {
    case -1:  //with no sound or viberate
         Spark.publish(low_pri_webhook_id, event_message, 60, PRIVATE);
         break;
    case 0:
          Serial.print(" publish ");
          Spark.publish(webhook_id, event_message, 60, PRIVATE);
          //Spark.publish("Elmst", mdata, 60, PRIVATE);
          break;
    case 1:
          Serial.print(" publish w/ priority ");
          Spark.publish(priority_webhook_id, event_message, 60, PRIVATE);
          //testing WEBHOOK parameter
          //Spark.publish(webhook_id, event_message, 60, PRIVATE);
          //Spark.publish("Elmst", mdata, 60, PRIVATE);
          break;
    case 2:
          Serial.print(" publish emergency ");
          Spark.publish(emergency_webhook_id, event_message, 60, PRIVATE);
          break;
  //  default:
  }
#endif
#ifdef SERIAL_DEBUG
  Serial.print("msg:");
  Serial.println(event_message);
#endif
}
void Notifier::pushAlarm(char* msg) {
  //this can include global info about
  //what is tripped ??
  if(alarm_times_notified++ < MAX_ALARM_NOTIFIED) {
    event_message[0] = 0;
    strcat(event_message,Time.timeStr().c_str());

    //  strcat(event_message, name of alarm in trip); ??

    //strcat(event_message, msg);
    //publish should be to another webhook, ie webhook_alarm
    //and set quite times different for this hook
    //Spark.publish now returns a bool to indicate success!!!
    Spark.publish(webhook_id, event_message, 60, PRIVATE);
  }
  else
    digitalWrite(MESSAGE_PIN, HIGH);
}

#ifdef PHOTON_REMOTE
//-------------------
//message from master to remote
void masterHandler(const char *eventName, const char *data) {

  /* format of masterdata
  6@1@T@17:07:4848$
  */
  device_t *d;
  String masterData = String(data);
  //parse this data
  String msgText;
  char temp[15];
  Parse p(masterData);
  int parse_cnt;
#ifdef SERIAL_DEBUG_REMOTE
  Serial.println("Parse masterdata:");
  Serial.print("mData: ");
  Serial.println(p.getData());
#endif
  parse_cnt = p.doParse();
#ifdef DEBUG_PARSE_ELEMENT
  Serial.print("parsed ");
  Serial.print(parse_cnt);
  Serial.println(" items");
  for(int i=0; i< parse_cnt; i++) {
    Serial.println(p.getElement(i));
  }
  Serial.println("end of items");
#endif
  uint8_t system_identification = p.getElement(1).toInt();
  //is message for me?
  if(system_identification != 0 && system_identification != sys.getSysId()) return;
  char msgType = p.getElement(2).charAt(0);
#ifdef SERIAL_DEBUG_REMOTE
  Serial.print("mstType:");
  Serial.print(msgType);
  Serial.print("system_id:");
  Serial.println(system_identification);
#endif
  //here we set LEDs based on msgType
  alarm1.setLEDRemote(msgType);
}
#endif

void reset_handler()
{
    // tell the world what we are doing
    Particle.publish("reset", "going down for reboot NOW!");
    notify.sendMessage(NO_HEADER,1,"SYSTEM REBOOTING");
}
void watchdog_reset()
{
    // tell the world what we are doing, doesn't work ??
    // BUG: does reset but not notifications
    Particle.publish("reset", "Watchdog timed out, reset!");
    notify.sendMessage(NO_HEADER,1,"WATCHDOG SYSTEM RESET");
    System.reset();
}

//define our own function so we can send alert on reset
ApplicationWatchdog wd(60000, watchdog_reset);
//ApplicationWatchdog wd(60000, System.reset);
#ifdef HARDWARE_WATCHDOG
HWD hwd;
#endif

void setup() {
  // register the reset handler
  System.on(reset, reset_handler);
#ifdef PHOTON_MASTER
  Particle.variable("state", stateA);
  Particle.variable("temp", temperatureF);
  Particle.variable("oilgal", &gallonsO, DOUBLE);
  Particle.variable("temprmt", remote_temp);
  Particle.subscribe("oillevel", oilHandler);
  Particle.subscribe("remotetemp", remoteHandler);  //use this for remote xxxxyy
  Particle.function("setoil", setTestOil);
  //Particle.function("remotealarm", remoteAlarm);
  Particle.subscribe("remotedata", remoteAlarm);
#endif
#ifdef PHOTON_REMOTE
    Particle.variable("rstate", stateA);
    Particle.variable("remotedata", remoteData);
    Particle.subscribe("masterdata", masterHandler);
//??    Particle.variable("testdata", rtestdata);
#endif
  int newseed = 0;
  remote_temp = -1002.00;
  random_seed_from_cloud(newseed); //?? usage ??
  // HERE we need to init any ports defined in sensor list ??
  //design??
  pinMode(D5, INPUT_PULLUP);
  pinMode(D6, INPUT_PULLUP);

  pinMode(ledStatus, OUTPUT);
  pinMode(ledNotice, OUTPUT);
  digitalWrite(MESSAGE_PIN, HIGH);
  //sys.sysState(sys_fail);  //<<<===REMOVE, DEBUG!!!

  pinMode(trigger3, OUTPUT);
  sys.sysState(sys_undefined);
#ifdef SERIAL_DEBUG
    Serial.begin(9600);
  #ifdef SERIAL_WAIT
    while(!Serial.available())  // Wait here until the user presses ENTER
      Spark.process();          // in the Serial Terminal. Call the BG Tasks
    Serial.read();
  #endif
#endif
    //delay(1000);
    Serial.printf("\nPhotoAlarm initializing...Version %d.%d.%d",
          SYSTEM_VERSION_MAJOR, SYSTEM_VERSION_MINOR, SYSTEM_VERSION_SUB);
    //checking to ensure Pushover is working
    //notify.sendMessage(NO_HEADER,1,"SYSTEM Initializing1");
    //notify.sendMessage(NO_HEADER,2,"SYSTEM Initializing2");
    sys.sysState(sys_starting);
//    digitalWrite(trigger3, HIGH); //DEBUG, trigger pin3 for logic 8 analyzer
    Time.zone(EST_OFFSET);
    uint8_t stat = mcp9808.begin(MCP9808_I2CADDR);  //bus master
    Serial.print("mcp stat=");
    Serial.println(stat);
#ifdef HARDWARE_REQUIRED_MSCP9808
    //non-zero = error, not mcp9808
    if(stat) {
      Serial.println("FAIL");
      sys.addStatus(fail_hardware);
      notify.sendMessage(NO_HEADER,1,"MCP9808 FAIL");
#ifdef HALT_ON_HDW_ERROR
      sys.sysState(sys_fail); //this will do HANG
#endif
    }
#endif
    ow.reset();
    if(!alarm1.readSavedState()) {
      alarm_saved_state.current_state = alarm_disarmed;
      alarm_saved_state.event_sequence = 1000;
#ifdef PHOTON_MASTER
      alarm_saved_state.alert_hours = ALERT_HOURS;
      alarm_saved_state.checkin_hours = CHECKIN_HOURS;
#endif
      alarm1.writeSavedState();
    }
    else {
      #ifdef SERIAL_DEBUG_EVENT
      Serial.print("readSavedState: event_sequence=");
      Serial.println(alarm_saved_state.event_sequence);
      #endif
    }
    notify.setAlertHours(alarm_saved_state.alert_hours);
#ifdef PHOTON_MASTER
    checkin.setCheckinHours(alarm_saved_state.checkin_hours);
    //$$$$$$$
    //set remotes
    alarm1.setRemotes(alarm_saved_state.remote_state);
#endif
    if(!alarm1.readConfiguration()) {
      notify.queueMessage(SHORT_HEADER,1,"FAILED");
      #ifdef SERIAL_DEBUG
          Serial.println("EEPROM Config Invalid");
      #endif
      if(alarm1.writeTestConfiguration()) {
        notify.queueMessage(SHORT_HEADER,1,"WRITECONFIG");
        #ifdef SERIAL_DEBUG
            Serial.println("EEPROM WriteConfig");
        #endif
      }
      sys.addStatus(fail_eeprom);
      notify.sendMessage(NO_HEADER,1,"CFG FAIL");   //send this message now
      #ifdef SERIAL_DEBUG
          Serial.println("CFG FAIL--HALT");
      #endif
      sys.sysState(sys_fail); //this will do HANG
    }
    //build device list from read configuration
    if(alarm1.buildDeviceList()) {
      alarm1.dump_device_list();
    }
    else notify.queueMessage(SHORT_HEADER,1,"NODEVICE");

// DEBUG: WHAT ARE REG MACRO VALUES?
Serial.print("D4=");
Serial.println(D4);
Serial.print("A0=");
Serial.println(A0);
    // check DS2482 status and config to confirm presence
    ow.reset();
    stat = ow.wireReadStatus(FALSE);
    #ifdef SERIAL_DEBUG
    Serial.print("DS2482 Reset - Status Reg:");
    Serial.println(stat, HEX);  //sb 0x18 (RST=1 AND 1-WIRE LL HIGH?)
    #endif
    if(stat != 0x18) {
#ifdef SERIAL_DEBUG
        Serial.println("HDW FAIL-missing DS2482");
#endif
    sys.addStatus(fail_ds2482);
#ifdef HARDWARE_REQUIRED_DS2482
      notify.sendMessage(NO_HEADER,1,"HDW FAIL DS2482");   //send this message now
#ifdef HALT_ON_HDW_ERROR
      sys.sysState(sys_fail);           //and halt
#endif
#endif
    }
    stat = ow.wireReadConfig();
    #ifdef SERIAL_WAIT
    Serial.print("Config Reg:");
    Serial.println(stat, HEX);
      while(!Serial.available())  // Wait here until the user presses ENTER
        Spark.process();          // in the Serial Terminal. Call the BG Tasks
      Serial.read();
    #endif
    // CHECK / COMPARE devices from config to
    // actual devices discovered
    // and mark PRESENT OR notify errors
    //CHECK FOR OW PRESENCE IF OW DEVICES IN CONFIGURE ???
    if(sys.sysStatus()==0 && sys.owPresent()) {
      if(ow.wireReset()) {
        #ifdef SERIAL_DEBUGXX
          Serial.println("1-wire device pulse detected");
          #endif
      }
      else {
        #ifdef SERIAL_DEBUG
          Serial.println("NO DS2482 device found");
          #endif
        }
#define  DEBUG_AQUIRE
      #ifdef DEBUG_AQUIRE
        Serial.println("scanning ow...");
        uint8_t rwa;
        rwa = alarm1.wiredeviceAquire();
        Serial.print("wiredeviceAquire return:");
        Serial.println(rwa);
        #endif
    }
    alarm1.setValidNonWire();
#ifdef SERIAL_DEBUGXX
    Serial.println("Final Dump device_list");
#endif
    alarm1.dump_device_list();
    // iterate over device_list and do search rom if type in xx
    if(alarm1.validate_device_list()) {
      notify.sendMessage(FULL_HEADER,1,"Starting");   //send this message now
  #ifdef SERIAL_DEBUG
      Serial.println("Device List Valid");
  #endif
    }
    else {
      //we allow system to run with some missing sensors
      //should halt if count is zero ??
      notify.sendMessage(SHORT_HEADER,1,"DEVFAILVAL");
#ifdef SERIAL_DEBUG
    Serial.println("Device List NOT Valid");
#endif
    }

    if(sys.sysStatus()==0) {
      sys.sysState(sys_running);
      notify.sendMessage(FULL_HEADER,1,"SYSTEM RUNNING");
    }
    else {
      char buf[18];
      sprintf(buf,"SYSTEM FAIL:%x\n",sys.sysStatus());
      notify.queueMessage(SHORT_HEADER,1,buf);
    }

    // set lasthour to  be on 'hours_between_alert' multiple
    notify.setStartTime();
    unsigned long timeSeed = micros();
    randomSeed(timeSeed);
    sensor.readTemperature();
    //sync alarm state with eeprom status  ?? necessary or already done ??
    alarm1.setState(alarm_saved_state.current_state);
#ifdef PHOTON_MASTER
    Serial.println("master running...");
    notify.queueMessage(FULL_HEADER,1,"MASTER RUNNING");
    //alarm1.setLastOil(100.0); //debug test
    alarm1.setLastRemote(SUB_OIL_GAUGE, 101.0);   //should set as inactive
    alarm1.setLastRemote(SUB_CAR_MONITOR,  72.0);
    digitalWrite(MESSAGE_PIN, LOW);
    sys.setSysId(0);
#endif  //photon master
#ifdef PHOTON_REMOTE
    //Particle.variable("remotedata", remoteData);
    alarm1.setState(alarm_armed); //remote is ALWAYS armed
    Serial.println("REMOTE RUNNING..Armed");
    notify.sendMessage(FULL_HEADER,1,"REMOTE RUNNING");
    sys.setSysId(REMOTE_SYSTEM_ID);
#endif
#ifdef HARDWARE_WATCHDOG
    hwd.enable();
#endif
}
#ifdef PHOTON_MASTER
void loop() {
    String event_message;
    char buf[80];
    notify.checkTime();
  #ifdef SERIAL_DEBUG
    Serial.print(".");
  #endif
    //DESIGN: should checkSensors even if disarmed,
    //need to check water, smoke, etc. ALWAYS.
    //third state? (Off, Disarmed, Armed)
    if(alarm1.isArmed()) alarm1.checkSensors(FALSE) ;

    int new_state = alarm1.setState();   //reset state based on current alarm conditions
    if(new_state != -1) {
      // fixed double backslash cr
      String msg="\nalarm state changed:\n";
      msg.concat(alarm1.getStateDescription(new_state));
      notify.queueMessage(SHORT_HEADER,1,msg);
    }

    alarm1.doStatusBlink();
    //blink2Timer.start();   // to clear blink at timeout
    notify.secret_countdown();  //timeout secret number
    if(message_countdown-- <= 0) {
      notify.dequeMessage();
      message_countdown = MESSAGE_LOOP_DELAY;  // manual messages should be immediate
    }
#ifdef SERIAL_DEBUGXX
    else if(!notify.msgqueueEmpty()) {
      Serial.print("dequeMessage delayed:");
      Serial.println(message_countdown);
    }
#endif
    if(alarm1.alarmNotifying()) alarm1.clearing_countup();
    temperatureF = sensor.getLastTemperature();
    delay(LOOP_DELAY_TIME);
    //wd.checkin() done automatically at end of each loop
#ifdef HARDWARE_WATCHDOG
    hwd.checkin();
#endif
}
#endif  //PHOTON_MASTER

#ifdef PHOTON_REMOTE
void loop() {
  char msgType; //in {Tripped,Clear,Notify,Reset,Info}
  bool do_master_send = notify.checkTime();
  if(do_master_send) msgType = 'I';
  //Serial.println(do_master_send);
  //DESIGN??
  //We need to FORCE ALL SENSORS inot ChangedSensor if checkTime ??
  bool t = alarm1.checkSensors(do_master_send);
  String changed = sensor.fmtChangedSensors();  //<<<=== report all if 'I' ??
  #ifdef SERIAL_DEBUG_SEN
  Serial.print("checkSensors returned:");
  Serial.println(t);
  Serial.print("changedSensors:");
  Serial.println(changed);
  #endif
  #ifdef SERIAL_DEBUGXX
    Serial.print("remoteState=");
    Serial.print(alarm_state_name_def[remoteState]);
  #endif
  if(sensor.hasChangedSensor()) do_master_send = TRUE; //OR from checkTime
  if(do_master_send) {
    //data format:
    //  messageid@system_id@A@hh:mm:ss@sensor-name:masterIdx:reading:?^...second sensor...$
    char sensordata[80]; //SIZE??
    //if(sensor.hasTrippedSensor())  //why doesn't work?
    if(alarm1.isTripped()) msgType = 'T';
    //else
    //  msgType = 'C';  //DESIGN ?? need to use State??
    String time = Time.format("%H:%M:%S%S");
    sprintf(data,"%d@%d@%c@%s%s$",msgSeq++,sys.getSysId(),msgType,time.c_str(),changed.c_str());
    String rData = String(data);
//#define REMOTE_DEBUG_NO_MASTER
#ifdef REMOTE_DEBUG_NO_MASTER
    Particle.publish("rtestdata", rData);
#else
    Particle.publish("remotedata", rData);
#endif
#ifdef SERIAL_DEBUG_CLOUD
    Serial.print("remotedata=");
    Serial.print(rData);
#endif
    alarm1.clearSensorReported();
    //digitalWrite(MESSAGE_PIN, HIGH);       //blink red led on publish
  }
  delay(REMOTE_LOOP_DELAY_TIME); //use two delays to show led
  alarm1.doStatusBlink();
  #ifdef SERIAL_LOOP_WAIT
    while(!Serial.available())  // Wait here until the user presses ENTER
      Spark.process();          // in the Serial Terminal. Call the BG Tasks
    Serial.read();
//  #else
//    delay(REMOTE_LOOP_DELAY_TIME);
  #endif
  //digitalWrite(MESSAGE_PIN, LOW);       //blink red led on publish
  nextState = remoteState;
}
#endif  //PHOTON_REMOTE
