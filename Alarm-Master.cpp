/*
  Master-Remote alarm system for Particle Photon
  @file     Alarm-Master.cpp
  @author   D. Thompson
  @license  GNU General Public License (see license.txt)
  @version  3.1.0

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

/* TO DO:
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
  * Added in  v3.1.0
  * HOME an AWAY modes and commands to set
  * Redesigned alarm event handling (in class Alarm)
  */

/*
 * Added in  v1.9
 * add last updated timestamp on device
 * remoteHandler to receiver car temperature alerts from sub-sensor
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

#define SYSTEM_VERSION 3.1.2
#define SYSTEM_VERSION_MAJOR 3
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
//#define SERIAL_DEBUG_SEN
//#define SERIAL_DEBUG_EVENT
 //#define SERIAL_DEBUGXX
//#define SERIAL_WAIT
//#define SERIAL_LOOP_WAIT
//#define HARDWARE_WATCHDOG
#endif
#ifdef PHOTON_REMOTE
#define SERIAL_DEBUG
#define SERIAL_DEBUGXX
#define SERIAL_WAIT
//#define SERIAL_LOOP_WAIT
//#define HARDWARE_WATCHDOG
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
    uint8_t sensor_priority = 0;  //reporting priority
    double tempF;
    double oilL;
    bool senret;
    bool rok;
    bool toSetSensor;
    if(d.status != DEV_PRESENT) {
  #ifdef SERIAL_DEBUG_SEN
      Serial.print("Device Missing - ROM:");
      Serial.println(d.dev_rom[7],HEX);
  #endif
      return FALSE;
    }
    if(d.state == SENSOR_DISABLED) {
  #ifdef SERIAL_DEBUG_SEN
      Serial.print("Device deactivated - ROM:");
      Serial.println(d.dev_rom[7],HEX);
  #endif
      return FALSE;
    }
    switch(d.dev_use) {
      case SWITCH:    //simple switch on PHOTON pin
  #ifdef SERIAL_DEBUG_SEN
          Serial.print("readSensor name=");
          Serial.print(d.name);
          Serial.print(" rom=");
          for(int k=0;k<8;k++) {
            Serial.print(d.dev_rom[k],HEX);
            Serial.print(":");
          }
          Serial.print(" use=");
          Serial.print(d.dev_use);
  #endif
         senval = digitalRead(d.dev_rom[0]);
         d.dev_reading = (float)senval;
         d.dev_last_read = Time.now();
  #ifdef SERIAL_DEBUG_SEN
          Serial.print(" readvalue=");
          Serial.println(senval);
  #endif
          return senval==d.sense;
          break;
      case OW_SENSOR:
      case OW_INDICATOR:  //one-wire device on DS2482-100
          senval = p_ow->readPIOX(d.dev_rom, d.port);    //??just readPIO and use d.port
          toSetSensor = (uint8_t)d.dev_reading != senval;
          d.dev_reading = (float)senval;
          d.dev_last_read = Time.now();
          senret = senval==d.sense;
          #ifdef SERIAL_DEBUG_SEN
                  Serial.print("readSensor-OW_SENSOR: name=");
                  Serial.print(d.name);
                  Serial.print(" dev_use=");
                  Serial.print(d.dev_use);
                  Serial.print(" dev_reading=");
                  Serial.print(d.dev_reading);
                  Serial.print(" tripped=");
                  Serial.print(d.tripped);
                  Serial.print(" reported=");
                  Serial.print(d.reported);
                  Serial.print(" ow value=");
                  Serial.print(senval);
                  Serial.print(" return=");
                  Serial.println(senret);
          #endif
          if(d.dev_use==OW_INDICATOR)
            //if(toSetSensor) setSensorIndicator(d, senret);
            setSensorIndicator(d, senret);
          return senret;
          break;
      // TODO: Implement??
      case SUB_REMOTE_THERMOMETER:
      case SUB_REMOTE_SENSOR:
          //TODO: NEED TO VALIDATE dev_last_read ??
          return d.dev_reading > d.alert_max;
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
              d.dev_reading = last_temp;
              d.dev_last_read = Time.now();
              // return value should  be tripped if outside temp range ??
              // report using priority of sensor ??
              if((int)last_temp <= d.alert_min) return TRUE;
              // readTemp should return an error condition ??
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
Notifier notify("shed_notice","shed_alarm", &sensor);

#include "alarm.h"

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
  if(curState != alarm_disarmed)
    stateDescr.concat(alarm_state_location_def[curLocation]);
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
  if(curState == alarm_disarmed) return -1; //no change
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
      new_state = alarm_armed;
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
  curState = new_state; //aaaaaa
  setStatusLED();
  stateA = getStateDescription(curState);
  Particle.publish("alarmState",stateA,60);
  return curState;
}

// rewrite of setState(x)
bool Alarm::setState(uint8_t new_state) {
  if(curState == new_state) //NO State Change
    return FALSE;
#ifdef SERIAL_DEBUG_EVENT
  Serial.print("setState(x) to ");
  Serial.print(alarm_state_name_def[new_state]);
  Serial.print(" :curState =");
  Serial.println(alarm_state_name_def[curState]);
#endif
  if(new_state == alarm_disarmed) {
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
      curState = alarm_disarmed;
      //then clear event_list
      event_list.clear();
      return TRUE;  //???
  }
  else if(new_state == alarm_armed) {
      curState = alarm_armed;
      //write to eeprom ??
      return TRUE;
  }
  else {
    return FALSE;
  }
  //setState();
  return TRUE;
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
            debug("clearing_cnt=%d\\n",clearing_cnt);
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

uint8_t Alarm::buildDeviceList() {
  uint8_t dev_cnt = 0;
  char nbuf[30];  //at least SENSOR_NAME_SIZE   and SIZE for ROMHEX ??
  if(configuration.magic != EE_MAGIC_CONFIG) return 0;
#ifdef SERIAL_DEBUGXX
  Serial.println("buildDeviceList:");
#endif
  for(int j = 0; j< MAXDEVICE; j++) {
    if(!configuration.use[j]) continue;
    #ifdef SERIAL_DEBUGXX
      sensor.romFormat(&nbuf[0], configuration.dev_addr[j]);
      Serial.print(" config rom=");
      Serial.print(nbuf);
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
#ifdef SERIAL_DEBUGXX
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
#ifdef SERIAL_DEBUGXX
  Serial.print("added to device_list:");
  Serial.println(dev_cnt);
#endif
  return dev_cnt;
}

char* Alarm::deviceListing(char *buf) {
  std::list<device_t>::iterator k;
  char temp[10];
  buf[0]=0;
  strcat(buf,"\\n"); //json escape line feed
  for(k=device_list.begin(); k !=device_list.end(); ++k) {
     device_t sp = *k;
     sprintf(temp,"%02d ", sp.idx);
     strcat(buf,temp);
     strncat(buf,sp.name,SENSOR_NAME_SIZE);
     strcat(buf," state:");
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
     strcat(buf,"\\n");
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
  strcat(buf,"\\n"); //json escape line feed
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
     sprintf(temp," %s  %5.1fF\\n", lastUpd.c_str(), sp.dev_reading);
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
  strcat(buf,"\\nevent list");
  if(event_list.empty()) strcat(buf,"no events");
  else
    for(e=event_list.begin(); e !=event_list.end(); ++e) {
      device_t* dp=getDevice(e->second);
      if(dp!=0) {
        strncpy(name,dp->name,SENSOR_NAME_SIZE);
        name[SENSOR_NAME_SIZE]=0;
      }
      sprintf(temp, "\\n%02d %s %d", e->second, name, e->first);
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
  strcpy(trippedbuf,"TRIPPED:\\n");
  for(k=trippedList.begin(); k != trippedList.end(); ++k) {
     device_t sp = *k;
     //??TODO: do something with reported ??
     strncat(trippedbuf,sp.name,SENSOR_NAME_SIZE);
     strcat(trippedbuf,"\\n");
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
  for(k=device_list.begin(); k !=device_list.end(); ++k) {
    device_t sp = *k;
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
    sys.sysState(sys_fail); //this will do HANG
  }
  if(all_present)
    Serial.println("all dev present");
  return all_present;
}

void Alarm::setDeviceActive(device_t *d, bool toActive) {
    if(toActive)
      d->state = SENSOR_ACTIVE;
    else {
      d->state = SENSOR_DISABLED;
      if(d->dev_use >= SUB_REMOTE_THERMOMETER) { //a REMOTE device
        d->dev_reading = 0;
        d->dev_last_read = 0;
        d->tripped = FALSE;
        d->reported = FALSE;
      }
    }
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
  curLocation = loc;
}
uint8_t Alarm::getCurLocation() {
  if(curLocation < 0 || curLocation > 1) return 0;
  return curLocation;
}

//This is set master device from remote data
bool Alarm::setAlarmRemote(uint8_t idx, float senValue) {
  std::list<device_t>::iterator k;
  bool fountIt = FALSE;
  for(k=device_list.begin(); k != device_list.end(); ++k) {
    if(k->idx != idx) continue;
    if(k->dev_use == SUB_REMOTE_SENSOR ||
       k->dev_use == SUB_REMOTE_THERMOMETER) {
         k->dev_reading = senValue;
         k->dev_last_read = Time.now();
         fountIt = TRUE;
    }
  }
  return fountIt;
}
//ONLY sensor with 'SUB..' use type
bool Alarm::setLastRemote(uint8_t use, float val) {
  std::list<device_t>::iterator k;
  if(use != SUB_OIL_GAUGE && use!=SUB_CAR_MONITOR) return FALSE;
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
  //xxxxzz
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
  return curState != alarm_disarmed;
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
      Serial.print("ROM CD:");
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
  //test config for neshed
  /*
  #define TEST_SENSOR_1
  #define TEST_SENSOR_2
  //#define TEST_SENSOR_3
  //#define TEST_SENSOR_4
  //#define TEST_SENSOR_5
  //#define TEST_SENSOR_6
  //#define TEST_SENSOR_7
  //#define TEST_SENSOR_8
  //#define TEST_SENSOR_9
  //#define TEST_SENSOR_10
  #define TEST_SENSOR_11
  #define TEST_SENSOR_12
  #define TEST_SENSOR_13
  #define TEST_SENSOR_14
  #define TEST_SENSOR_15
*/
  //Sensor 1: mcp_9808
#ifdef TEST_SENSOR_1
configuration.dev_addr[i][0] = MCP9808_I2CADDR;
  configuration.master_idx[i] = 0;
  for(int k=1;k<8;k++)
    configuration.dev_addr[i][k] = 0xff;
  configuration.dev_flags[i] = 0;
  configuration.port[i] = 0;
  configuration.use[i] = MCP9808_THERMOMETER;
  configuration.sense[i] = 0;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 0;
  configuration.alert_max[i] = 0;
  strncpy(configuration.name[i], "INSIDE TEMP", SENSOR_NAME_SIZE);
  i++;
#endif
  //Sensor 2: NE Shed Door
#ifdef TEST_SENSOR_2
const uint8_t testrom1[8] = { 0x12,0x3a,0x84,0x72,0x00,0x00,0x00,0xd8 };
  configuration.master_idx[i] = 0;
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom1[k];
  configuration.dev_flags[i] = 1 << DEVICE_PRIORITY;
  configuration.port[i] = PIO_B;
  configuration.use[i] = OW_SENSOR;
  configuration.sense[i] = SENSE_NORMAL_CLOSED;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 0;
  configuration.alert_max[i] = 0;
  strncpy(configuration.name[i], "NE SHED DOOR", SENSOR_NAME_SIZE);
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
  configuration.use[i] = OW_INDICATOR;
  configuration.sense[i] = SENSE_NORMAL_CLOSED;
  configuration.alert_level[i] = HOME_DEVICE;
  configuration.alert_min[i] = 0;
  configuration.alert_max[i] = 0.90;
  strncpy(configuration.name[i], "TEST DOOR", SENSOR_NAME_SIZE);
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
  configuration.master_idx[i] = 1;
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom12[k];
  configuration.dev_flags[i] = 0;
  configuration.port[i] = 0;
  configuration.use[i] = SUB_REMOTE_SENSOR;
  configuration.sense[i] = SENSE_NORMAL_OPEN;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 0.0;
  configuration.alert_max[i] = 0.90;
  strncpy(configuration.name[i], "REMOTE-DOOR", SENSOR_NAME_SIZE);
  i++;
#endif
#ifdef TEST_SENSOR_14
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
    strncpy(configuration.name[i], "OUTSIDE TEMP", SENSOR_NAME_SIZE);
    i++;
#endif
#ifdef TEST_SENSOR_15
    //Temperature MCP9808 -- REMOTE Actual Device
    const uint8_t testrom14[8] = { MCP9808_I2CADDR,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
      configuration.master_idx[i] = 4;
      for(int k=0;k<8;k++)
        configuration.dev_addr[i][k] = testrom14[k];
      configuration.dev_flags[i] = 0;
      configuration.port[i] = 0;
      configuration.use[i] = MCP9808_THERMOMETER;
      configuration.sense[i] = 0;
      configuration.alert_level[i] = AWAY_DEVICE;
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
configuration.alert_level[i] = AWAY_DEVICE;
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
  configuration.port[i] = PIO_A;
  configuration.use[i] = OW_SENSOR;
  configuration.sense[i] = SENSE_NORMAL_OPEN;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 0;
  configuration.alert_max[i] = 0;
  strncpy(configuration.name[i], "KITCHEN SMOKE", SENSOR_NAME_SIZE);
  i++;
#endif
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
#endif
    curState = alarm_saved_state.current_state;
    curLocation = alarm_saved_state.current_location;
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
bool Alarm::writeSavedState() {
  alarm_saved_state.magic = EE_MAGIC_STATE;
  alarm_saved_state.current_state = curState;
  alarm_saved_state.current_location = curLocation;
  alarm_saved_state.alert_hours = notify.getAlertHours();
  alarm_saved_state.oil_gallons = gallonsO;
  alarm_saved_state.event_sequence = event_sequence;
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
bool Alarm::checkSensors(void) {
  //bool temptrip = FALSE;
  char buf[30];
  uint8_t alert_priority = 0;  //readSensor needs to update this
  bool sensorIsTripped = FALSE;
  bool sensorIsReporting = FALSE;
  bool reporting = FALSE;      //reporting a new tripped sensor
  firstTrippedSensor = 0;
  trippedList.clear();
  std::list<device_t>::iterator k;
  for(k=device_list.begin(); k !=device_list.end(); ++k) {
    device_t sp = *k;
  #ifdef SERIAL_DEBUG_SEN
    Serial.print("checkSensor:device_list name=");
    Serial.print(k->name);
    Serial.print(" rom=");
    for(int n=0;n<8;n++) {
      Serial.print(k->dev_rom[n],HEX);
      Serial.print(":");
    }
    Serial.print(" use=");
    Serial.print(k->dev_use);
    Serial.print(" tripped=");
    Serial.print(k->tripped);
    Serial.print(" reported=");
    Serial.println(k->reported);
  #endif
    //if(sp.status == DEV_PRESENT) {  <=== add this ???
    if(sensor.readSensor(*k)) {
      if(curLocation==AT_HOME && k->alert_level==AT_HOME) return FALSE;
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
        #ifdef SERIAL_DEBUGXX
        Serial.print("tripped dev idx=");
        Serial.println(k->idx);
        #endif
        if(addEvent(k->idx)) {
          //added new event, so notify
          String event_message = "sensor tripped\\n";
          event_message.concat(k->name.c_str());
          sprintf(buf,"\\nevent seq:%d",event_sequence);
          event_message.concat(buf);
          notify.queueMessage(FULL_HEADER,1,event_message);
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
      #ifdef PHOTON_REMOTE
      reporting = TRUE;
      #endif
    }
    else {
      k->tripped = FALSE;
    }
    #ifdef SERIAL_DEBUG_SEN
      if(k->dev_use==5) {
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
  }
  return reporting;
}

void Alarm::clearing_countup() {
  if(allClear()) {
    if(clearing_cnt++ > CLEARING_COUNT_MAX) { //?? count
      //debug("clearing_countup:cnt=%d\\n",clearing_cnt);
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
Alarm alarm1;

void remoteAlarm(const char *eventName, const char *data) {
  device_t *d;
  String alarmData = String(data);
  notify.sendMessage(SHORT_HEADER,1,alarmData);
  //parse this data
  String msgText;
  char temp[15];
  Parse p(alarmData);
  int parse_cnt;
  #ifdef DEBUGZZ
  Serial.println("Parse alarmData:");
  Serial.print("rData: ");
  Serial.println(p.getData());
  #endif

  parse_cnt = p.doParse();
  #ifdef DEBUGZZ
  Serial.print("parsed ");
  Serial.print(parse_cnt);
  Serial.println(" items");
  for(int i=0; i< parse_cnt; i++) {
    Serial.println(p.getElement(i));
  }
  Serial.println("end of items");
  //END REMOVE--------------------
  #endif
  //int i = alarmData.indexOf("@");
  //char msgType = alarmData.charAt(i+1);
  uint8_t system_identification = p.getElement(1).toInt();
  char msgType = p.getElement(2).charAt(0);
  uint8_t setValue = 0;
  //DEBUG shows message received from REMOTE
  if(msgType=='I') msgText = "Info";
  else if(msgType == 'T') {
    msgText = "Tripped";
    setValue = 1;
  }
  else if(msgType == 'N') msgText = "Notify";
  else if(msgType == 'R') msgText = "Reset";
  else if(msgType == 'C') msgText = "Clear";
  else if(msgType == '?') msgText = "?Unk";
  else msgText = "Unknown";
  // specify which remote here (for multiple remotes), sys.sysId
  sprintf(temp,"remote %d:",system_identification);
  String tmsg = String(temp);
  tmsg.concat(temp);
  tmsg.concat(msgText);

  #ifdef PUSHOVER_SEND_MESSAGES
  notify.sendMessage(SHORT_HEADER,1,tmsg);
  String smsg = "sensors:\\n";
  if(parse_cnt > 3) {
    for(int i=4; i< parse_cnt; i++) {
      Parse ps(FIELD_DELIMITER, END_DELIMITER, p.getElement(i));
      //Serial.println(p.getElement(i));
      smsg.concat("\\nel:");
      smsg.concat(p.getElement(i));
      //notify.sendMessage(SHORT_HEADER,1,smsg);
      ps.doParse();
      //String sdetail = "detail: ";
      uint8_t masterIdx = ps.getElement(1).toInt();
      float setValue = ps.getElement(2).toFloat();
      smsg.concat("\\ndt:");
      smsg.concat (ps.getElement(0));
      smsg.concat(":midx=");
      smsg.concat(ps.getElement(1));
      smsg.concat(":val=");
      smsg.concat(ps.getElement(2));
      alarm1.setAlarmRemote(masterIdx, setValue);  //??xxxxyy
    }
  }
  notify.sendMessage(SHORT_HEADER,1,smsg);
  #endif
}
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
  alarm1.writeSavedState();
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
    //check = TRUE;
  }
  min_hours_between = max(1,hours_between_alert);
  tempHour = hour;
  if(tempHour < lasthour) tempHour += 24;
  //if hours_between_alert = 4  {4,8,12}
  //FIRST TIME EARLY: LAST_HOUR 3
  //if hours_between_alert is zero, don't alert but do log temp hourly
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
  String msg = String("\\nsys up time: ");
  msg.concat(uptime);
  //remote data
  if(remote_temp > -1000.0) {
    sprintf(buf,"\\ncar temp: %5.1fF",remote_temp);
    msg.concat(buf);
  }
  sprintf(buf,"\\nworry %d hours",hours_between_alert);
  msg.concat(buf);
  ts = alarm1.firstTripped();
  if(ts != NULL) {
    //SHOULD do all tripped, not just first
    sprintf(buf,"\\ntripped sensors:\\n");
    strcat(buf,ts->name);
  } else
    sprintf(buf,"\\nall sensor clear");
  msg.concat(buf);
  //sprintf(buf,"\\nclearing: %d",alarm1.clearingCount());
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
const String Notifier::commands = String("HLP.ABT.TMP.SET.DIS.ACK.HOM.AWA.HOU.LIS.NAM.SEN.ACT.DEA.MIN.MAX.EVT.CFG.");
//                                        0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17

int Notifier::request(String cmd_line) {
  char msg[20];
  int cidx;
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
  if(cidx <= 2) {
    return do_command(cmd_line);
  }
// if ok, generate new secrets
  int new_secret = random(RANDOM_MIN,RANDOM_MAX);
// enter into command_list
  command_list[new_secret] = cmd_line;
// message secret to confirm
  sprintf(msg, " Confirmation: %d", new_secret);
  queueMessage(NO_HEADER,1,msg);
  return 1;
}

#ifdef DEBUGXX
//debug dump of command list
int Notifier::dump(String t) {
  char buf[40];
  char msg[400];
  std::map <int, String>::iterator mi;
  if(command_list.empty())
      queueMessage("\\ncommand_list is empty");
  else {
      strcpy(msg,"\\ncommand_list:");
      for(mi=command_list.begin();mi != command_list.end(); ++mi) {
        sprintf(buf,"\\ncommand: %s secret: %d", mi->second.c_str(), mi->first );
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
    queueMessage(NO_HEADER,1,"TOO MANY BAD SECRETS");  //debug message
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
  sprintf(buf, "\\nInvalid Confirmation #: %d\\n", secret_index );
  bad_secret_cnt++;
  queueMessage(NO_HEADER,1,buf);
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
    case 0 : // Command HELP
    //("HLP.ABT.TMP.SET.DIS.ACK.HOM.AWA.HOU.LIS.NAM.SEN.ACT.DEA.MIN.MAX.EVT.CFG.");

      strcpy(listing_buffer,"\\nCommands:");
      strcat(listing_buffer,"\\nABT - About");
      strcat(listing_buffer,"\\nHLP - This Help");
      strcat(listing_buffer,"\\nSET On");
      strcat(listing_buffer,"\\nDISable");
      strcat(listing_buffer,"\\nHOMe");
      strcat(listing_buffer,"\\nAWAy");
      strcat(listing_buffer,"\\nACKnowledge n");
      strcat(listing_buffer,"\\nTMPerature");
      strcat(listing_buffer,"\\nHOU btw Worry to n");
      queueMessage(NO_HEADER,1,listing_buffer);

      strcpy(listing_buffer,"\\nLISt Sensors");
      strcat(listing_buffer,"\\nNAMe Sensor n");
      strcat(listing_buffer,"\\nSENsor Detail");
      strcat(listing_buffer,"\\nACTivate sensor n");
      strcat(listing_buffer,"\\nDEActivate sensor n");
      strcat(listing_buffer,"\\nEVT - event list");
      strcat(listing_buffer,"\\nMAXimum Alert Temp to x");
      strcat(listing_buffer,"\\nMAXimum Alert Temp to x");
      strcat(listing_buffer,"\\nCFG - gen device config");
      queueMessage(NO_HEADER,1,listing_buffer);
      break;
    case 1 : // ABT
      strcpy(listing_buffer,"\\nphotonAlarm\\n");
      strcat(listing_buffer,"\\ncopyright (c) re:Engineering 2016");
      strcat(listing_buffer,"\\nDonald Thompson");
      sprintf(msg,"\\nVersion %d.%d.%d", SYSTEM_VERSION_MAJOR, SYSTEM_VERSION_MINOR, SYSTEM_VERSION_SUB);
      strcat(listing_buffer, msg);
      queueMessage(NO_HEADER,1,listing_buffer);
      break;
    case 2 : //TMP Display All Thermometers
        alarm1.thermometerListing(listing_buffer);
        queueMessage(NO_HEADER, 1, listing_buffer);
        return 1;
        break;
    case 3 : // SET Alarm if Secret matches
        //command = 'SET <secret>'
          // set unless tripped
        if(alarm1.setState(alarm_armed)) {
            alarm_saved_state.current_state = alarm1.getState();
            alarm1.writeSavedState();
            #ifdef SERIAL_DEBUG
              Serial.print('set armed');
            #endif
        }
        queueMessage(SHORT_HEADER, alarm1.getPriority(),alarm1.getPendingMessage());
        return 1;
        break;
    case 4 : // Disarm Alarm if Secret matches
        //command = 'DISARM <secret>'
        // DISABLE alarm here!!
        alarm1.setState(alarm_disarmed);
        alarm_saved_state.current_state = alarm_disarmed;
        alarm1.writeSavedState();
        queueMessage(SHORT_HEADER,alarm1.getPriority(),alarm1.getPendingMessage());
        return 1;
        break;
    case 5 : // Acknowledge tripped Alarm
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
    case 6 : // SET alarm mode to at HOME
        alarm1.setCurLocation(AT_HOME);
        queueMessage(FULL_HEADER,1,"Location set to AT HOME");
        break;
    case 7 : // SET alarm mode to AWAY
        alarm1.setCurLocation(AWAY);
        queueMessage(FULL_HEADER,1,"Location set to AWAY");
        break;
    case 8 : // SET alert Hours if Secret matches
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
    case 9 : // List all Devices
          //command = 'LIST <secret>'
          // LIST ALL Sensor Devices
          alarm1.deviceListing(listing_buffer);
          queueMessage(NO_HEADER, 1, listing_buffer);
          return 1;
          break;

    case 10 : // Name a Sensor
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
    case 11 : // List a Devices in Full
          //command = 'SEN <sen#>'
          listing_buffer[0]=0;
          d = alarm1.getDevice(p1);
          if(d) {
              sprintf(listing_buffer,"\\nIndex: %d",d->idx);
              strcat(listing_buffer,"\\nrom: ");
              char buf[26];
              p_sensor->romFormat(&buf[0], d->dev_rom);
              strcat(listing_buffer,buf);
              strcat(listing_buffer,"\\nStatus: ");
              strcat(listing_buffer,sensor_status_def[d->status]);
              strcat(listing_buffer,"\\nState: ");
              strcat(listing_buffer,sensor_state_def[d->state]);
              strcat(listing_buffer,"\\nUse: ");
              strcat(listing_buffer,sensor_use_def[d->dev_use]);
              strcat(listing_buffer,"\\nSense: ");
              strcat(listing_buffer,sensor_sense_def[d->sense]);
              if(d->alert_min) {
                sprintf(buf,"\\nAlert Min: %d", d->alert_min);
                strcat(listing_buffer, buf);
              }
              if(d->alert_max) {
                sprintf(buf,"\\nAlert Max: %d", d->alert_max);
                strcat(listing_buffer, buf);
              }
              strcat(listing_buffer,"\\nName: ");
              strncat(listing_buffer,d->name,SENSOR_NAME_SIZE);
              strcat(listing_buffer,"\\nTripped: ");
              if(d->tripped)
                  strcat(listing_buffer,"Y");
              else
                  strcat(listing_buffer,"N");
              strcat(listing_buffer,"\\nReported: ");
              if(d->reported)
                  strcat(listing_buffer,"Y");
              else
                  strcat(listing_buffer,"N");
              sprintf(buf,"\\nReading: %5.2f",d->dev_reading);
              strcat(listing_buffer,buf);
              if(d->dev_last_read > Time.now() - 3600 * 24) {
                strcat(listing_buffer,"\\nLast Read: ");
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
      case 12 : // Activate a sensor
          //command = 'ACT <sen#>'
          //TO DO: only set if present ??
          d = alarm1.getDevice(p1);
          if(d) {
              alarm1.setDeviceActive(d, TRUE);
              sprintf(msg, "SENSOR ACTIVATED: %d", p1);
          }
          else
              sprintf(msg, "NO SENSOR# %d", p1);
          queueMessage(SHORT_HEADER,1,msg);
          return 1;
          break;
      case 13 : // Deactivate a sensor
          //command = 'DEA <secret> <sen#>'
          d = alarm1.getDevice(p1);
          if(d) {
              alarm1.setDeviceActive(d, FALSE);
              sprintf(msg, "SENSOR DEACTIVATEDxx: %d", p1);
          }
          else
              sprintf(msg, "NO SENSOR# %d", p1);
          queueMessage(SHORT_HEADER,1,msg);
          return 1;
          break;
      case 14 : // Set Sensor Minimum Alarm Temperature
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

      case 15 : // Set Sensor MAXimum Alarm Temperature
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

          case 16 : // Display Event List
          //command = 'EVT'
          alarm1.eventListing(listing_buffer);
          queueMessage(NO_HEADER, 1, listing_buffer);
          return 1;

          break;
          case 17 : // Auto-Configure Sensors
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
      default:   // Error
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
  //xxxxzz
  std::map <int, uint8_t>::iterator ei;
  if(!event_list.empty()) {  //??WHY THIS line ?? empty ??
    //first make sure this device is not already in event list (by idx)
    for(ei=event_list.begin();ei != event_list.end(); ++ei) {
      if(ei->second == idx) {
        #ifdef SERIAL_DEBUGXX
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
  alarm1.writeSavedState();
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
  //xxxxzz
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
  //write ??
  alarm1.writeSavedState();
}
uint8_t Notifier::getAlertHours() {
  return hours_between_alert;
}

bool Notifier::msgqueueEmpty() {
    return message_queue.empty();
}
void Notifier::queueMessage(uint8_t header, uint8_t pri, String msg) {
  Message message;
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
  Message next_message;
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

void Notifier::sendMessage(uint8_t hdr, uint8_t pri, String msg) {
  //TO DO: shorten tmessage to 200 ?? messages can not be this long
  char tmessage[620];  //debug testing size limit / was 600
  strncpy(tmessage, msg.c_str(), 618);
  tmessage[619]=0;
  sendMessage(hdr, pri, tmessage);
}

void Notifier::sendMessage(uint8_t hdr, uint8_t pri, char* msg) {
#ifdef SERIAL_DEBUG
  Serial.print("sendMessage: pri=");
  Serial.print(pri);
#endif
  event_message[0] = 0;
  if(hdr != NO_HEADER) {
    String alarmStateName = alarm1.getStateDescription();
    String time = Time.format("%m/%d/%y %H:%M:%S");
    sprintf(event_message,"%s %4.1fF\\n%s",time.c_str(),
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
        strcat(event_message," sensors tripped:\\n");
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
#ifdef PUSHOVER_SEND_MESSAGES
  //TO DO:  SELECT webhook_id based upon priority of message
  if(pri) {
    Serial.print(" publish w/ priority ");
    Spark.publish(priority_webhook_id, event_message, 60, PRIVATE);
//  Spark.publish(webhook_id, event_message, 60, PUBLIC);
  }
  else {
    Serial.print(" publish ");
    Spark.publish(webhook_id, event_message, 60, PRIVATE);
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
    Particle.variable("remotedata", remoteData);
#endif
  int newseed = 0;
  remote_temp = -1002.00;
  random_seed_from_cloud(newseed); //?? usage ??
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
    Serial.printf("\\nPhotoAlarm initializing...Version %d.%d.%d",
          SYSTEM_VERSION_MAJOR, SYSTEM_VERSION_MINOR, SYSTEM_VERSION_SUB);
    sys.sysState(sys_starting);
//    digitalWrite(trigger3, HIGH); //DEBUG, trigger pin3 for logic 8 analyzer
    Time.zone(EDT_OFFSET);
    uint8_t stat = mcp9808.begin(MCP9808_I2CADDR);  //bus master
    //non-zero = error, not mcp9808
    if(stat) {
      sys.addStatus(fail_hardware);
      notify.sendMessage(NO_HEADER,1,"MCP9808 FAIL");
      sys.sysState(sys_fail); //this will do HANG
    }
    ow.reset();
    if(!alarm1.readSavedState()) {
      alarm_saved_state.current_state = alarm_disarmed;
      alarm_saved_state.event_sequence = 1000;
      alarm_saved_state.alert_hours = ALERT_HOURS;
      alarm1.writeSavedState();
    }
    else {
      #ifdef SERIAL_DEBUG_EVENT
      Serial.print("readSavedState: event_sequence=");
      Serial.println(alarm_saved_state.event_sequence);
      #endif
    }
    notify.setAlertHours(alarm_saved_state.alert_hours);
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
      sys.sysState(sys_fail); //this will do HANG
    }
    //build device list from read configuration
    if(alarm1.buildDeviceList()) {
      alarm1.dump_device_list();
    }
    else notify.queueMessage(SHORT_HEADER,1,"NODEVICE");

    // check DS2482 status and config to confirm presence
    ow.reset();
    stat = ow.wireReadStatus(FALSE);
    #ifdef SERIAL_DEBUG
    Serial.print("DS2482 Reset - Status Reg:");
    Serial.println(stat, HEX);  //sb 0x18 (RST=1 AND 1-WIRE LL HIGH?)
    #endif
    if(stat != 0x18) {
      notify.sendMessage(NO_HEADER,1,"HDW FAIL");   //send this message now
      sys.addStatus(fail_ds2482);
      sys.sysState(sys_fail);           //and halt
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
    if(sys.sysStatus()==0) {
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
    bool nowTripped = FALSE;
    notify.checkTime();
  #ifdef SERIAL_DEBUG
    Serial.print(".");
  #endif
    if(alarm1.isArmed()) alarm1.checkSensors() ;

    int new_state = alarm1.setState();   //reset state based on current alarm conditions
    if(new_state != -1) {
      String msg="\\nalarm state changed:\\n";
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
  //TODO: fix message dequeue so that queueMessage works, don't use sendMessage
  #ifdef SERIAL_DEBUG
    Serial.println("loop.");
  #endif
  bool do_master_send = notify.checkTime();
  //DEBUG: REMOVE
  do_master_send = FALSE;  //shut off worry sends
  //ALL THIS !!! REWRITE TODO!!!
  bool t = alarm1.checkSensors();
  #ifdef SERIAL_DEBUGXX
  Serial.print("checkSensors returned:");
  Serial.println(t);
  #endif
  uint8_t currentState = alarm1.getState();
  uint8_t nextState = currentState;
  #ifdef SERIAL_DEBUGXX
    Serial.print("currentState=");
    Serial.print(alarm_state_name_def[currentState]);
  #endif
  //change this to use setState() like MASTER loop ????
  if(t){
    switch(alarm1.getState()) {
      case alarm_armed:     nextState = alarm_tripped;
      break;
      //DOES NOT go to notifyint until sensor reset
      //case alarm_tripped:   nextState = alarm_notifying;
      //break;
      //case alarm_notifying: nextState = alarm_notifying;
      //break;
    }
  }
  else {
    switch(alarm1.getState()) {
      //case alarm_armed:
      //break;
      case alarm_tripped:   nextState = alarm_notifying;
      break;
      case alarm_notifying: nextState = alarm_clearing;
      break;
      case alarm_clearing:  nextState = alarm_armed;
      break;
    }

  }
  #ifdef SERIAL_DEBUGXX
    Serial.print("  nextState=");
    Serial.println(alarm_state_name_def[nextState]);
  #endif
  alarm1.setState(nextState);
  //do_master_send = currentState != nextState; //OR checkTime ??
  if(currentState != nextState) do_master_send = TRUE; //OR from checkTime
  if(do_master_send) {
    //data format:
    //messageid@A@hh:mm:ss@sensor-name@reading$
    char sensordata[80]; //SIZE??
    //bool remoteIsTripped = alarm1.formatRemoteTrip(sensordata);
    //char remoteCondition = remoteIsTripped ? 'A' : 'I';
    //Clear, Tripped, Reset
    uint8_t remoteIsTripped = alarm1.formatRemoteTrip(sensordata);
    #ifdef SERIAL_DEBUGXX
    Serial.print("formatted sensorData=");
    Serial.print(sensordata);
      Serial.print("  remoteIsTripped=");
      Serial.println(remoteIsTripped);
    #endif
    //0=C 1=T 2=S 3=R
    char remoteCondition;
    //switch(remoteIsTripped){
    switch(nextState) {
      case alarm_armed: remoteCondition = 'A';
      break;
      case alarm_tripped: remoteCondition = 'T';
      break;
      case alarm_notifying: remoteCondition = 'N';
      break;
      case alarm_clearing: remoteCondition = 'R';
      break;
      default: remoteCondition = 'I';
    }
    #ifdef SERIAL_DEBUGXX
      Serial.print("remoteCondition=");
      Serial.println(remoteCondition);
    #endif

    String time = Time.format("%H:%M:%S");
    sprintf(data,"%d@%d@%c@%s",msgSeq++,sys.getSysId(),remoteCondition,time.c_str());
//---xxxxyy
    strcat(data,sensordata);
    strcat(data,"$");
    String rData = String(data);
    Particle.publish("remotedata", rData);
    /* REMOVE--------------------
    //debug / test Parse class
    Parse p(rData);
    int parse_cnt;
    Serial.println("Parse class test:");
    Serial.print("rData: ");
    Serial.println(p.getData());
    parse_cnt = p.doParse();
    Serial.print("parsed ");
    Serial.print(parse_cnt);
    Serial.println(" items");
    for(int i=0; i< parse_cnt; i++) {
      Serial.println(p.getElement(i));
    }
    Serial.println("end of items");
    //END REMOVE--------------------
    */
    alarm1.clearSensorReported();
    digitalWrite(MESSAGE_PIN, HIGH);       //blink red led on publish
    delay(LOOP_DELAY_TIME); //use two delays to show led
  }
  delay(LOOP_DELAY_TIME);
  digitalWrite(MESSAGE_PIN, LOW);       //blink red led on publish
  #ifdef SERIAL_LOOP_WAIT
    while(!Serial.available())  // Wait here until the user presses ENTER
      Spark.process();          // in the Serial Terminal. Call the BG Tasks
    Serial.read();
  #endif
}
#endif  //PHOTON_REMOTE
