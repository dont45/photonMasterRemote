/*
  @file     parms.h
  @author   Don Thompson (raynham Engineering)
  @license  GNU (see license.txt)

  Configuration parameters
  Copyright (C) 2016 Don Thompson, Raynham Engineering

  Raynham MA
 */

#ifndef __PARMS_H__
#define __PARMS_H__

#define EDT_OFFSET -4
#define EST_OFFSET -5

#define PHOTON_MASTER
#ifndef PHOTON_MASTER
#define PHOTON_REMOTE
#endif

//#define HOUSE_CONFIGURATION         //use this for HOUSE
//which configuration?  (ONLY ONE)
//Testing Master-Remote WITH Shed as master
#ifdef PHOTON_MASTER
#define EE_MAGIC_CONFIG 0X3F        // change this to write new test config
#define EE_MAGIC_STATE 0X3F
#define PUSHOVER_SEND_MESSAGES      //actually send Pushover Messages
#define WORRY_MINUTES 5             //time between follow-up notifications
#define HOME_CONFIGURATION
#define MCP9808_I2CADDR  0X19       //cranky_jester
#endif
#ifdef PHOTON_REMOTE
#define REMOTE_SYSTEM_ID 1
#define SHED_CONFIGURATION
#define EE_MAGIC_CONFIG 0X30        // change this to write new test config
#define EE_MAGIC_STATE 0x03
//#define PUSHOVER_SEND_MESSAGES      //DEBUG?? actually send Pushover Messages
#define WORRY_MINUTES 5             //time between follow-up notifications
#define MCP9808_I2CADDR 0x1A        //scrper_neshed 0x1A  A1 HIGH, A0,A2 LOW
#endif
//bench-board test configuration
#ifdef TEST_CONFIGURATION
#define MAXDEVICE 1
#define TEST_SENSOR_1
#endif
//config for neshed
#ifdef HOME_CONFIGURATION
#define MAXDEVICE 10
#define TEST_SENSOR_1   //mcp9808
#define TEST_SENSOR_9   //FRONT DOOR
#define TEST_SENSOR_10  //basement water
#define TEST_SENSOR_13  //Remote SHED door
#define TEST_SENSOR_14  //Remote Outside Temp
#define TEST_SENSOR_16  //boiler temp
#define TEST_SENSOR_20  //GARAGE DOORS
#define TEST_SENSOR_21  //DINING ROOM IR
#define TEST_SENSOR_23  //INSIDE DOORS
#define TEST_SENSOR_24  //FAMILY RM SMOKE DETECTOR
#endif
//config for breadboard remote unit
#ifdef SHED_CONFIGURATION
#define MAXDEVICE 2
#define TEST_SENSOR_2   //SHED magnetic door
#define TEST_SENSOR_15  //mcp9808
#endif

#define CHECKIN_EMERGENCY_MESSAGE  "DON FAILED TO CHECK IN "
#define OUTSIDE_THERMOMETER_IDX 5   //for now use boiler temp
#define SENSOR_LIST_SCAN
#define ALERT_HOURS 1               //??TESTING, S/B longer (3 or 6)
#define CHECKIN_HOURS 0b000010000100010000000000;  //10am 2pm 7pm
#define LOOP_DELAY_TIME 250         //Delay ms between loop passes
#define REMOTE_LOOP_DELAY_TIME 1000 //Delay ms between Loop on Remotes
#define MAX_ALARM_NOTIFIED 1000     //Global limit on published alarms (DEBUG limit                        )
#define SYSTEM_STATUS_PIN D2        //RED if Armed, Blinking if any tripped
#define SYSTEM_NOTIFICATION_PIN D3  //GREEN blinks at each loop ??
#define TRIGGER_PIN D4              //debug pin for logic analyzer trigger
#define LOOP_SENSOR_PIN D4          //magnetic door loops
#define MOTION_LOOP_SENSOR_PIN D5   //motion detector
#define TAMPER_PIN D6               //system tamper switch loop
#define MESSAGE_PIN D7              //on-board led to show message limts reachec
#define RANDOM_MIN 1000             //range of random number for secret
#define RANDOM_MAX 9999
#define BAD_SECRET_MAX 3            //maximum bad secret tries in a row
#define MESSAGE_LOOP_DELAY 1        //number of loop passes btw notifications = 5secU
#define MAX_MESSAGE_PER_HOUR 100    //absolute maximum of pushover messages per hour
#define CLEARING_COUNT_MAX 60       //no. loops passes in notifying state
#define DEVICE_TIMEOUT 1800         // HALF HOUR
#define MAX_ALLOW_OWDEV  20         //absolute maximum of ow devices
#define FMLY_IBUTTON 0x01           //NOT USED ???
#define FMLY_2405 0x05
#define FMLY_2406 0x12              //and FMLY_2407
#define FMLY_1820 0x10
#define USE_FMLY_IBUTTON
#define USE_FMLY_2405
#define USE_FMLY_2406
#define USE_FMLY_1820
#define PIO_A 0
#define PIO_B 1
#define SENSOR_NAME_SIZE 16
#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)
//#define WD_EN 5                   // WatchDog enable PIN
//#define WD_WDI 6                  // WatchDog Timer Reset Input
#endif
