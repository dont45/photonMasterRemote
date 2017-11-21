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

//  COMPILES WITH BOTH
#define PHOTON_MASTER
#ifndef PHOTON_MASTER
#define PHOTON_REMOTE
//#define PHOTON_REMOTE_1
#define PHOTON_REMOTE_2
#endif

//#define HOUSE_CONFIGURATION         //use this for HOUSE
//which configuration?  (ONLY ONE)
//Testing Master-Remote WITH Shed as master
#ifdef PHOTON_MASTER
#define HARDWARE_REQUIRED_MSCP9808
#define HARDWARE_REQUIRED_DS2482
#define EE_MAGIC_CONFIG 0X47        // change this to write new test config
#define EE_MAGIC_STATE 0X43
#define PUSHOVER_SEND_MESSAGES      //actually send Pushover Messages
#define WORRY_MINUTES 5             //time between follow-up notifications
#define HOME_CONFIGURATION
#define MCP9808_I2CADDR  0X19       //cranky_jester
#endif
#ifdef PHOTON_REMOTE_1
#define REMOTE_SYSTEM_ID 1
#define SEED_CONFIGURATION
#define EE_MAGIC_CONFIG 0X08        // change this to write new test config
#define EE_MAGIC_STATE 0x00
//#define PUSHOVER_SEND_MESSAGES    //DEBUG?? actually send Pushover Messages
#define WORRY_MINUTES 5             //time between follow-up notifications
#define MCP9808_I2CADDR 0x1A        //scrper_neshed 0x1A  A1 HIGH, A0,A2 LOW
#define REMOTE_SENSOR_CHECK_MINUTES 15
#endif
#ifdef PHOTON_REMOTE_2
#define REMOTE_SYSTEM_ID 2
#define SHED_CONFIGURATION
#define HARDWARE_REQUIRED_MSCP9808
#define HARDWARE_REQUIRED_DS2482
#define HALT_ON_HDW_ERROR
#define EE_MAGIC_CONFIG 0x19        // change this to write new test config
#define EE_MAGIC_STATE 0x01
#define WORRY_MINUTES 5             //time between follow-up notifications
#define MCP9808_I2CADDR 0x1A        //scrper_neshed 0x1A  A1 HIGH, A0,A2 LOW
#define REMOTE_SENSOR_CHECK_MINUTES 10
#endif

//bench-board test configuration
#ifdef TEST_CONFIGURATION
#define MAXDEVICE 1
#define TEST_SENSOR_1
#endif
//config for neshed
#ifdef HOME_CONFIGURATION

#define MAXDEVICE 13
#define TEST_SENSOR_1   //0  mcp9808
#define TEST_SENSOR_2   //1  Shed Door
#define TEST_SENSOR_9   //2  FRONT DOOR
#define TEST_SENSOR_10  //3  basement water
#define TEST_SENSOR_13  //4  Remote SEED
#define TEST_SENSOR_14  //5  Remote SENSOR ENABLE
#define TEST_SENSOR_15  //6  Shed Outside Temp
#define TEST_SENSOR_16  //7  boiler temp
#define TEST_SENSOR_20  //8  GARAGE DOORS
#define TEST_SENSOR_21  //9  DINING ROOM IR
#define TEST_SENSOR_23  //10 INSIDE DOORS
#define TEST_SENSOR_24  //11 FAMILY RM SMOKE DETECTOR
#endif
//config for breadboard remote unit
#ifdef SHED_CONFIGURATION
#define MAXDEVICE 2
#define TEST_SENSOR_27  //SHED magnetic door
#define TEST_SENSOR_28  //mcp9808
#endif

//config for our first electron seed node
#ifdef SEED_CONFIGURATION
#define MAXDEVICE 2
#define TEST_SENSOR_25  //port sensor ON D5
#define TEST_SENSOR_26  //set-alarm on D6
#endif

#define CHECKIN_EMERGENCY_MESSAGE  "DON FAILED TO CHECK IN "
#define OUTSIDE_THERMOMETER_IDX 6   //SHED
#define SENSOR_LIST_SCAN
#define ALERT_HOURS 1               //??TESTING, S/B longer (3 or 6)
#define CHECKIN_HOURS 0b000100000100010000000000;  //10am 2pm 8pm
#define LOOP_DELAY_TIME 250         //Delay ms between loop passes
#define REMOTE_LOOP_DELAY_TIME 250 //Delay ms between Loop on Remotes
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
