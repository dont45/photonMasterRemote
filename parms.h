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

#define SYSTEM_VERSION_MAJOR 7
#define SYSTEM_VERSION_MINOR 0
#define SYSTEM_VERSION_SUB 3

// BOSTON
#define EDT_OFFSET -4
#define EST_OFFSET -5
// LAS VEGAS
#define PST_OFFSET -8
#define PDT_OFFSET -7
#define TIME_OFFSET PDT_OFFSET
#define SYSTEM_CONFIGURATION
//  COMPILES WITH BOTH
// <<<>>>Define which to compile here
#define PHOTON_MASTER   // <<<=== Build MASTER or Remotes
// <<<>>> Define MASTER OR REMOTE on Line Above
#ifndef PHOTON_MASTER
#define PHOTON_REMOTE
#define PHOTON_REMOTE_1   // GARAGE REMOTE on Argon:
//#define PHOTON_REMOTE_2 // AIRSTREAM on BORON
#endif

//all debugs defined here
#ifdef PHOTON_REMOTE
#define SERIAL_DEBUG
//#define SERIAL_DEBUG_REMOTE
//#define SERIAL_DEBUG_OW
//#define SERIAL_DEBUG_WIRE
//#define DEBUG_AQUIRE
//#define SERIAL_DEBUG_SEN
//#define SERIAL_DEBUG_THERM
//#define SERIAL_DEBUG_ALARM
//#define SERIAL_DEBUG_CLOUD
//#define HARDWARE_WATCHDOG
//#define DEBUG_PARSE_ELEMENT
//#define DEBUG_PARSE_DETAIL
//this disables all but ONE sensor <<<===
//#define DEBUG_ONLY_ONE_SENSOR
//#define DEBUG_THIS_SENSOR 3
//#define SERIAL_DEBUGXX
//#define SERIAL_DEBUG_EVENT
//#define SERIAL_DEBUG_CONFIG
//#define SERIAL_WAIT
//#define SERIAL_LOOP_WAIT
#define HALT_ON_HDW_ERROR          //DISABLE FOR TESTING
#endif

//Testing Master-Remote WITH NEShed as master
#ifdef PHOTON_MASTER
#define REMOTE_SYSTEM_ID 0        // NOT a remote
#define SERIAL_DEBUG
//#define SERIAL_DEBUG_REMOTE
//#define SERIAL_DEBUG_WIRE
//#define SERIAL_DEBUG_OW
//#define SERIAL_DEBUG_I2C
//#define DEBUG_PARSE_ELEMENT
//#define COMMAND_DEBUG               // zero confirm on ALL
//#define MQTT_REMOTES                // using MQTT messaging
#define CHECKIN_DISABLED
#define EE_MAGIC_STATE 0x01
#define PUSHOVER_SEND_MESSAGES      //actually send Pushover Messages
#define WORRY_MINUTES 5             //time between follow-up notifications
//#define VERDI_FALLS_CONFIGURATION
#define REMOTE_NAMES "master","garage","camper","truck","greatrm","kitchen","",""
#define HARDWARE_REQUIRED_MCP9808
#define HARDWARE_REQUIRED_DS2482
//#define HALT_ON_HDW_ERROR          // skip halt for debug, 06/11/2021
#define MCP9808_I2CADDR  0X19       //cranky_jester
//#define MCP9808_I2CADDR  0X1A       //scrper_neshed  <<<MASTER UNIT
#endif

#ifdef PHOTON_REMOTE_1              //GARAGE on Argon BB
#define REMOTE_SYSTEM_ID 1
//#define GARAGE_CONFIGURATION
#define EE_MAGIC_STATE 0x05
#define WORRY_MINUTES 5             //time between follow-up notifications
#define HARDWARE_REQUIRED_MCP9808
#define HALT_ON_HDW_ERROR          // skip halt for debug, 06/11/2021
#define MCP9808_I2CADDR 0x18       //scrper_neshed 0x1A  A1, A0,A2 LOW
#define REMOTE_SENSOR_CHECK_MINUTES 10 //USE 15
//#define SERIAL_DEBUG_SEN_READ
//#define SERIAL_DEBUG_CONFIG
//#define DEBUG_ONLY_ONE_SENSOR
//#define DEBUG_THIS_SENSOR 0
#endif

#ifdef PHOTON_REMOTE_2              //AIRSTREAM on New Boron
#define REMOTE_SYSTEM_ID 2
#define HARDWARE_REQUIRED_MCP9808
#define MCP9808_I2CADDR 0x19        //cranky 0x19  A0 HIGH, A1,A2 LOW
//#define HARDWARE_REQUIRED_DS2482    // using one-wire
#define HALT_ON_HDW_ERROR           // skip halt for debug, 06/11/2021
#define EE_MAGIC_STATE 0x05
#define WORRY_MINUTES 15     // used in REMOTE?? time between follow-up notifications
#define REMOTE_SENSOR_CHECK_MINUTES 2 // debug: set to 10
#endif

//bench-board test configuration
//CONVERT TO IN-WALL living room controller
#ifdef SYSTEM_CONFIGURATION
#define MAXDEVICE 8
#define EE_MAGIC_CONFIG 0x06       // change this to write new test config
#define DEV_SENSOR_1               // GARAGE OVERHEAD
#define DEV_SENSOR_2               // GARAGE TEMP
#define DEV_SENSOR_10              // RED DS2013 OUTSIDE PATIO
#define DEV_SENSOR_11              // BLUE DS2013 INSIDE LR
#define DEV_SENSOR_12              // ORANGE: LR MOTION
#define DEV_SENSOR_13              // BLACK: LR SLIDER
#define DEV_SENSOR_14              // AIRSTREAM DOOR
#define DEV_SENSOR_15              // AIRSTREAM TEMP
#endif
#define INSIDE_THERMOMETER_IDX 3    //MAIN INSIDE TEMP
#define OUTSIDE_THERMOMETER_IDX 2   //MAIN OUTSIDE TEMP
#define GARAGE_THERMOMETER_IDX 1    //GARAGE TEMP
#define CAMPER_THERMOMETER_IDX 7    //CAMPER TEMP
#define SENSOR_LIST_SCAN
#define ALERT_HOURS 1               //??TESTING, S/B longer (3 or 6)
#define CHECKIN_HOURS 0b000000000000000000000000;  //10am 2pm 8pm
#define CHECKIN_EMERGENCY_MESSAGE  "DON FAILED TO CHECK IN "
#define LOOP_DELAY_TIME 250         //250: Debug: Delay ms between loop passes
#define REMOTE_LOOP_DELAY_TIME 250  //250: Debug: Delay ms between Loop on Remotes
//ledStatusBlink is style of blink: none,slow,fast
#define FAST_LED_BLINK_CNT 0
#define SLOW_LED_BLINK_CNT 3
#define MAX_ALARM_NOTIFIED 1000     //Global limit on published alarms (DEBUG limit                        )
#define SYSTEM_STATUS_PIN D2        //RED if Armed, Blinking if any tripped
#define SYSTEM_NOTIFICATION_PIN D3  //GREEN blinks at each loop ??
#define TRIGGER_PIN D6              //debug pin for logic analyzer trigger
//TODO: REVIEW USE OF THE NEXT THREE PINS ??
#define LOOP_SENSOR_PIN D6          //magnetic door loops
#define MOTION_LOOP_SENSOR_PIN D6   //motion detector
#define TAMPER_PIN D6               //system tamper switch loop
#define MESSAGE_PIN D7              //on-board led to show message limts reachec
#define RANDOM_MIN 1000             //range of random number for secret
#define RANDOM_MAX 9999
#define BAD_SECRET_MAX 3            //maximum bad secret tries in a row
// FOR PRODUCTION, MESSAGE_LOOP_DELAY S/B 1
#define MESSAGE_LOOP_DELAY 0        //number of loop passes btw notifications = 5secU
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
