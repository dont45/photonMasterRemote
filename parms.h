/* parms.h
 *
 * Configuration parameters
 */
#ifndef __PARMS_H__
#define __PARMS_H__

#define EDT_OFFSET -4
#define EST_OFFSET -5

//#define PHOTON_MASTER
#ifndef PHOTON_MASTER
#define PHOTON_REMOTE
#endif

//#define HOUSE_CONFIGURATION         //use this for HOUSE
//which configuration?  (ONLY ONE)
//Testing Master-Remote WITH Shed as master
#ifdef PHOTON_MASTER
#define EE_MAGIC_CONFIG 0X06        // change this to write new test config
#define EE_MAGIC_STATE 0x9e
#define PUSHOVER_SEND_MESSAGES      //actually send Pushover Messages
#define WORRY_MINUTES 5             //time between follow-up notifications
#define SHED_CONFIGURATION
#define MCP9808_I2CADDR                0X1A     //0x1A   //A1 HIGH, A0,A2 LOW
#endif
#ifdef PHOTON_REMOTE
#define REMOTE_CONFIGURATION
#define EE_MAGIC_CONFIG 0X25        // change this to write new test config
#define EE_MAGIC_STATE 0x01
#define PUSHOVER_SEND_MESSAGES      //DEBUG?? actually send Pushover Messages
#define WORRY_MINUTES 2             //time between follow-up notifications
#define MCP9808_I2CADDR                0X19
#endif
//bench-board test configuration
#ifdef TEST_CONFIGURATION
#define MAXDEVICE 1
#define TEST_SENSOR_1
#endif
//config for HOUSE
#ifdef HOUSE_CONFIGURATION
#define MAXDEVICE 1
#define TEST_SENSOR_1
#endif
//config for neshed
#ifdef SHED_CONFIGURATION
#define MAXDEVICE 4
#define TEST_SENSOR_1
#define TEST_SENSOR_2
#define TEST_SENSOR_13  //REMOTE door
#define TEST_SENSOR_14  //REMOTE temp
#endif
//config for breadboard remote unit
#ifdef REMOTE_CONFIGURATION
#define MAXDEVICE 2
#define TEST_SENSOR_9
#define TEST_SENSOR_15
#endif

#define SENSOR_LIST_SCAN
#define ALERT_HOURS 3               //no longer used ??
#define LOOP_DELAY_TIME 1000        //Delay ms between loop passes
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
#define MAX_ALLOW_OWDEV  14         //absolute maximum of ow devices
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
//#define SHORT_NAME_SIZE 2
#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)
//#define WD_EN 5                   // WatchDog enable PIN
//#define WD_WDI 6                  // WatchDog Timer Reset Input
#endif
