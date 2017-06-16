/*
@file     notifier.h
@author   Don Thompson (raynham Engineering)
@license  GNU (see license.txt)

Notifier class handles user notifications
Handles user alerts via Pushover and communication directly to iPhone app
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
*/

 #ifndef __NOTIFIER_H__
 #define __NOTIFIER_H__

 #include "application.h"
 #include "util.h"
 #include "parms.h"
 #include "alarm.h"
 #include <queue>
 #include <list>
 #include <map>

typedef enum {NO_HEADER=0, SHORT_HEADER=1, FULL_HEADER=2};
//Message name conflicts with
struct pMessage {
  uint8_t header_style;
  uint8_t priority;
  String message_text;
};

 class Notifier {
 public:
   Notifier(char *event_name, char *priority_event_name, char *emergency_event_name, Sensor *s) {
     // move upd to Alarm class ??
     Particle.function("upd", &Notifier::upd, this);
     //Particle.function("set", &Notifier::set, this);  //old xxxxyy, remove
     Particle.function("request", &Notifier::request, this);
     Particle.function("confirm", &Notifier::confirm, this);
     //Particle.function("dump", &Notifier::dump, this); xxxxyy
     //TODO use array for webhook_id
     strcpy(webhook_id, event_name);
     strcpy(priority_webhook_id, priority_event_name);
     strcpy(emergency_webhook_id, emergency_event_name);
     p_sensor = s;
     msg_limit = 0;        //limit mesages per hour
     secret_timeout = 0;   //loop passes to timeout secret
     bad_secret_cnt = 0;
     alarm_notify_cnt = 0;
     alarm_times_notified = 0;
     hours_between_alert = ALERT_HOURS;
     digitalWrite(MESSAGE_PIN, LOW);
   }
   int upd(String command);
   int set(String);
   String updData();
   int request(String);
   int confirm(String);
   int do_command(String);
   //int dump(String); xxxxyy
   void sendMessage(uint8_t header, int priority, char* msg);  //send message to pushover
   void sendMessage(uint8_t, int, String);
   void queueMessage(uint8_t, int, String);    //add message to queue
   bool msgqueueEmpty();
   void dequeMessage(void);      //get next message from queue and send
   void hourlyReset();
   void pushAlarm(char* msg);
   void secret_countdown();
   void setStartTime();
   void setAlertHours(int h);
   uint8_t getAlertHours();
   bool checkTime();

 private:
   const static String commands;
   std::map <int, String> command_list;
   //std::queue<String> msg_queue;
   std::queue<pMessage> message_queue; // replaces msg_queue (with priority)
   char webhook_id[20];
   char priority_webhook_id[20];
   char emergency_webhook_id[20];
   char event_message[1000];   //??reduce size of this ??
   // to do, clear secret after xx loops ??? DEBUG
   int secret;
   unsigned int secret_timeout;
   uint8_t bad_secret_cnt;
   int alarm_times_notified;
   int alarm_notify_cnt;
   int msg_limit;
   int hour = 0;
   int minute = 0;
   int lasthour = 0;   //use -1 to skip startup
   int lastminute = 0;
   int elapsedminutes = 0;
   uint8_t hours_between_alert;
   unsigned long lastSync = millis();
   Sensor *p_sensor;
   Alarm *p_alarm;  //??
 };
#endif
