/* notifier.h
 *
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
struct Message {
  uint8_t header_style;
  uint8_t priority;
  String message_text;
};

 class Notifier {
 public:
   Notifier(char *event_name, char *priority_event_name, Sensor *s) {
     // move upd to Alarm class ??
     Particle.function("upd", &Notifier::upd, this);
     //Particle.function("set", &Notifier::set, this);  //old xxxxyy, remove
     Particle.function("request", &Notifier::request, this);
     Particle.function("confirm", &Notifier::confirm, this);
     //Particle.function("dump", &Notifier::dump, this); xxxxyy
     strcpy(webhook_id, event_name);
     strcpy(priority_webhook_id, priority_event_name);
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
   void sendMessage(uint8_t header, uint8_t priority, char* msg);  //send message to pushover
   void sendMessage(uint8_t, uint8_t, String);
   void queueMessage(uint8_t, uint8_t, String);    //add message to queue
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
   std::queue<Message> message_queue; // replaces msg_queue (with priority)
   char webhook_id[20];
   char priority_webhook_id[20];
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
