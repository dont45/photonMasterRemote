/*
  weather.cpp
  system weathers
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

#include "weather.h"

Weather::Weather(){
  todaysLowTemp = 999;
  todaysHighTemp = -999;
  yesterdayLowTemp = 999;
  yesterdayHighTemp = 999;
  longtermHigh = 0;
  longtermLow = 0;
}

//Weather Report
char* Weather::weatherReport(char *buf) {
  char temp[80];
  buf[0]=0;
  strcat(buf,"Todays Weather\n");
  sprintf(temp,"Now  %5.1fF\n", currentTemp);
  strcat(buf,temp);
  sprintf(temp,"Low  %5.1fF\n", todaysLowTemp);
  strcat(buf,temp);
  sprintf(temp,"High %5.1fF\n", todaysHighTemp);
  strcat(buf,temp);
  if(yesterdayLowTemp != 999) {
    strcat(buf,"yesterday\n");
    sprintf(temp,"Low  %5.1fF\n", yesterdayLowTemp);
    strcat(buf,temp);
    sprintf(temp,"High %5.1fF\n", yesterdayHighTemp);
    strcat(buf,temp);
  }
   return buf;
}
//set's todaysLow/High
void Weather::setHiLo(float curTemp) {
  timeLastTemp = Time.now();
  if(lastHour > Time.hour()) setNewDay();
  lastHour = Time.hour();
  currentTemp = curTemp;
  if(curTemp > todaysHighTemp) todaysHighTemp = curTemp;
  if(curTemp < todaysLowTemp) todaysLowTemp = curTemp;
}

//copies today's to yesterday
void Weather::setNewDay(){
  yesterdayLowTemp = todaysLowTemp;
  yesterdayHighTemp = todaysHighTemp;
  todaysLowTemp = 999;
  todaysHighTemp = -999;
}

float Weather::getTodaysLow(){return todaysLowTemp;}
float Weather::getTodaysHigh(){return todaysHighTemp;}
float Weather::getYesterdqyLow(){return yesterdayLowTemp;}
float Weather::getYesterdqyHigh(){return yesterdayHighTemp;}
float Weather::getlongtermHigh(){return longtermHigh;}
float Weather::getlongtermLow(){return longtermLow;}
