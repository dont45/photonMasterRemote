/*
  @file     state.h
  @author   Don Thompson (raynham Engineering)
  @license  GNU (see license.txt)

  Weather class manages a simple weather station
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

#ifndef __WEATHER_H__
#define __WEATHER_H__

#include "application.h"
#include "parms.h"

class Weather {
public:
  Weather();
  char *weatherReport(char *buf);
  void setHiLo(float);       //set's todaysLow/High
  void setNewDay();    //copies today's to yesterday
  float getTodaysLow();
  float getTodaysHigh();
  float getYesterdqyLow();
  float getYesterdqyHigh();
  float getlongtermHigh();
  float getlongtermLow();
private:
  int timeLastTemp;
  uint8_t lastHour;
  float currentTemp;
  float todaysLowTemp;
  float todaysHighTemp;
  float yesterdayLowTemp;
  float yesterdayHighTemp;
  float longtermHigh;
  float longtermLow;
};
#endif
