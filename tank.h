/*
  @file     tank.h
  @author   Don Thompson (raynham Engineering)
  @license  GNU (see license.txt)
  oil tank class manages the heating oil sensor
  calculates gallons remaining in tank based upon sensor level
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

#ifndef __TANK_H__
#define __TANK_H__
//define some standard home heating oil tank sizes
typedef enum {VERTICAL_275, HORIZONTAL_275, VERTICAL_330, HORIZONTAL_330}
TANK_STYLES;
const char* TANK_NAME[4] = {"Vertical 275", "Horizontal 275", "Vertical 330", "Horizontal 330"};

#define WIDTH 0
#define DIAMETER 1
#define LENGTH 2

#define CVT_IN2_GALLONS 0.004329

static const float tank_d[4][3] = {{27.0, 44.0, 60.0},  //VERTICAL_275
                                   {27.0, 44.0, 60.0},  //HORIZONTAL_275
                                   {27.0, 44.0, 72.0},  //VERTICAL_330
                                   {44.0, 27.0, 72.0}}; //HORIZONTAL_330
static const float pi = 3.14159;

class Tank {
public:
    Tank(uint8_t);
    int calcOilLevel(int);
    float tankGallons(int);

private:
    int tankVolume(int);
    int fullVolume();
    float fullGallons();
    float calcCurved(int);
    float curReading;	//sensor Reading in mm
    float curLevel;		//oil level in inches
    uint8_t tankStyle;
    int width;
    int diameter;
    float length;
    float radius;
    int volumeIN2;  //volume in cubic inches
    int cur_gallons;  //current tank gallons
};
#endif
