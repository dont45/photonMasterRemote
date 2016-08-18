/*
  hwd.cpp
  hardware watchdog
  Don Thompson
  Raynham MA

  Copyright (C) 2016 Donald Thompson and Raynhyam Engineering
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

#include "hwd.h"
#include "application.h"

HWD::HWD(){
  pinMode(HWD_WDI_PIN, OUTPUT);
  pinMode(HWD_EN_PIN, OUTPUT);
  digitalWrite(HWD_EN_PIN, HIGH);
  digitalWrite(HWD_WDI_PIN, LOW);
}

void HWD::disable() {
  digitalWrite(HWD_EN_PIN, HIGH);
}

void HWD::enable() {
  digitalWrite(HWD_EN_PIN, LOW);
}

void HWD::checkin() {
  digitalWrite(HWD_WDI_PIN, HIGH);
  delay(1);
  digitalWrite(HWD_WDI_PIN, LOW);
}
