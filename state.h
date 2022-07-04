/*
  @file     state.h
  @author   Don Thompson (raynham Engineering)
  @license  GNU (see license.txt)

  system states
  State class manages system states, principally during startup
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

#ifndef __STATE_H__
#define __STATE_H__
#define SECS_IN_DAY 86400
#define SECS_IN_HOUR 3600
#define SECS_IN_MINUTE 60

#include "application.h"
#include "parms.h"

enum {sys_undefined=0, sys_starting=1,sys_configure=2,sys_fail=3,sys_running=4};
//sys status bit definitions:
// 1 == HDW FAIL
// 2 == DS2482
// 4 == EEPROMCFG
// 8 == OWDEVICE
enum {fail_hardware=1, fail_ds2482=2, fail_eeprom=4, fail_owdevice=8};
class State {
public:
  State();
  void sysState(uint8_t);
  uint8_t sysStatus();
  void setRemoteStatus(uint8_t remoteId, bool newState); 
  bool getRemoteStatus(uint8_t);
  void setRemotes(uint8_t);
  void setRemoteName(uint8_t, String);
  String getRemoteName(uint8_t);
  uint8_t getRemotes();
  void setSysId(uint8_t);
  uint8_t getSysId();
  void addStatus(uint8_t);
  bool owPresent();
  char* upTime(char *);
  int upTime();
private:
  uint8_t sysId;  //master = 0, remotes 1,2,...
  int ledNoticeState = 0;
  uint8_t curState;
  uint8_t status;
  uint8_t remoteStatus; //bit-mask of remote nodes
  String remoteNames[8];
  int startTime;
};
#endif
