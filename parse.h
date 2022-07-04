/*
  @file     parse.h
  @author   Don Thompson (raynham Engineering)
  @license  GNU (see license.txt)

  remote command parser
  parse data String from remote alarm
  3@T@13:12:16@TEST DOOR:2:0.000000:3@OUTSIDE TEMP:3:89.937500:0$
  0@T@10:24:43@TEST DOOR:2:0.000000:3@OUTSIDE TEMP:3:81.724998:0$
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

#ifndef __PARSE_H__
#define __PARSE_H__
//Aug 16, 2016

#include "application.h"
#include "util.h"
#include "parms.h"

#define MAX_PARSED_ITEMS 8
#define ELEMENT_DELIMITER '@'
#define END_DELIMITER '$'
#define FIELD_DELIMITER ':'
#define FIELD_END '^'

class Parse {
public:
  Parse(String);
  Parse(char, char, String);
  int doParse();
  void setData(String);
  void setData(char, char, String);
  void setDelimiters(char, char);
  String getData();
  String getElement(int);
  String el1, el2, el3, el4;
private:
  int findDelim(char, int);
  int parseNext(int, int);

  String remotedata;
  String element[MAX_PARSED_ITEMS];

  int parsePos;
  int idx;
  char elementDelimiter;
  char endDelimiter;
};

#endif
