/*
  DS2482 library for Arduino
  Copyright (C) 2009 Paeae Technologies

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

#ifndef __DS2482_H__
#define __DS2482_H__

#include "application.h"
#include <inttypes.h>
//#include <HardwareSerial.h>

// NOT USED ..allow for serial printing for debug
//#ifndef DEBUG_SERIAL
//#define DEBUG_SERIAL 1
//#endif

// debug prints for wireSearch
#ifndef DEBUG_WIRESEARCH
#define DEBUG_WIRESEARCH 0 //<<<===serial IS Arduino
#endif

// you can exclude onewire_search by defining that to 0
#ifndef ONEWIRE_SEARCH
#define ONEWIRE_SEARCH 1
#endif

// You can exclude CRC checks altogether by defining this to 0
#ifndef ONEWIRE_CRC
#define ONEWIRE_CRC 1
#endif


#define DS2482_CONFIG_APU (1<<0)
#define DS2482_CONFIG_PPM (1<<1)
#define DS2482_CONFIG_SPU (1<<2)  //STRONG PULL-UP, CONTROLS PCTLZ PULLUP CONTROL
#define DS2484_CONFIG_WS  (1<<3)

#define DS2482_STATUS_BUSY 	(1<<0)
#define DS2482_STATUS_PPD 	(1<<1)
#define DS2482_STATUS_SD	(1<<2)
#define DS2482_STATUS_LL	(1<<3)
#define DS2482_STATUS_RST	(1<<4)
#define DS2482_STATUS_SBR	(1<<5)
#define DS2482_STATUS_TSB	(1<<6)
#define DS2482_STATUS_DIR	(1<<7)

class DS2482
{
public:
	//Address is 0-3
	DS2482(uint8_t address);
	bool configure(uint8_t config);
	void reset();

	//DS2482-800 only
	bool selectChannel(uint8_t channel);

	bool wireReset(); // return true if presence pulse is detected
	uint8_t wireReadStatus(bool setPtr=false);
	uint8_t wireReadConfig();
	uint8_t wireStatus() { return statusByte; };

	void wireWriteByte(uint8_t b);
	uint8_t wireReadByte();

	void wireWriteBit(uint8_t bit);
	uint8_t wireReadBit();
  // Issue a 1-Wire rom select command, you do the reset first.
  void wireSelect( uint8_t rom[8]);
	// Issue skip rom
	void wireSkip();

	uint8_t hasTimeout() { return mTimeout; }
#if ONEWIRE_SEARCH
    // Clear the search state so that if will start from the beginning again.
    void wireResetSearch();

    // Look for the next device. Returns 1 if a new address has been
    // returned. A zero might mean that the bus is shorted, there are
    // no devices, or you have already retrieved all of them.  It
    // might be a good idea to check the CRC to make sure you didn't
    // get garbage.  The order is deterministic. You will always get
    // the same devices in the same order.
    uint8_t wireSearch(uint8_t *newAddr);
    // set wireSearch for SearchROM
    void setStdSearch() { searchType = 0xF0; };
    // set wireSearch for ConditionalSearchROM
    void setCondSearch() { searchType = 0xEC;};
#endif
    uint8_t wireMatchRom(uint8_t *ROM);
    uint8_t status() { return statusByte; };
    bool ppd() { return statusByte & DS2482_STATUS_PPD ? true : false; };
#if ONEWIRE_CRC
    // Compute a Dallas Semiconductor 8 bit CRC, these are used in the
    // ROM and scratchpad registers.
    static uint8_t crc8( uint8_t *addr, uint8_t len);
#endif

private:
	uint8_t statusByte;
protected:

	uint8_t mAddress;
	uint8_t mTimeout;
	uint8_t readByte();
	void setReadPtr(uint8_t readPtr);

	uint8_t busyWait(bool setReadPtr=false); //blocks until
	void begin();
	void end();

#if ONEWIRE_SEARCH
	uint8_t searchType;
	uint8_t searchAddress[8];
	int8_t searchLastDiscrepancy;
	uint8_t searchExhausted;
#endif

#if DEBUG_SERIAL_REMOVED
	// Search State
	int LastDiscrepancy;
        int LastFamilyDiscrepancy;
   	int LastDeviceFlag;
	unsigned char vcrc8;
#endif
};
#endif
