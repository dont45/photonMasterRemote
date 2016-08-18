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

	crc code is from OneWire library

*/

#include "DS2482.h"

#define PTR_STATUS 0xf0
#define PTR_READ 0xe1
#define PTR_CONFIG 0xc3

#define WIRE_MATCH_ROM	0x55


DS2482::DS2482(uint8_t addr)
{
	mAddress = 0x18 | addr;
	searchType = 0xf0;	//default to searchROM
}

//-------helpers
void DS2482::begin()
{
	Wire.beginTransmission(mAddress);
	//Wire.begin();	//already begin called??
}

void DS2482::end()
{
	Wire.endTransmission();
}

void DS2482::setReadPtr(uint8_t readPtr)
{
	begin();
	// Here we should start Logic8 trigger (TRIGGER_PIN HIGH)
	//#define TRIGGER_PIN D3
	//digitalWrite(TRIGGER_PIN, HIGH); //DEBUG, trigger pin3 for logic 8 analyzer
	Wire.write(0xe1);
	Wire.write(readPtr);
	end();
}

uint8_t DS2482::readByte()
{
	Wire.requestFrom(mAddress,(uint8_t)1);
	return Wire.read();
}

/*
Status Register Bit Assignment
   bit 7  bit 6   bit 5  bit 4 bit 3 bit 2 bit 1 bit 0
   DIR    TSB     SBR    RST    LL   SD    PPD   1WB
*/
uint8_t DS2482::wireReadStatus(bool setPtr)
{
	if (setPtr)
		setReadPtr(PTR_STATUS);
	return readByte();
}

uint8_t DS2482::wireReadConfig()
{
	setReadPtr(PTR_CONFIG);
	return readByte();
}

uint8_t DS2482::busyWait(bool bsetReadPtr)
{
	uint8_t status;
	int loopCount = 100;
	while((status = wireReadStatus(bsetReadPtr)) & DS2482_STATUS_BUSY)
	{
		if (--loopCount <= 0)
		{
			mTimeout = 1;
			break;
		}
		delayMicroseconds(20);
	}
	return status;
}

//----------interface
// status bits:
/*
The PPD bit is updated with every 1-Wire Reset command. If the DS2482 detects a presence pulse from a 1-Wire
device at tMSP during the Presence Detect cycle, the PPD bit will be set to 1. This bit returns to its default 0 if there
is no presence pulse or if the 1-Wire line is shorted during a subsequent 1-Wire Reset command.
Short Detected (SD)

The SD bit is updated with every 1-Wire Reset command. If the DS2482 detects a logic 0 on the 1-Wire line at tSI
during the Presence Detect cycle, the SD bit is set to 1. This bit returns to its default 0 with a subsequent 1-Wire
Reset command provided that the short has been removed. If SD is 1, PPD is 0. The DS2482 cannot distinguish
between a short and a DS1994 or DS2404 signaling a 1-Wire interrupt. For this reason, if a DS2404/DS1994 is
used in the application, the interrupt function must be disabled. The interrupt signaling is explained in the
respective device data sheets.
*/
void DS2482::reset()
{
	mTimeout = 0;
	begin();
	Wire.write(0xf0);
	end();
}

bool DS2482::configure(uint8_t config)
{
	busyWait(true);
	begin();
	Wire.write(0xd2);
	Wire.write(config | (~config)<<4);
	end();

	return readByte() == config;
}

bool DS2482:: selectChannel(uint8_t channel)
{
	uint8_t ch, ch_read;

	switch (channel)
	{
		case 0:
		default:
			ch = 0xf0;
			ch_read = 0xb8;
			break;
		case 1:
			ch = 0xe1;
			ch_read = 0xb1;
			break;
		case 2:
			ch = 0xd2;
			ch_read = 0xaa;
			break;
		case 3:
			ch = 0xc3;
			ch_read = 0xa3;
			break;
		case 4:
			ch = 0xb4;
			ch_read = 0x9c;
			break;
		case 5:
			ch = 0xa5;
			ch_read = 0x95;
			break;
		case 6:
			ch = 0x96;
			ch_read = 0x8e;
			break;
		case 7:
			ch = 0x87;
			ch_read = 0x87;
			break;
	};

	busyWait(true);
	begin();
	Wire.write(0xc3);
	Wire.write(ch);
	end();
	busyWait();

	uint8_t check = readByte();

	return check == ch_read;
}


// PPD and SD bits are updated by wireReset??

bool DS2482::wireReset()
{
	busyWait(true);
	begin();
	Wire.write(0xb4);
	end();
	// save this in class statusByte
//	uint8_t status = busyWait();
	statusByte = busyWait();

	return statusByte & DS2482_STATUS_PPD ? true : false;
}


void DS2482::wireWriteByte(uint8_t b)
{
	busyWait(true);
	begin();
	Wire.write(0xa5);
	Wire.write(b);
	end();
}

uint8_t DS2482::wireReadByte()
{
	busyWait(true);
	begin();
	Wire.write(0x96);
	end();
	busyWait();
	setReadPtr(PTR_READ);
	busyWait();
	return readByte();
}

void DS2482::wireWriteBit(uint8_t bit)
{
	busyWait(true);
	begin();
	Wire.write(0x87);
	Wire.write(bit ? 0x80 : 0);
	end();
}

uint8_t DS2482::wireReadBit()
{
	wireWriteBit(1);
	uint8_t status = busyWait(true);
	return status & DS2482_STATUS_SBR ? 1 : 0;
}

void DS2482::wireSkip()
{
	wireWriteByte(0xcc);
}

void DS2482::wireSelect(uint8_t rom[8])
{
	wireWriteByte(WIRE_MATCH_ROM);
	for (int i=0;i<8;i++)
		wireWriteByte(rom[i]);
}


#if ONEWIRE_SEARCH
void DS2482::wireResetSearch()
{
	searchExhausted = 0;
	searchLastDiscrepancy = -1;

	for(uint8_t i = 0; i<8; i++)
		searchAddress[i] = 0;
}

uint8_t DS2482::wireSearch(uint8_t *newAddr)
{
	uint8_t i;
	uint8_t direction = 0;
	uint8_t last_zero=0xff;

	if (searchExhausted)
	{
#if DEBUG_WIRESEARCH
		Serial.println("wireSearch: searchExhausted"); //xxx
#endif
		return 0;
	}

	if (!wireReset())
	{
#if DEBUG_WIRESEARCH
		Serial.println("wireSearch no wireReset");
#endif
		return 0;
	}
	busyWait(true);
	wireWriteByte(searchType);
#if DEBUG_WIRESEARCH
	Serial.print("Search: searchLastDiscrepancy=");
	Serial.println(searchLastDiscrepancy,DEC);
#endif
	for(i=0;i<64;i++)
	{
		int romByte = i/8;
		int romBit = 1<<(i&7);
#if DEBUG_WIRESEARCH
		Serial.print("wireSearch: i=");
		Serial.print(i,DEC);
#endif
		if (i < searchLastDiscrepancy)
		{
#if DEBUG_WIRESEARCH

			Serial.print("searchAddress byte:");
			Serial.print(searchAddress[romByte],BIN);
			Serial.print(" bit=");
			Serial.println(romBit, DEC);
#endif
//			direction = searchAddress[romByte] & romBit;
			if(searchAddress[romByte] & romBit)
			    direction = 1;
			else
			    direction = 0;
		}
		else
		{
#if DEBUG_WIRESEARCH
			Serial.print(" FORWARD");
#endif
//			direction = i == searchLastDiscrepancy;
			if(i == searchLastDiscrepancy)
			    direction = 1;
			else
			    direction = 0;
		}
#if DEBUG_WIRESEARCH
		Serial.print(" direction=");
		Serial.print(direction,DEC);
#endif
		busyWait();
		begin();
		Wire.write(0x78);
		Wire.write(direction ? 0x80 : 0);
		end();
		uint8_t status = busyWait();

//		uint8_t firstBit = status & DS2482_STATUS_SBR;
		uint8_t firstBit = ((status & DS2482_STATUS_SBR) == DS2482_STATUS_SBR) ? (byte)1 : (byte)0;
//		uint8_t secondBit = status & DS2482_STATUS_TSB;
		uint8_t secondBit = ((status & DS2482_STATUS_TSB) == DS2482_STATUS_TSB) ? (byte)1 : (byte)0;
//		direction = status & DS2482_STATUS_DIR;
		direction = ((status & DS2482_STATUS_DIR) == DS2482_STATUS_DIR) ? (byte)1 : (byte)0;
#if DEBUG_WIRESEARCH
		Serial.print(" firstBit=");
		Serial.print(firstBit, HEX);
		Serial.print(" secondBit=");
		Serial.print(secondBit, HEX);
		Serial.print(" direction=");
		Serial.print(direction,DEC);
#endif
		if (firstBit && secondBit)
		{
#if DEBUG_WIRESEARCH
			Serial.print(" point 1");
#endif			return 0;
		}
		else
		{
#if DEBUG_WIRESEARCH
			Serial.print(" point 2");
#endif
			// if 0 was picked then record its position in LastZero
			// if (firstBit==0 && secondBit==0 && dir == 0)
			if (!firstBit && !secondBit && !direction)
			{
#if DEBUG_WIRESEARCH
				Serial.print(" set last_zero=");
				Serial.print(i,DEC);
#endif
				last_zero = i;
			}
		}

		if (direction)
		{
			searchAddress[romByte] |= romBit;
#if DEBUG_WIRESEARCH
			Serial.println(" :one");
#endif
		}
		else
		{
			searchAddress[romByte] &= (uint8_t)~romBit;
#if DEBUG_WIRESEARCH
			Serial.println(" :zero");
#endif
		}
	}

	searchLastDiscrepancy = last_zero;

	if (last_zero == 0xff)
	{
#if DEBUG_WIRESEARCH
		Serial.print(": searchExhausted");
#endif
		searchExhausted = 1;
	}

#if DEBUG_WIRESEARCH
	Serial.print(": copyRom=");
#endif
	for (i=0;i<8;i++)
	{
#if DEBUG_WIRESEARCH
		Serial.print(searchAddress[i],HEX);
		Serial.print(":");
#endif
		newAddr[i] = searchAddress[i];
	}
#if DEBUG_WIRESEARCH
	Serial.print("\n");
#endif
	return 1;
}
#endif

/* match_rom selects device with id=ROM
   state of pio is returned
   Note that MatchRom causes a toggle to state of pio
 */
uint8_t DS2482::wireMatchRom(uint8_t *ROM)
{
	uint8_t tc;
	uint8_t i,j;
	wireReset();			// Added this reset ????
        wireSelect(ROM);
// ?? Do we need waitBusy() here ?? I don't think so...this is read of bus, not DS2482??

	tc = wireReadBit();		//pio pin is toggled, now read it
	return tc;			//and return pio state
}

#if ONEWIRE_CRC
// The 1-Wire CRC scheme is described in Maxim Application Note 27:
// "Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products"
//

uint8_t DS2482::crc8( uint8_t *addr, uint8_t len)
{
	uint8_t crc=0;

	for (uint8_t i=0; i<len;i++)
	{
		uint8_t inbyte = addr[i];
		for (uint8_t j=0;j<8;j++)
		{
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix)
				crc ^= 0x8C;

			inbyte >>= 1;
		}
	}
	return crc;
}

#endif

#if MY_MODS_MOVED

// read status block (8 bytes + crc) into buf
// device = ds2406
// SENSOR:0:12D8046000E9
// Family:12
// 2406
// deviceReadStatus:10
// Status=FF:FF:FF:FF:FF:0:0:7F:ED:C1:
// ok!!

XXuint8_t DS2482::deviceReadStatus(uint8_t *ROM, uint8_t *buf)
{
    // should verify family here ??
    unsigned char status;
    int i;
    wireReset();
    wireSelect(ROM);
    busyWait(true);
    wireWriteByte(0xAA);
    wireWriteByte(0x00);
    wireWriteByte(0x00);
    for(i=0; i<10; i++)
        *buf++ = wireReadByte();
}

// write status byte to DS2406
// SENSOR:0:12D8046000E9
// Family:12
// 2406 Before (power-on default=0X7F)
// Status=FF:FF:FF:FF:FF:0:0:7F:ED:C1:
// Channel Info:33
// deviceWriteStatus: crc=1FF6
// SENSOR:0:12D8046000E9
// Family:12
// 2406 After Status Byte Write (0x0F)
// Status=FF:FF:FF:FF:FF:0:0:F:EC:25:
// Channel Info:30
// deviceWriteStatus: crc=1FF6

uint8_t DS2482::deviceWriteStatus(uint8_t *ROM, uint8_t status)
{
    uint8_t crc1, crc2;
    bool ok;
    // should verify family here ??
    wireReset();
    wireSelect(ROM);
    busyWait(true);
    wireWriteByte(0x55);
    wireWriteByte(0x07);	// write just the status byte
    wireWriteByte(0x00);
    wireWriteByte(status);
    crc1=wireReadByte();
    crc2=wireReadByte();
#if DEBUG_WIRESEARCH
    Serial.print("deviceWriteStatus: crc=");
    Serial.print(crc1,HEX);
    Serial.println(crc2,HEX);
#endif
    ok = true;			// calc crc and compare to set ok
    if(ok) wireWriteByte(0xFF);
    else wireReset();
}

// read channel info byte into buf
// device = ds2406
// Family:12
// 2406
// Channel Info:33

uint8_t DS2482::deviceReadChannel(uint8_t *ROM, uint8_t *buf)
{
    uint8_t channel_control = 0x44;	// 0 1 0 0 0 1 0 0
    unsigned char status;
    int i;
    wireReset();
    wireSelect(ROM);
    busyWait(true);
    wireWriteByte(0xF5);
    wireWriteByte(channel_control);
    wireWriteByte(0xFF);
    *buf++ = wireReadByte();
}
#endif
