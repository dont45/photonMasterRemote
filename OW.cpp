/*
  OW (1-WIRE) library for Arduino
  Adapted to Particle Photon
  2009 raynham engineering

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

//#include "WConstants.h"
#include "OW.h"

// read status block (8 bytes + crc) into buf
// device = ds2406
// SENSOR:0:12D8046000E9
// Family:12
// 2406
// deviceReadStatus:10
// Status=FF:FF:FF:FF:FF:0:0:7F:ED:C1:
// ok!!

uint8_t OW::readStatus(uint8_t *ROM, uint8_t *buf)
{
//    unsigned char status;
    int i;
    wireReset();
    wireSelect(ROM);
    busyWait(true);
    wireWriteByte(0xAA);
    wireWriteByte(0x00);
    wireWriteByte(0x00);
    for(i=0; i<10; i++)
        *buf++ = wireReadByte();
    return 1;
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

// status byte (7)
// |7        |6     |5     |4      ||3      |2      |1      |0        |
// |Supply   |PIO-B |PIO-A |CSS4   ||CSS3   |CSS2   |CSS1   |CSS0     |
// |Ind.     |Chan  |Chan  |Chan   ||Chan   |Source |Source |Polarity |
// |read-only| F-F  | F-F  |Select ||Select |Select |Select |         |

uint8_t OW::writeStatus(uint8_t *ROM, uint8_t status)
{
//   uint8_t crc1, crc2;
    bool ok;
    // should verify family here ??
    wireReset();
    wireSelect(ROM);
    busyWait(true);
    wireWriteByte(0x55);
    wireWriteByte(0x07);	// write just the status byte
    wireWriteByte(0x00);
    wireWriteByte(status);
//    crc1=wireReadByte();
//    crc2=wireReadByte();
//    serial->print("writeStatus: crc=");
//    serial->print(crc1,HEX);
//    serial->println(crc2,HEX);
    ok = true;			// calc crc and compare to set ok
    if(ok) wireWriteByte(0xFF);
    else wireReset();
    return ok;      // ?? this return was missing??
}

// This should be called channelAccess
uint8_t OW::channelAccess(uint8_t *ROM, uint8_t channel_control, uint8_t *buf)
{
    return true;
}

// read channel info byte into buf
// device = ds2406
// Family:12
// 2406
// Channel Info:33

// Need to pass channel_control as a parameter??
// No,  Need to develope a general channelInfo routine,
// call it from readChannel and writeChannel ??
// This version reads Ch B Only
uint8_t OW::readChannel(uint8_t *ROM, uint8_t *buf)
{
    uint8_t channel_control = 0x48;	// 0 1 0 0 1 0 0 0
    wireReset();
  #ifdef SERIAL_DEBUG_OW
    Serial.print("OW::readChannel: ");
    for(int k=0;k<8;k++) {
      Serial.print(ROM[k],HEX);
      Serial.print(":");
    }
  #endif
    wireSelect(ROM);
    busyWait(true);
    wireWriteByte(0xF5);
    wireWriteByte(channel_control);
  #ifdef SERIAL_DEBUG_OW
    Serial.print(" ch_control:");
    Serial.print(channel_control, BIN);
  #endif
    wireWriteByte(0xFF);
    *buf = wireReadByte();
  #ifdef SERIAL_DEBUG_OW
    Serial.print(" ch_info:");
    Serial.println(*buf, BIN);
  #endif
    return *buf;
}

//ds1820 class thermometer
bool OW::readThermometer(uint8_t *ROM, double &rTempF) {
  uint8_t present = 0;
  uint8_t data[12];
  uint8_t type_s;
  int raw;
  double celsius;
  #ifdef SERIAL_DEBUG_THERM
  Serial.print("readThermometer-ROM:");
  Serial.println(ROM[1], HEX);
  #endif
  switch (ROM[0]) {
    case 0x10:
      //Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      //Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      //Serial.println("  Chip = DS1822");
      type_s = 0;
    default:
      //Serial.println("Device is not a DS18x20 family device.");
      return FALSE;
  }
  wireReset();
  wireSelect(ROM);
  wireWriteByte(0x44);     //start conversion
#ifdef DS1820_POWERED
// if NOT parasetic powered, read bits until
// 1 is returned (reads 0 until conversion complete)
#else
  delay(400);  //maybe longer ??
#endif
  present = wireReset();  //reset ok because scratcpad written
  if(present) {
    wireSelect(ROM);
    wireWriteByte(0xbe);   //read scratchpad
    #ifdef SERIAL_DEBUG_THERM
    Serial.print("Scratchpad: ");
    #endif
    for(int i=0; i<9; i++) {
      data[i] = wireReadByte();
      #ifdef SERIAL_DEBUG_THERM
      Serial.print(data[i],HEX);
      Serial.print(":");
      #endif
    }
    #ifdef SERIAL_DEBUG_THERM
    Serial.println();
    #endif
    if(DS2482::crc8(data,8) != data[8]) {
      #ifdef SERIAL_DEBUG_THERM
      Serial.println("CRC is not valid");
      #endif
      return FALSE;
    }
    raw = (data[1]<<8) | data[0];   //put two bytes of temp into raw
    if (type_s) {
      raw = raw << 3; // 9 bit resolution default
      if (data[7] == 0x10) {
        // "count remain" gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - data[6];
      }
    } else {
      uint8_t cfg = (data[4] & 0x60);
      // at lower res, the low bits are undefined, so let's zero them
      if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
      //// default is 12 bit resolution, 750 ms conversion time
    }
    celsius = (double)raw / 16.0;
    rTempF = celsius * 1.8 + 32;    //convert to Fahrenheit
    #ifdef SERIAL_DEBUG_THERM
    Serial.println("convert tempC=");
    Serial.println(celsius);
    #endif
    return TRUE;
  }
  return FALSE;
}

// write PIO and/or PIO-B to ds2406 class device
uint8_t OW::writePIOtest(uint8_t *ROM, uint8_t port, uint8_t val)
{
/*ds2414
port = 0 ==> PIO-A, port = 1 ==> PIO-B
PIO Output Data Byte
bit assignment
|  b7   |  b6   |  b5   |  b4   |  b3   |  b2   |  b1   |  b0   |
|   X   |   X   |   X   |   X   |   X   |   X   |  PIOB |  PIOA |
  X == 1, PIO val == 0 ==> transistor switched on
  val = A ==1 (0b000001), B == 2 (0b00000010)
  inverted = (-b11111110),or (0x11111101)
  SIMPLE:
  1) write ~port (11111110)
  2) write port (inverted)
*/
    uint8_t port_data;
    uint8_t conf_data;
#ifdef SERIAL_DEBUG_OW
    Serial.print("writePIOtest: port=");
    Serial.print(port, BIN);
    Serial.print(" val=");
    Serial.println(val, BIN);
#endif
    switch(ROM[0]) {
      case FMLY_2413:
      port_data =  0xff & (~(val << port));
#ifdef SERIAL_DEBUG_OW
      Serial.print("Output Data: ");
      Serial.print(port_data, HEX);
#endif
      // write this to wire
      // and then write the complement
      break;
      default:
#ifdef SERIAL_DEBUG_OW
        Serial.println("FMLY not supported");
#endif
        return 0;
    }
    wireReset();
    wireSelect(ROM);
    busyWait(true);
    wireWriteByte(0x5a);
    wireWriteByte(port_data);
    wireWriteByte(~port_data);
    conf_data=wireReadByte(); //confirmation byte should read 0xAA
#ifdef SERIAL_DEBUG_OW
    Serial.print(" conf byte=");
    Serial.print(conf_data, HEX);
#endif
    if(conf_data != 0xAA) {
#ifdef SERIAL_DEBUG_OW
      Serial.print("DS2413 write not confirmed:");
      Serial.println(conf_data, HEX);
#endif
      return 0;
    }
    conf_data=wireReadByte(); //confirmation port data
#ifdef SERIAL_DEBUG_OW
    Serial.print(" port=");
    Serial.println(conf_data, HEX);
#endif
    wireReset();
    return conf_data;
}
// write PIO-A and/or PIO-B to ds2406 class device
uint8_t OW::writePIO(uint8_t *ROM, uint8_t port, uint8_t val)
{
//  using write to status memory
//   ??? THIS NEEDS FMLY WORK ???  ??port use ???
    status_memory_7 = 0x1f;		// shut off bits 5 & 6   (0001 1111)
    status_memory_7 |= val << 5;	// set bits 5 & 6        (0xx0 0000)
    wireReset();
    wireSelect(ROM);
    busyWait(true);
    wireWriteByte(0x55);
    wireWriteByte(0x07);
    wireWriteByte(0x00);
    wireWriteByte(status_memory_7);
    wireReadByte();
    return 1;
}

// read PIO port=d.port state by family
uint8_t OW::readPIOX(uint8_t *ROM, uint8_t port) {
  uint8_t pio;
  uint8_t sense_bit;
  pio = readPIO(ROM);
  #ifdef SERIAL_DEBUGXX
    Serial.print("::readPIOX..pio=");
    Serial.println(pio,BIN);
    Serial.print(" dev=");
    Serial.print(ROM[0],HEX);
    Serial.print(" port=");
    Serial.print(port);
  #endif
  switch(ROM[0])
  {
    case FMLY_2406:
          sense_bit = 1 << sense_port_shift[0][port];
        #ifdef SERIAL_DEBUGXX
          Serial.print(" sense_bit=");
          Serial.println(sense_bit,BIN);
        #endif
          break;
    case FMLY_2413:
          sense_bit = 1 << sense_port_shift[1][port];
        #ifdef SERIAL_DEBUGXX
          Serial.print(" sense_bit=");
          Serial.println(sense_bit,BIN);
        #endif
          break;
    default:
        #ifdef SERIAL_DEBUGXX
          Serial.print(" NO FMLY=");
          Serial.println(ROM[0],HEX);
          #endif
          return 0;
  }
  return (pio & sense_bit) ? 1 : 0;
}

// read PIOA state by family
uint8_t OW::readPIOA(uint8_t *ROM) {
  uint8_t pio;
  pio = readPIO(ROM);
  switch(ROM[0])
  {
    case FMLY_2406:
          return (pio & DS2406_PIOA_SENSE) ? 1 : 0;
    break;
    case FMLY_2413:
          return (pio & DS2413_PIOA_SENSE) ? 1 : 0;
    break;
    default:
          return 0;
  }
}
// read PIOB state by family
uint8_t OW::readPIOB(uint8_t *ROM) {
  uint8_t pio;
  pio = readPIO(ROM);
  switch(ROM[0])
  {
    case FMLY_2406:
          return (pio & DS2406_PIOB_SENSE) ? 1 : 0;
    break;
    case FMLY_2413:
          return (pio & DS2413_PIOB_SENSE) ? 1 : 0;
    break;
    default:
          return 0;
  }
};

// read PIO state (switch family)
uint8_t OW::readPIO(uint8_t *ROM)
{
    uint8_t pio;
    switch(ROM[0])
    {
  	   case FMLY_2405:
		      uint8_t r1, r2;
		      readChannel(ROM, &r1);
		      readChannel(ROM, &r2);
		      return r1 == r2;
		      break;
    case FMLY_2413:
  	case FMLY_2406:
		      readChannel(ROM, &pio);
		      return pio;
	default:
#ifdef SERIAL_DEBUG_OW
		      Serial.println("readPIO FMLY UNSUPPORTED");
#endif
		      return 0xFF;
    }
}

uint8_t OW::condSearch(uint8_t *newAddr)
{
    uint8_t search_status;
    // set devices for conditional search
    // this MUST be done external to this and prior to calling this
    // set for conditional search
    setCondSearch();
    search_status = wireSearch(newAddr);
    // restore normal search
    setStdSearch();
    return search_status;
}
