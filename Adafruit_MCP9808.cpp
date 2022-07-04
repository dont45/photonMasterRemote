/**************************************************************************/
/*!
    @file     Adafruit_MCP9808.cpp
    @author   K.Townsend (Adafruit Industries)
	@license  BSD (see license.txt)

	I2C Driver for Microchip's MCP9808 I2C Temp sensor

  This library has been adapted for use with Particle Photon by
  Don Thompson, 02/01/2016

	This is a library for the Adafruit MCP9808 breakout
	----> http://www.adafruit.com/products/1782

	Adafruit invests time and resources providing this open source code,
	please support Adafruit and open-source hardware by purchasing
	products from Adafruit!

	@section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/

#include "Adafruit_MCP9808.h"

/**************************************************************************/
/*!
    @brief  Instantiates a new MCP9808 class
*/
/**************************************************************************/
Adafruit_MCP9808::Adafruit_MCP9808() {
}

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
uint16_t Adafruit_MCP9808::begin(uint8_t addr) {
  _i2caddr = addr;
  Wire.begin();
  #ifdef SERIAL_DEBUG_I2C
  Serial.println("Wire.begin() ok");
  #endif
  uint16_t rval;
  rval = read16(MCP9808_REG_MANUF_ID);
  if (rval != 0x0054) {
#ifdef SERIAL_DEBUG_I2C
    Serial.print("REG_MANUF_ID:");
    Serial.println(rval);
#endif
    return rval;
  }
  rval = read16(MCP9808_REG_DEVICE_ID);
  if (rval != 0x0400) {
#ifdef SERIAL_DEBUG_I2C
    Serial.print("DEVICE_ID:");
    Serial.println(rval);
#endif
    return rval;
  }
#ifdef SERIAL_DEBUG_I2C
  Serial.println("MCP9808 REG SPECS PASSED");
#endif
  return 0;
}

/**************************************************************************/
/*!
    @brief  Reads the 16-bit temperature register and returns the Centigrade
            temperature as a float.

*/
/**************************************************************************/
float Adafruit_MCP9808::readTempC( void )
{
  uint16_t t = read16(MCP9808_REG_AMBIENT_TEMP);
  #ifdef SERIAL_DEBUG_I2C
    Serial.print("mcp9808.readTempC=");
    Serial.println(t);
  #endif
  float temp = t & 0x0FFF;
  temp /=  16.0;
  if (t & 0x1000) temp -= 256;

  return temp;
}

/**************************************************************************/
/*!
    @brief  Reads the 16-bit temperature register and returns the Fahrenheit
            temperature as a float.
*/
/**************************************************************************/
float Adafruit_MCP9808::readTempF( void )
{
  float tempC = readTempC();
  return (tempC * 9.0 / 5.0) + 32.0;
}

//*************************************************************************
// Set Sensor to Shutdown-State or wake up (Conf_Register BIT8)
// 1= shutdown / 0= wake up
//*************************************************************************

int Adafruit_MCP9808::shutdown_wake( uint8_t sw_ID )
{
    uint16_t conf_shutdown ;
    uint16_t conf_register = read16(MCP9808_REG_CONFIG);
    if (sw_ID == 1)
    {
       conf_shutdown = conf_register | MCP9808_REG_CONFIG_SHUTDOWN ;
       write16(MCP9808_REG_CONFIG, conf_shutdown);
    }
    if (sw_ID == 0)
    {
       conf_shutdown = conf_register ^ MCP9808_REG_CONFIG_SHUTDOWN ;
       write16(MCP9808_REG_CONFIG, conf_shutdown);
    }


    return 0;
}




/**************************************************************************/
/*!
    @brief  Low level 16 bit read and write procedures!
*/
/**************************************************************************/

void Adafruit_MCP9808::write16(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(_i2caddr);
    Wire.write((uint8_t)reg);
    Wire.write(value >> 8);
    Wire.write(value & 0xFF);
    Wire.endTransmission();
}

uint16_t Adafruit_MCP9808::read16(uint8_t reg) {
  uint16_t val;
  #ifdef SERIAL_DEBUG_I2C
    Serial.println("mcp9808.read16");
  #endif
  Wire.beginTransmission(_i2caddr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();

  Wire.requestFrom((uint8_t)_i2caddr, (uint8_t)2);
  val = Wire.read();
 #ifdef SERIAL_DEBUG_I2C
    Serial.print("val1:");
    Serial.print(val);
  #endif
   val <<= 8;
  val |= Wire.read();
  #ifdef SERIAL_DEBUG_I2C
    Serial.print(" val2:");
    Serial.println(val);
  #endif
  return val;
}
