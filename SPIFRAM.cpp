/* Arduino SPIFRAM Library v 0.0.1b
 * Copyright (C) 2015 by Prajwal Bhattaram
 * Modified by Prajwal Bhattaram - 14/11/2016
 *
 * This file is part of the Arduino SPIFRAM Library. This library is for
 * Winbond NOR flash memory modules. In its current form it enables reading
 * and writing individual data variables, structs and arrays from and to various locations;
 * reading and writing pages; continuous read functions; sector, block and chip erase;
 * suspending and resuming programming/erase and powering down for low power operation.
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License v3.0
 * along with the Arduino SPIFRAM Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "SPIFRAM.h"

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//     Uncomment the code below to run a diagnostic if your flash 	  //
//                         does not respond                           //
//                                                                    //
//      Error codes will be generated and returned on functions       //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//#define RUNDIAGNOSTIC                                               //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//   Uncomment the code below to increase the speed of the library    //
//                  by disabling _notPrevWritten()                    //
//                                                                    //
// Make sure the sectors being written to have been erased beforehand //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//#define HIGHSPEED                                                   //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

// Constructor

SPIFRAM::SPIFRAM(uint8_t cs, bool overflow) {
  csPin = cs;
#if defined (ARDUINO_ARCH_AVR)
#ifndef __AVR_ATtiny85__
  cs_port = portOutputRegister(digitalPinToPort(csPin));
#endif
  cs_mask = digitalPinToBitMask(csPin);
#endif
  pageOverflow = overflow;
  pinMode(csPin, OUTPUT);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//     Private functions used by read, write and erase operations     //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

//Double checks all parameters before calling a read or write.
//Takes address and returns the address if true, else returns false. Throws an error if there is a problem.
bool SPIFRAM::_prep(uint8_t opcode, uint32_t address, uint32_t size) {
  switch (opcode) {
    case PAGEPROG:
    if (!_addressCheck(address, size)) {
      return false;
    }
    if(!_writeEnable()){
      return false;
    }
    #ifndef HIGHSPEED
    if(!_notPrevWritten(address, size)) {
      return false;
    }
    #endif
    return true;
    break;

    default:
    if (!_addressCheck(address, size)) {
      return false;
    }
    return true;
    break;
  }
}

bool SPIFRAM::_transferAddress(void) {
  _nextByte(_currentAddress >> 16);
  _nextByte(_currentAddress >> 8);
  _nextByte(_currentAddress);
}

bool SPIFRAM::_startSPIBus(void) {
#ifndef SPI_HAS_TRANSACTION
    noInterrupts();
#endif

#if defined (ARDUINO_ARCH_SAM)
  _dueSPIInit(DUE_SPI_CLK);
#else
  #if defined (ARDUINO_ARCH_AVR)
    //save current SPI settings
      _SPCR = SPCR;
      _SPSR = SPSR;
  #endif
  #ifdef SPI_HAS_TRANSACTION
    SPI.beginTransaction(_settings);
  #else
    SPI.setClockDivider(SPI_CLOCK_DIV_4)
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    #endif
#endif
  SPIBusState = true;
}

//Initiates SPI operation - but data is not transferred yet. Always call _prep() before this function (especially when it involves writing or reading to/from an address)
bool SPIFRAM::_beginSPI(uint8_t opcode) {
  if (!SPIBusState) {
    //Serial.println("Starting SPI Bus");
    _startSPIBus();
  }
  CHIP_SELECT
  switch (opcode) {
    case READDATA:
    _nextByte(opcode);
    _transferAddress();
    break;

    case PAGEPROG:
    _nextByte(opcode);
    _transferAddress();
    break;

    default:
    _nextByte(opcode);
    break;
  }
  return true;
}
//SPI data lines are left open until _endSPI() is called

//Reads/Writes next byte. Call 'n' times to read/write 'n' number of bytes. Should be called after _beginSPI()
uint8_t SPIFRAM::_nextByte(uint8_t data) {
#if defined (ARDUINO_ARCH_SAM)
  return _dueSPITransfer(data);
#else
  return xfer(data);
#endif
}

//Reads/Writes next int. Call 'n' times to read/write 'n' number of bytes. Should be called after _beginSPI()
uint16_t SPIFRAM::_nextInt(uint16_t data) {
  //return xfer16(data);
  return SPI.transfer16(data);
}

//Reads/Writes next data buffer. Call 'n' times to read/write 'n' number of bytes. Should be called after _beginSPI()
void SPIFRAM::_nextBuf(uint8_t opcode, uint8_t *data_buffer, uint32_t size) {
  uint8_t *_dataAddr = &(*data_buffer);
  switch (opcode) {
    case READDATA:
    #if defined (ARDUINO_ARCH_SAM)
      _dueSPIRecByte(&(*data_buffer), size);
    #elif defined (ARDUINO_ARCH_AVR)
      SPI.transfer(&data_buffer[0], size);
    #else
      for (uint16_t i = 0; i < size; i++) {
        *_dataAddr = xfer(NULLBYTE);
        _dataAddr++;
      }
      #endif
    break;

    case PAGEPROG:
    #if defined (ARDUINO_ARCH_SAM)
      _dueSPISendByte(&(*data_buffer), size);
    #elif defined (ARDUINO_ARCH_AVR)
      SPI.transfer(&(*data_buffer), size);
    #else
      for (uint16_t i = 0; i < size; i++) {
        xfer(*_dataAddr);
        _dataAddr++;
      }
    #endif
    break;
  }
}

//Stops all operations. Should be called after all the required data is read/written from repeated _nextByte() calls
void SPIFRAM::_endSPI(void) {
  CHIP_DESELECT
  #ifdef SPI_HAS_TRANSACTION
  SPI.endTransaction();
  #else
  interrupts();
  #endif

  #if defined (ARDUINO_ARCH_AVR)
  SPCR = _SPCR;
  SPSR = _SPSR;
  #endif
  SPIBusState = false;
}

// Checks if status register 1 can be accessed - used during powerdown and power up and for debugging
uint8_t SPIFRAM::_readStat1(void) {
	_beginSPI(READSTAT1);
  uint8_t stat1 = _nextByte();
  //_endSPI();
  CHIP_DESELECT
	return stat1;
}

//Enables writing to chip by setting the WRITEENABLE bit
bool SPIFRAM::_writeEnable(uint32_t timeout) {
  uint32_t startTime = millis();
  if (!(state & WRTEN)) {
    do {
      _beginSPI(WRITEENABLE);
      //_endSPI();
      CHIP_DESELECT
      state = _readStat1();
      if((millis()-startTime) > timeout) {
        errorcode = CANTENWRITE;
        #ifdef RUNDIAGNOSTIC
        _troubleshoot();
        #endif
        return false;
       }
     } while (!(state & WRTEN));
  }
  return true;
}

//Disables writing to chip by setting the Write Enable Latch (WEL) bit in the Status Register to 0
//_writeDisable() is not required under the following conditions because the Write Enable Latch (WEL) flag is cleared to 0
// i.e. to write disable state:
// Power-up, Write Disable, Page Program, Quad Page Program, Sector Erase, Block Erase, Chip Erase, Write Status Register,
// Erase Security Register and Program Security register
bool SPIFRAM::_writeDisable(void) {
	_beginSPI(WRITEDISABLE);
  _endSPI();
	return true;
}

//Checks for presence of chip by requesting JEDEC ID
bool SPIFRAM::_getJedecId(uint8_t *b1, uint8_t *b2, uint8_t *b3, uint8_t *b4) {
  _beginSPI(JEDECID);
  *b1 = _nextByte(NULLBYTE);		// manufacturer id
  *b2 = _nextByte(NULLBYTE);		// continuation code
  *b3 = _nextByte(NULLBYTE);		// capacity
  *b4 = _nextByte(NULLBYTE);		// product id
  _endSPI();
  return true;
}

//Identifies the chip
bool SPIFRAM::_chipID(void) {
#ifdef CHIPSIZE
  // If a custom chip size is defined
  capacity = CHIPSIZE/8;
  return true;
#else
  // If no custom chip size declared, ID the chip
  //Get Manfucturer/Device ID so the library can identify the chip
  uint8_t manID, contID, capID, devID ;
  _getJedecId(&manID, &contID, &capID, &devID);

  if (manID != FUJITSU_MANID) {		//If the chip is not a Winbond Chip
    errorcode = UNKNOWNCHIP;		//Error code for unidentified chip
    #ifdef RUNDIAGNOSTIC
    _troubleshoot();
    #endif
    while(1);
  }
  //Check flash memory type and identify capacity
  //capacity & chip name
  for (uint8_t i = 0; i < sizeof(devType); i++)
  {
    if (devID == devType[i]) {
      capacity = (memSize[i])/8;
    }
  }
  if (capacity == 0) {
    errorcode = UNKNOWNCAP;		//Error code for unidentified capacity
    #ifdef RUNDIAGNOSTIC
    _troubleshoot();
    #endif
    while(1);
  }
  return true;
#endif
}

//Checks to see if pageOverflow is permitted and assists with determining next address to read/write.
//Sets the global address variable
bool SPIFRAM::_addressCheck(uint32_t address, uint32_t size) {
	if (capacity == 0) {
    errorcode = CALLBEGIN;
    #ifdef RUNDIAGNOSTIC
    _troubleshoot();
    #endif
	}

  for (uint32_t i = 0; i < size; i++) {
    if (address + i >= maxAddress) {
    	if (!pageOverflow) {
        errorcode = OUTOFBOUNDS;
        #ifdef RUNDIAGNOSTIC
        _troubleshoot();
        #endif
        return false;					// At end of memory - (!pageOverflow)
      }
      else {
        _currentAddress = 0x00;
        return true;					// At end of memory - (pageOverflow)
      }
    }
  }
  _currentAddress = address;
  return true;				// Not at end of memory if (address < capacity)
}

bool SPIFRAM::_notPrevWritten(uint32_t address, uint32_t size) {
  _beginSPI(READDATA);
  for (uint16_t i = 0; i < size; i++) {
    if (_nextByte() != EMPTYCELL) {
      _endSPI();
      return false;
    }
  }
  //_endSPI();
  CHIP_DESELECT
  return true;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//     Public functions used for read, write and erase operations     //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

//Identifies chip and establishes parameters
void SPIFRAM::begin(void) {
#if defined (ARDUINO_ARCH_SAM)
  _dueSPIBegin();
#else
  SPI.begin();
#endif

#ifdef SPI_HAS_TRANSACTION
  //Define the settings to be used by the SPI bus
  _settings = SPISettings(SPI_CLK, MSBFIRST, SPI_MODE0);
#endif
  _chipID();
}

//Allows the setting of a custom clock speed for the SPI bus to communicate with the chip.
//Only works if the SPI library in use supports SPI Transactions
#ifdef SPI_HAS_TRANSACTION
void SPIFRAM::setClock(uint32_t clockSpeed) {
  _settings = SPISettings(clockSpeed, MSBFIRST, SPI_MODE0);
}
#endif

uint8_t SPIFRAM::error(void) {
	return errorcode;
}

//Returns capacity of chip
uint32_t SPIFRAM::getCapacity(void) {
	return capacity;
}

//Returns the library version as a string
bool SPIFRAM::libver(uint8_t *b1, uint8_t *b2, uint8_t *b3) {
  *b1 = LIBVER;
  *b2 = LIBSUBVER;
  *b3 = BUGFIXVER;
  return true;
}

//Checks for and initiates the chip by requesting JEDEC ID which is returned as a 32 bit int
uint32_t SPIFRAM::getJEDECID(void) {
	uint8_t b1, b2, b3, b4;
    _getJedecId(&b1, &b2, &b3, &b4);
    uint32_t id = b1;
    id = (id << 8)|(b2 << 0);
    id = (id << 8)|(b3 << 0);
    id = (id << 8)|(b4 << 0);
    return id;
}

// Gets the next available address for use.
// Takes the size of the data as an argument and returns a 32-bit address
// All addresses in the in the sketch must be obtained via this function or not at all.
uint32_t SPIFRAM::getAddress(uint16_t size) {
	if (!_addressCheck(currentAddress, size)){
    errorcode = OUTOFBOUNDS;
    #ifdef RUNDIAGNOSTIC
    _troubleshoot();
    #endif
    return false;
	}
	else {
		uint32_t address = currentAddress;
		/*Serial.print("Current Address: ");
		Serial.println(currentAddress);*/
		currentAddress+=size;
		return address;
	}
}

//Function for returning the size of the string (only to be used for the getAddress() function)
uint16_t SPIFRAM::sizeofStr(String &inputStr) {
	uint16_t size;
  size = (sizeof(char)*(inputStr.length()+1));
	size+=sizeof(inputStr.length()+1);

	return size;
}

// Reads a byte of data from a specific address.
// Takes one argument -
//		1. address --> Any address from 0 to maxAddress
uint8_t SPIFRAM::readByte(uint32_t address) {
  uint8_t data;

	if (!_prep(READDATA, address, sizeof(data))) {
		return false;
  }
  _beginSPI(READDATA);
  data = _nextByte();
  _endSPI();
  return data;
}

// Reads a char of data from a specific address.
// Takes one argument -
//		1. address --> Any address from 0 to maxAddress
int8_t SPIFRAM::readChar(uint32_t address) {
	int8_t data;
	if (!_prep(READDATA, address, sizeof(data))) {
		return false;
  }
  _beginSPI(READDATA);
  data = _nextByte();
  _endSPI();
  return data;
}

// Reads an array of bytes starting from a specific address.
// Takes two arguments
//		1. address --> Any address from 0 to maxAddress
//		2. data_buffer --> The array of bytes to be read from the flash memory - starting at the address indicated
bool  SPIFRAM::readByteArray(uint32_t address, uint8_t *data_buffer, uint16_t bufferSize) {
	if (!_prep(READDATA, address, bufferSize)) {
    return false;
	}
  _beginSPI(READDATA);
  _nextBuf(READDATA, &(*data_buffer), bufferSize);
  _endSPI();
	return true;
}

// Reads an array of chars starting from a specific address.
// Takes two arguments
//		1. address --> Any address from 0 to maxAddress
//		2. data_buffer --> The array of bytes to be read from the flash memory - starting at the address indicated
bool  SPIFRAM::readCharArray(uint32_t address, char *data_buffer, uint16_t bufferSize) {
  if (!_prep(READDATA, address, bufferSize)) {
    return false;
	}
  _beginSPI(READDATA);
  _nextBuf(READDATA, (uint8_t*) &(*data_buffer), bufferSize);
  _endSPI();
	return true;
}

// Reads an unsigned int of data from a specific address.
// Takes one argument -
//		1. address --> Any address from 0 to maxAddress
uint16_t SPIFRAM::readWord(uint32_t address) {
  const uint8_t size = sizeof(uint16_t);
	union
	{
		uint8_t b[size];
		uint16_t I;
	} data;
	if (!_prep(READDATA, address, size)) {
		return false;
  }
  _beginSPI(READDATA);
  _nextBuf(READDATA, &data.b[0], size);
  _endSPI();
  return data.I;
}

// Reads a signed int of data from a specific address.
// Takes one argument -
//		1. address --> Any address from 0 to maxAddress
int16_t SPIFRAM::readShort(uint32_t address) {
  const uint8_t size = sizeof(int16_t);
	union
	{
		byte b[size];
		int16_t s;
	} data;

	if (!_prep(READDATA, address, size)) {
    return false;
  }
  _beginSPI(READDATA);
  _nextBuf(READDATA, &data.b[0], size);
  _endSPI();
  return data.s;
}

// Reads an unsigned long of data from a specific address.
// Takes one argument -
//		1. address --> Any address from 0 to maxAddress
uint32_t SPIFRAM::readULong(uint32_t address) {
  const uint8_t size = (sizeof(uint32_t));
	union
	{
		uint8_t b[size];
		uint32_t l;
	} data;

	if (!_prep(READDATA, address, size)) {
    return false;
  }
  _beginSPI(READDATA);
  _nextBuf(READDATA, &data.b[0], size);
  _endSPI();
  return data.l;
}

// Reads a signed long of data from a specific address.
// Takes one argument -
//		1. address --> Any address from 0 to maxAddress
int32_t SPIFRAM::readLong(uint32_t address) {
  const uint8_t size = (sizeof(int32_t));
	union
	{
		byte b[size];
		int32_t l;
	} data;

	if (!_prep(READDATA, address, size)) {
    return false;
  }
  _beginSPI(READDATA);
  _nextBuf(READDATA, &data.b[0], size);
  _endSPI();
  return data.l;
}

// Reads a signed long of data from a specific address.
// Takes one argument -
//		1. address --> Any address from 0 to maxAddress
float SPIFRAM::readFloat(uint32_t address) {
  const uint8_t size = (sizeof(float));
	union
	{
		byte b[size];
		float f;
	} data;

	if (!_prep(READDATA, address, size)) {
    return false;
  }
  _beginSPI(READDATA);
  _nextBuf(READDATA, &data.b[0], size);
  _endSPI();
  return data.f;
}

// Reads a string from a specific location on a page.
// Takes two arguments
//		1. address --> Any address from 0 to maxAddress
//		2. outputString --> String variable to write the output to
bool SPIFRAM::readStr(uint32_t address, String &outStr) {
  uint16_t strLen;
  strLen = readWord(address);
  address+=(sizeof(strLen));
  char outputChar[strLen];

  readCharArray(address, outputChar, strLen);

  outStr = String(outputChar);
  return true;
}

// Writes a byte of data to a specific address.
// Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> One byte of data to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors
bool SPIFRAM::writeByte(uint32_t address, uint8_t data, bool errorCheck) {
  if(!_prep(PAGEPROG, address, sizeof(data))) {
    return false;
  }

  _beginSPI(PAGEPROG);
  _nextByte(data);
  CHIP_DESELECT

		if (!errorCheck) {
      _endSPI();
      return true;
  }
	else {
		return _writeErrorCheck(address, data);
  }
}

// Writes a char of data to a specific address.
// Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> One char of data to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors
bool SPIFRAM::writeChar(uint32_t address, int8_t data, bool errorCheck) {
  if(!_prep(PAGEPROG, address, sizeof(data))) {
    return false;
  }

  _beginSPI(PAGEPROG);
  _nextByte(data);
  CHIP_DESELECT

		if (!errorCheck) {
      _endSPI();
      return true;
  }
	else {
		return _writeErrorCheck(address, data);
  }
}

// Writes an array of bytes starting from a specific address.

// Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> An array of bytes to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors
bool SPIFRAM::writeByteArray(uint32_t address, uint8_t *data_buffer, uint16_t bufferSize, bool errorCheck) {
  if (!_prep(PAGEPROG, address, bufferSize)) {
    return false;
  }

  _beginSPI(PAGEPROG);
  _nextBuf(PAGEPROG, &data_buffer[0], bufferSize);
  CHIP_DESELECT

  /*_beginSPI(PAGEPROG);
  for (uint16_t i = 0; i < bufferSize; ++i) {
    _nextByte(data_buffer[data_offset + i]);
  }
  CHIP_DESELECT*/

  if (!errorCheck) {
    _endSPI();
    return true;
  }
  else {
    _currentAddress = address;
    CHIP_SELECT
    _nextByte(READDATA);
    _transferAddress();
    for (uint16_t j = 0; j < bufferSize; j++) {
      if (_nextByte(NULLBYTE) != data_buffer[j]) {
        return false;
      }
    }
    _endSPI();
    return true;
  }
}

// Writes an array of bytes starting from a specific address.
// Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> An array of chars to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors
bool SPIFRAM::writeCharArray(uint32_t address, char *data_buffer, uint16_t bufferSize, bool errorCheck) {
  if (!_prep(PAGEPROG, address, bufferSize)) {
    return false;
  }

  _beginSPI(PAGEPROG);
  _nextBuf(PAGEPROG, (uint8_t*) &data_buffer[0], bufferSize);
  CHIP_DESELECT

  /*_beginSPI(PAGEPROG);
  for (uint16_t i = 0; i < writeBufSz; ++i) {
    _nextByte(data_buffer[data_offset + i]);
  }
  CHIP_DESELECT*/

  if (!errorCheck) {
    _endSPI();
    return true;
  }
  else {
    _currentAddress = address;
    CHIP_SELECT
    _nextByte(READDATA);
    _transferAddress();
    for (uint16_t j = 0; j < bufferSize; j++) {
      if (_nextByte(NULLBYTE) != data_buffer[j]) {
        return false;
      }
    }
    _endSPI();
    return true;
  }
}

// Writes an unsigned int as two bytes starting from a specific address.
// Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> One unsigned int of data to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors
bool SPIFRAM::writeWord(uint32_t address, uint16_t data, bool errorCheck) {
  const uint8_t size = sizeof(uint16_t);

	if(!_prep(PAGEPROG, address, size)) {
    return false;
  }

	union
	{
		uint8_t b[size];
		uint16_t w;
	} var;
	var.w = data;

  _beginSPI(PAGEPROG);
  _nextBuf(PAGEPROG, &var.b[0], size);
  CHIP_DESELECT

		if (!errorCheck) {
      _endSPI();
      return true;
  }
	else
		return _writeErrorCheck(address, data);
}

// Writes a signed int as two bytes starting from a specific address.
// Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> One signed int of data to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors
bool SPIFRAM::writeShort(uint32_t address, int16_t data, bool errorCheck) {
  const uint8_t size = sizeof(data);
	if(!_prep(PAGEPROG, address, size)) {
    return false;
  }

	union
	{
		uint8_t b[size];
		int16_t s;
	} var;
	var.s = data;

  _beginSPI(PAGEPROG);
  _nextBuf(PAGEPROG, &var.b[0], size);
  CHIP_DESELECT

	if (!errorCheck) {
    _endSPI();
    return true;
  }
	else
		return _writeErrorCheck(address, data);
}

// Writes an unsigned long as four bytes starting from a specific address.
// Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> One unsigned long of data to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors
bool SPIFRAM::writeULong(uint32_t address, uint32_t data, bool errorCheck) {
  const uint8_t size = (sizeof(data));

	if(!_prep(PAGEPROG, address, size)) {
    return false;
  }

	union
	{
		uint8_t b[size];
		uint32_t l;
	} var;
  var.l = data;

  _beginSPI(PAGEPROG);
  _nextBuf(PAGEPROG, &var.b[0], size);
  CHIP_DESELECT

	if (!errorCheck){
    _endSPI();
		return true;
  }
	else {
		return _writeErrorCheck(address, data);
  }
}

// Writes a signed long as four bytes starting from a specific address.
// Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> One signed long of data to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors

bool SPIFRAM::writeLong(uint32_t address, int32_t data, bool errorCheck) {
const uint8_t size = sizeof(data);

	if(!_prep(PAGEPROG, address, size)) {
    return false;
  }

  union
  {
    uint8_t b[size];
    int32_t l;
  } var;
  var.l = data;

  _beginSPI(PAGEPROG);
  _nextBuf(PAGEPROG, &var.b[0], size);
  CHIP_DESELECT

	if (!errorCheck){
    _endSPI();
		return true;
  }
	else {
		return _writeErrorCheck(address, data);
  }
}

// Writes a float as four bytes starting from a specific address.
// Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> One float of data to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors
bool SPIFRAM::writeFloat(uint32_t address, float data, bool errorCheck) {
  const uint8_t size = (sizeof(data));

  if(!_prep(PAGEPROG, address, size)) {
    return false;
  }
  union
  {
    uint8_t b[size];
    float f;
  } var;
  var.f = data;

  _beginSPI(PAGEPROG);
  _nextBuf(PAGEPROG, &var.b[0], size);
  CHIP_DESELECT

  if (!errorCheck) {
    _endSPI();
    return true;
  }
  else {
    return _writeErrorCheck(address, data);
  }
}

// Reads a string from a specific location on a page.
// Takes three arguments -
//  	1. address --> Any address from 0 to maxAddress
//		2. inputString --> String variable to write the data from
//		3. errorCheck --> Turned on by default. Checks for writing errors
bool SPIFRAM::writeStr(uint32_t address, String &inputStr, bool errorCheck) {
  uint16_t inStrLen = inputStr.length() +1;
  if(!_prep(PAGEPROG, address, inStrLen)) {
    return false;
  }

  const uint16_t size = sizeof(inStrLen);
  union
  {
    uint8_t b[size];
    uint16_t w;
  } var;

  var.w = inStrLen;
  char inputChar[inStrLen];
  inputStr.toCharArray(inputChar, inStrLen);

  _beginSPI(PAGEPROG);
  _nextBuf(PAGEPROG, &var.b[0], size);
  _nextBuf(PAGEPROG, (uint8_t*)&inputChar, inStrLen);
  CHIP_DESELECT

  if (!errorCheck) {
    _endSPI();
    return true;
  }
  else {
    String tempStr;
    readStr(address, tempStr);
    return inputStr.equals(tempStr);
  }
}


//Erases a sector of predetermined size - defaults to one byte
// Takes the address & size fo sector to be erased as the arguments and erases the sector of
// the requested size, starting the address provided.
// Sector size can be indicated in bytes
bool SPIFRAM::eraseSector(uint32_t address, uint32_t sectorSize) {
	if(!_writeEnable())
 		return false;

	_beginSPI(PAGEPROG);
  _currentAddress = address;
  _transferAddress();
  for(uint16_t i = 0; i < sectorSize; i++) {
    _nextByte(EMPTYCELL);
  }
  _endSPI();

	return true;
}

//Erases the entire chip
bool SPIFRAM::eraseChip(void) {
	if(!_writeEnable()) {
 		return false;
  }

	_beginSPI(PAGEPROG);
  _currentAddress = 0x00;
  _transferAddress();
  for(uint16_t i = 0; i < capacity; i++) {
    _nextByte(EMPTYCELL);
  }
  _endSPI();

	return true;
}
