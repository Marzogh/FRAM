/* Arduino SPIFRAM Library v.0.0.1
 * Copyright (C) 2015 by Prajwal Bhattaram
 * Modified by Prajwal Bhattaram - 30/06/2016
 *
 * This file is part of the Arduino SPIFRAM Library. This library is for
 * Fujitsu FRAM memory modules. In its current form it enables reading
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
#include "SPIFRAM_defines.h"

#if defined (ARDUINO_ARCH_SAM) || defined (ARDUINO_ARCH_SAMD) || defined (ARDUINO_ARCH_ESP8266)
 #define _delay_us(us) delayMicroseconds(us)
#else
 #include <util/delay.h>
#endif

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//     Uncomment the code below to run a diagnostic if your flash 	  //
//	   						does not respond   						  //
//																	  //
// 		Error codes will be generated and returned on functions		  //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//#define RUNDIAGNOSTIC												  //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//   Uncomment the code below to increase the speed of the library    //
//	   				by disabling _notPrevWritten()   				  //
//																	  //
// Make sure the sectors being written to have been erased beforehand //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//#define HIGHSPEED
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

#if defined (ARDUINO_ARCH_SAM)
 #include <SPI.h>
#elif defined (ARDUINO_ARCH_AVR)
	#ifdef __AVR_ATtiny85__
		#define CHIP_SELECT   PORTB &= ~cs_mask;
		#define CHIP_DESELECT PORTB |=  cs_mask;
		#define SPIBIT                      \
		  USICR = ((1<<USIWM0)|(1<<USITC)); \
		  USICR = ((1<<USIWM0)|(1<<USITC)|(1<<USICLK));
		static uint8_t xfer(uint8_t n) {
			USIDR = n;
			SPIBIT
			SPIBIT
			SPIBIT
			SPIBIT
			SPIBIT
			SPIBIT
			SPIBIT
			SPIBIT
			return USIDR;
		}
	#else
        #include <SPI.h>
        #define CHIP_SELECT   *cs_port &= ~cs_mask;
        #define CHIP_DESELECT *cs_port |=  cs_mask;
        #define xfer(n)   SPI.transfer(n)
    #endif
#elif defined (ARDUINO_ARCH_ESP8266) || defined (ARDUINO_ARCH_SAMD)
        #include <SPI.h>
        #define CHIP_SELECT   digitalWrite(csPin, LOW);
        #define CHIP_DESELECT digitalWrite(csPin, HIGH);
        #define xfer(n)   SPI.transfer(n)
#endif

// Constructor
#if defined (ARDUINO_ARCH_SAM)
SPIFRAM::SPIFRAM(uint8_t cs, bool overflow) {
	csPin = cs;
	pageOverflow = overflow;
}
#elif defined (ARDUINO_ARCH_AVR)
SPIFRAM::SPIFRAM(uint8_t cs, bool overflow) {
	csPin = cs;
#ifndef __AVR_ATtiny85__
	cs_port = portOutputRegister(digitalPinToPort(csPin));
#endif
	cs_mask = digitalPinToBitMask(csPin);
	SPI.begin();
    SPI.setDataMode(0);
    SPI.setBitOrder(MSBFIRST);
    //SPI.setClockDivider(SPI_CLOCK_DIV2);
    pageOverflow = overflow;
    pinMode(cs, OUTPUT);
}
#elif defined (ARDUINO_ARCH_ESP8266) || defined (ARDUINO_ARCH_SAMD)
SPIFRAM::SPIFRAM(uint8_t cs, bool overflow) {
	csPin = cs;
	SPI.begin();
    SPI.setDataMode(0);
    SPI.setBitOrder(MSBFIRST);
    #ifdef ARDUINO_ARCH_ESP8266
    SPI.setFrequency(800000);
    #endif
    //SPI.setClockDivider(SPI_CLOCK_DIV2);
    pageOverflow = overflow;
    pinMode(cs, OUTPUT);
}
#endif

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//     Private functions used by read, write and erase operations     //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

// Select chip and issue command - data to follow
void SPIFRAM::_cmd(uint8_t cmd, bool _continue) {
	uint8_t cs = csPin;
 #if defined (ARDUINO_ARCH_SAM)
	if (!_continue)
		SPI.transfer(cs, cmd);
	else
		SPI.transfer(cs, cmd, SPI_CONTINUE);
 #else
	CHIP_SELECT
	(void)xfer(cmd);
 #endif
}

// Checks if status register 1 can be accessed - used during powerdown and power up and for debugging
uint8_t SPIFRAM::_readStat(void) {
	uint8_t stat1;
	_cmd(RDSR);
	#if defined (ARDUINO_ARCH_SAM)
	stat1 = SPI.transfer(csPin, 0x00);
	#else
	stat1 = xfer(0);
	CHIP_DESELECT
	#endif
	return stat1;
}

//Enables writing to chip by setting the WREN bit
bool SPIFRAM::_writeEnable(uint32_t timeout) {
	//uint8_t state;
  uint32_t startTime = millis();
  do{
    if (!(state & WRTEN)){
       #if (ARDUINO_ARCH_SAM)
       SPI.transfer(csPin, WREN);
       #else
       _cmd(WREN);
       CHIP_DESELECT
       #endif
       state = _readStat();
     }
     if((millis()-startTime) > timeout){
       #ifdef RUNDIAGNOSTIC
       errorcode = CHIPBUSY;
       _troubleshoot(errorcode);
       #endif
       return false;
     }
   } while (!(state & WRTEN));
   return true;
}

//Disables writing to chip by setting the Write Enable Latch (WEL) bit in the Status Register to 0
//_writeDisable() is not required under the following conditions because the Write Enable Latch (WEL) flag is cleared to 0
// i.e. to write disable state:
// Power-up, Write Disable, Page Program, Quad Page Program, Sector Erase, Block Erase, Chip Erase, Write Status Register,
// Erase Security Register and Program Security register
bool SPIFRAM::_writeDisable(void) {
	_cmd(WRDI);
	#if defined (ARDUINO_ARCH_AVR) || defined (ARDUINO_ARCH_ESP8266)
	CHIP_DESELECT
	#endif

	return true;
}

//Checks for presence of chip by requesting JEDEC ID
bool SPIFRAM::_getJedecId(uint8_t *b1, uint8_t *b2, uint8_t *b3, uint8_t *b4) {
  if(!_notBusy())
  	return false;
  _cmd(JEDECID);
  #if defined (ARDUINO_ARCH_SAM)
		*b1 = SPI.transfer(csPin, 0x00, SPI_CONTINUE);		// manufacturer id
		*b2 = SPI.transfer(csPin, 0x00, SPI_CONTINUE);		// continuation code
		uint8_t density = SPI.transfer(csPin, 0x00, SPI_CONTINUE);		// product id 1 (density)
    *b4 = SPI.transfer(csPin, 0x00)                   // product id 2
  #else
		*b1 = xfer(0); 		// manufacturer id
		*b2 = xfer(0); 		// continuation code
		uint8_t density = xfer(0);		// product id 1 (density)
    *b4 = xfer(0);    // product id 2
		CHIP_DESELECT
  #endif

  *b3 = density & 0x1F //this makes sure that the proprietary bits (7-5) are not read. Only bits 4-0
                       //make up the density ID

  return true;
}

//Identifies the chip
bool SPIFRAM::_chipID(void) {
	//Get Manfucturer/Device ID so the library can identify the chip
    uint8_t manID, conCode, density, prodID2 ;
    //_getManId(&manID, &devID);
    _getJedecId(&manID, &conCode &density, &prodID2);
    //Serial.println(manID, HEX);
    //Serial.println(capID, HEX);
    //Serial.println(devID, HEX);

    if (manID != FUJITSU_MANID){		//If the chip is not a Winbond Chip
    	#ifdef RUNDIAGNOSTIC
    	errorcode = UNKNOWNCHIP;		//Error code for unidentified chip
    	_troubleshoot(errorcode);
    	#endif
    	while(1);
    }

    //Check flash memory type and identify capacity
    uint8_t i;
    //capacity & chip name
    for (i = 0; i < sizeof(devType); i++)
    {
    	if (density == density[i]) {
    		capacity = memSize[i];
    		name = chipName[i];
        //Serial.println(devID, HEX);
        //Serial.println(capacity);
        //Serial.println(name);
    	}
    }
    if (capacity == 0) {
    	#ifdef RUNDIAGNOSTIC
    	errorcode = UNKNOWNCAP;		//Error code for unidentified capacity
    	_troubleshoot(errorcode);
    	#endif
    	while(1);
    }

   	/*#ifdef RUNDIAGNOSTIC
    char buffer[64];
    sprintf(buffer, "Manufacturer ID: %02xh\nMemory Type: %02xh\nCapacity: %lu\nmaxPage: %d", manID, devID, capacity, maxPage);
    Serial.println(buffer);
    #endif*/
    return true;
}

//Checks to see if pageOverflow is permitted and assists with determining next address to read/write.
//Sets the global address variable
uint32_t SPIFRAM::_addressCheck(uint32_t address, uint32_t size) {
	#ifdef RUNDIAGNOSTIC
	if (maxAddress == 0) {
		errorcode = CALLBEGIN;
		_troubleshoot(errorcode);
	}
	#endif
  for (uint32_t i = 0; i < size; i++) {
    if (address + i >= maxAddress) {
    	if (!pageOverflow) {
	    	#ifdef RUNDIAGNOSTIC
	    	errorcode = OUTOFBOUNDS;
	    	_troubleshoot(errorcode);
	    	#endif
	    	break;					// At end of memory - (!pageOverflow)
      }
      else {
        address = 0x00;					// At end of memory - (pageOverflow)
        return address;
      }
    }
  }
  return address;				// Not at end of memory if (address < capacity)
}

//Double checks all parameters before calling a Read
//Takes address and returns the address if true, else returns false. Throws an error if there is a problem.
uint32_t SPIFlash::_prepRead(uint32_t address, uint32_t size) {
  return _addressCheck(address, size));
}

//Initiates read operation - but data is not read yet
void SPIFRAM::_beginRead(uint32_t address) {
	_cmd(READ);
	#if defined (ARDUINO_ARCH_SAM)
  SPI.transfer(csPin, address >> 16, SPI_CONTINUE);
  SPI.transfer(csPin, address >> 8, SPI_CONTINUE);
  SPI.transfer(csPin, address, SPI_CONTINUE);
  #else
	(void)xfer(address >> 16);
	(void)xfer(address >> 8);
	(void)xfer(address);
	#endif
	//SPI data lines are left open until _endProcess() is called
}

//Reads next byte. Call 'n' times to read 'n' number of bytes. Should be called after _beginRead()
uint8_t SPIFRAM::_readNextByte(bool _continue) {
	uint8_t result;
	#if defined (ARDUINO_ARCH_SAM)
		if (!_continue)
			result = SPI.transfer(csPin, 0x00);
		else
			result = SPI.transfer(csPin, 0x00, SPI_CONTINUE);
	#else
		result = xfer(0x00);
	#endif
	return result;
}

//Double checks all parameters before initiating a write
//Takes address and returns address if true or 'false' if false. Throws an error if there is a problem.
bool SPIFRAM::_prepWrite(uint32_t address, uint32_t size) {
	if(!_writeEnable())
 		return false;

  #ifndef HIGHSPEED
  if(!_notPrevWritten(address, size);
  	return false;
  #endif

  if (!_addressCheck(address, size)) {
		#ifdef RUNDIAGNOSTIC
		errorcode = OUTOFBOUNDS;
 		_troubleshoot(errorcode);
 		#endif
 		return false;
  }
  else {
    return true;
  }
}

//Initiates write operation - but data is not written yet
bool SPIFRAM::_beginWrite(uint32_t address) {
	_cmd(WRITE);
	#if defined (ARDUINO_ARCH_SAM)
	SPI.transfer(csPin, address >> 16, SPI_CONTINUE);
  	SPI.transfer(csPin, address >> 8, SPI_CONTINUE);
  	SPI.transfer(csPin, address, SPI_CONTINUE);
    #else
  	(void)xfer(address >> 16);
    (void)xfer(address >> 8);
    (void)xfer(address);
	#endif
	//SPI data lines are left open until _endProcess() is called
	return true;
}

//Writes next byte. Call 'n' times to read 'n' number of bytes. Should be called after _beginWrite()
bool SPIFRAM::_writeNextByte(uint8_t b, bool _continue) {
	#if defined (ARDUINO_ARCH_SAM)
		if (!_continue)
			SPI.transfer(csPin, b);
		else
			SPI.transfer(csPin, b, SPI_CONTINUE);
	#else
		xfer(b);
	#endif
	return  true;
}

//Stops all operations. Should be called after all the required data is read/written from repeated _readNextByte()/_writeNextByte() calls
void SPIFRAM::_endProcess(void) {
	#if defined (ARDUINO_ARCH_AVR)
	CHIP_DESELECT
	#endif
	_delay_us(3);
}

#ifndef HIGHSPEED
bool SPIFRAM::_notPrevWritten(uint32_t address, uint32_t size) {
	uint32_t _size = size;
	uint32_t sampleSize;
	if (_size <= 10) {
		sampleSize = _size;
	}

	if (_size > 10) {
    sampleSize = 10;
		do {
			sampleSize++;
			_size/=10;
		} while((_size/10) >= 1);
	}

	uint32_t addresses[sampleSize];

	for (uint16_t i = 0; i < sampleSize; i++) {
		addresses[i] = (rand() % size) + address;
	}

	for (uint16_t i = 0; i < sampleSize; i++) {
		_beginRead(addresses[i]);
		if (_readNextByte() != 0xFF) {
			#ifdef RUNDIAGNOSTIC
			errorcode = PREVWRITTEN;
 			_troubleshoot(errorcode);
 			#endif
			return false;
		}
    #if defined (ARDUINO_ARCH_SAM)
    _readNextByte(NO_CONTINUE);
    _delay_us(3)
    #else
    CHIP_DESELECT
    _delay_us(3);
    #endif
	}
	return true;
}
#endif

#ifdef RUNDIAGNOSTIC
//Troubleshooting function. Called when #ifdef RUNDIAGNOSTIC is uncommented at the top of this file.
void SPIFRAM::_troubleshoot(uint8_t error) {

	switch (error) {
		case SUCCESS:
		#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega32U4__) || defined (__AVR_ATtiny85__)
 		Serial.print("Error code: 0x0");
		Serial.println(SUCCESS, HEX);
		#else
		Serial.println("Action completed successfully");
		#endif
		break;

 		case CALLBEGIN:
 		#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega32U4__) || defined (__AVR_ATtiny85__)
 		Serial.print("Error code: 0x0");
		Serial.println(CALLBEGIN, HEX);
		#else
 		Serial.println("*constructor_of_choice*.begin() was not called in void setup()");
		#endif
		break;

		case UNKNOWNCHIP:
		#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega32U4__) || defined (__AVR_ATtiny85__)
 		Serial.print("Error code: 0x0");
		Serial.println(UNKNOWNCHIP, HEX);
		#else
		Serial.println("Unable to identify chip. Are you shure this is a Winbond Flash chip");
 		Serial.println("Please raise an issue at http://www.github.com/Marzogh/SPIFRAM/issues with your chip type and I will try to add support to your chip");
		#endif

		break;

 		case UNKNOWNCAP:
 		#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega32U4__) || defined (__AVR_ATtiny85__)
 		Serial.print("Error code: 0x0");
		Serial.println(UNKNOWNCAP, HEX);
		#else
 		Serial.println("Unable to identify capacity.");
 		Serial.println("Please raise an issue at http://www.github.com/Marzogh/SPIFRAM/issues with your chip type and I will work on adding support to your chip");
		#endif
		break;

 		case CHIPBUSY:
 		#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega32U4__) || defined (__AVR_ATtiny85__)
 		Serial.print("Error code: 0x0");
		Serial.println(CHIPBUSY, HEX);
		#else
 		Serial.println("Chip is busy.");
 		Serial.println("Make sure all pins have been connected properly");
 		Serial.print("If it still doesn't work, ");
 		Serial.println("please raise an issue at http://www.github.com/Marzogh/SPIFRAM/issues with the details of what your were doing when this error occured");
		#endif
		break;

 		case OUTOFBOUNDS:
 		#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega32U4__) || defined (__AVR_ATtiny85__)
 		Serial.print("Error code: 0x0");
		Serial.println(OUTOFBOUNDS, HEX);
		#else
 		Serial.println("Page overflow has been disabled and the address called exceeds the memory");
		#endif
		break;

 		case CANTENWRITE:
 		#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega32U4__) || defined (__AVR_ATtiny85__)
 		Serial.print("Error code: 0x0");
		Serial.println(CANTENWRITE, HEX);
		#else
 		Serial.println("Unable to Enable Writing to chip.");
 		Serial.println("Please make sure the HOLD & WRITEPROTECT pins are connected properly to VCC & GND respectively");
 		Serial.print("If you are still facing issues, ");
 		Serial.println("please raise an issue at http://www.github.com/Marzogh/SPIFRAM/issues with the details of what your were doing when this error occured");
		#endif
		break;

		case PREVWRITTEN:
 		#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega32U4__) || defined (__AVR_ATtiny85__)
 		Serial.print("Error code: 0x0");
		Serial.println(PREVWRITTEN, HEX);
		#else
 		Serial.println("This sector already contains data.");
 		Serial.println("Please make sure the sectors being written to are erased.");
 		Serial.print("If you are still facing issues, ");
 		Serial.println("please raise an issue at http://www.github.com/Marzogh/SPIFRAM/issues with the details of what your were doing when this error occured");
		#endif
		break;

		default:
		#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega32U4__) || defined (__AVR_ATtiny85__)
 		Serial.print("Error code: 0x");
		Serial.println(UNKNOWNERROR, HEX);
		#else
		Serial.println("Unknown error");
 		Serial.println("Please raise an issue at http://www.github.com/Marzogh/SPIFRAM/issues with the details of what your were doing when this error occured");
		#endif
		break;
	}
}
#endif


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//     Public functions used for read, write and erase operations     //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

//Identifies chip and establishes parameters
void SPIFRAM::begin(void) {
	#if defined (ARDUINO_ARCH_SAM)
	SPI.begin(csPin);
	SPI.setClockDivider(csPin, 21);
	SPI.setDataMode(csPin, SPI_MODE0);
	#endif
	_chipID();
}

//Returns capacity of chip
uint32_t SPIFRAM::getCapacity() {
	return capacity;
}

//Returns maximum number of pages
uint32_t SPIFRAM::getMaxPage() {
	return maxPage;
}


//Returns identifying name of the chip
uint16_t SPIFRAM::getChipName() {
	return name;
}

//Checks for and initiates the chip by requesting the Manufacturer ID which is returned as a 16 bit int
uint16_t SPIFRAM::getManID() {
	uint8_t b1, b2;
    _getManId(&b1, &b2);
    uint32_t id = b1;
    id = (id << 8)|(b2 << 0);
    return id;
}

//Checks for and initiates the chip by requesting JEDEC ID which is returned as a 32 bit int
uint32_t SPIFRAM::getJEDECID() {
	uint8_t b1, b2, b3;
    _getJedecId(&b1, &b2, &b3);
    uint32_t id = b1;
    id = (id << 8)|(b2 << 0);
    id = (id << 8)|(b3 << 0);
    return id;
}

//Gets the next available address for use.
// Takes the size of the data as an argument and returns a 32-bit address
// All addresses in the in the sketch must be obtained via this function or not at all.
uint32_t SPIFRAM::getAddress(uint16_t size) {
	if (!_addressCheck(currentAddress, size)){
		#ifdef RUNDIAGNOSTIC
		errorcode = OUTOFBOUNDS;
 		_troubleshoot(errorcode);
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
	uint16_t inStrLen = inputStr.length() + 1;
	uint16_t size;

	//inputStr.toCharArray(inputChar, inStrLen);

	size=(sizeof(char)*inStrLen);
	size+=sizeof(inStrLen);

	return size;
}

// Reads a byte of data from a specific location.
// Takes one argument -
//		1. address --> Any address from 0 to maxAddress
uint8_t SPIFRAM::readByte(uint32_t address) {
	uint8_t data;
	uint32_t _address = _prepRead(address, sizeof(data));
	_beginRead(_address);
	data = _readNextByte(NO_CONTINUE);
	_endProcess();
  return data;
}

// Reads a char of data from a specific location.
// Takes one argument -
//		1. address --> Any address from 0 to maxAddress
int8_t SPIFRAM::readChar(uint32_t address) {
	int8_t data;
	uint32_t _address = _prepRead(address, sizeof(data));
  _beginRead(address);
	data = _readNextByte(NO_CONTINUE);
	_endProcess();
	return data;
}

// Reads an array of bytes starting at a specific location
// Takes one argument -
//		1. address --> Any address from 0 to maxAddress
bool  SPIFRAM::readByteArray(uint32_t address, uint8_t *data_buffer, uint16_t bufferSize) {
	uint32_t _address = _prepRead(address, bufferSize);
	_beginRead(_address);
	for (uint16_t a = 0; a < bufferSize; a++) {
		if (a == (bufferSize-1))
			data_buffer[a] = _readNextByte(NO_CONTINUE);
		else
			data_buffer[a] = _readNextByte();
	}
	_endProcess();
	return true;
}

// Reads an array of chars starting at a specific location
// Takes one argument -
//		1. address --> Any address from 0 to maxAddress
bool  SPIFRAM::readCharArray(uint32_t address, char *data_buffer, uint16_t bufferSize) {
uint32_t _address = _prepRead(address, bufferSize);
_beginRead(_address);_beginRead(address);
	for (uint16_t a = 0; a < bufferSize; a++) {
		if (a == (bufferSize-1))
			data_buffer[a] = _readNextByte(NO_CONTINUE);
		else
			data_buffer[a] = _readNextByte();
	}
	_endProcess();
	return true;
}

// Reads an unsigned int of data from a specific location.
// Takes one argument -
//		1. address --> Any address from 0 to maxAddress
uint16_t SPIFRAM::readWord(uint32_t address) {
	union
	{
		byte b[sizeof(uint16_t)];
		uint16_t I;
	} data;
	uint32_t _address = _prepRead(address, sizeof(data.I));
	_beginRead(_address);_beginRead(address);
	for (uint16_t i=0; i < (sizeof(int16_t)); i++) {
		if (i == (sizeof(uint16_t)-1))
			data.b[i] = _readNextByte(NO_CONTINUE);
		else
			data.b[i] = _readNextByte();
	}
	_endProcess();
	return data.I;
}

// Reads a signed int of data from a specific location.
// Takes one argument -
//		1. address --> Any address from 0 to maxAddress
int16_t SPIFRAM::readShort(uint32_t address) {
	union
	{
		byte b[sizeof(int16_t)];
		int16_t s;
	} data;
	uint32_t _address = _prepRead(address, sizeof(data.s));
	_beginRead(_address);
  for (uint16_t i=0; i < (sizeof(int16_t)); i++) {
		if (i == (sizeof(int16_t)-1))
			data.b[i] = _readNextByte(NO_CONTINUE);
		else
			data.b[i] = _readNextByte();
	}
	_endProcess();
	return data.s;
}

// Reads an unsigned long of data from a specific location.
// Takes one argument -
//		1. address --> Any address from 0 to maxAddress
uint32_t SPIFRAM::readULong(uint32_t address) {
	union
	{
		byte b[sizeof(uint32_t)];
		uint32_t l;
	} data;
	uint32_t _address = _prepRead(address, sizeof(data.l));
	_beginRead(_address);
  for (uint16_t i=0; i < (sizeof(uint32_t)); i++) {
		if (i == (sizeof(uint32_t)-1))
			data.b[i] = _readNextByte(NO_CONTINUE);
		else
			data.b[i] = _readNextByte();
	}
	_endProcess();
	return data.l;
}

// Reads a signed long of data from a specific location.
// Takes one argument -
//		1. address --> Any address from 0 to maxAddress
int32_t SPIFRAM::readLong(uint32_t address) {
	union
	{
		byte b[sizeof(int32_t)];
		int32_t l;
	} data;
	uint32_t _address = _prepRead(address, sizeof(data.l));
	_beginRead(_address);
  for (uint16_t i=0; i < (sizeof(int32_t)); i++) {
		if (i == (sizeof(int32_t)-1))
			data.b[i] = _readNextByte(NO_CONTINUE);
		else
			data.b[i] = _readNextByte();
	}
	_endProcess();
	return data.l;
}

// Reads a float of data from a specific location.
// Takes one argument -
//		1. address --> Any address from 0 to maxAddress
float SPIFRAM::readFloat(uint32_t address) {
	union
	{
		byte b[(sizeof(float))];
		float f;
	} data;
	uint32_t _address = _prepRead(address, sizeof(data.f));
	_beginRead(_address);
  for (uint16_t i=0; i < (sizeof(float)); i++) {
		if (i == (sizeof(float)-1))
			data.b[i] = _readNextByte(NO_CONTINUE);
		else
			data.b[i] = _readNextByte();
	}
	_endProcess();
	return data.f;
}

// Reads a string from a specific location.
// Takes two argument -
//		1. address --> Any address from 0 to maxAddress
//		2. outputString --> String variable to write the output to
bool SPIFRAM::readStr(uint32_t address, String &outStr) {
  uint16_t strLen;

  uint32_t _address = _prepRead(address, (strLen+sizeof(strLen));

  strLen = readShort(_address);
  _address+=(sizeof(strLen));

  char outputChar[strLen];

  readCharArray(_address, outputChar, strLen);

  outStr = String(outputChar);
  return true;
}

// Writes a byte of data to a specific location in a page.
//	Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> One byte of data to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors
// WARNING: You can write to previously written memory locations (see datasheet).
//      However, this behaviour is disabled by default.
// 			Use the erase()/eraseSector()/eraseBlock32K/eraseBlock64K commands to first clear memory (write 0xFFs)
bool SPIFRAM::writeByte(uint32_t address, uint8_t data, bool errorCheck) {
	if(!_prepWrite(address))
		return false;

	#ifndef HIGHSPEED
	if(!_notPrevWritten(address, sizeof(data)))
		return false;
	#endif

	_beginWrite(address);
	_writeNextByte(data, NO_CONTINUE);
	_endProcess();

	if (!errorCheck)
		return true;
	else
		return _writeErrorCheck(address, data);
}

// Writes a char of data to a specific location in a page.
//  Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> One char of data to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors
// WARNING: You can write to previously written memory locations (see datasheet).
//      However, this behaviour is disabled by default.
// 			Use the erase()/eraseSector()/eraseBlock32K/eraseBlock64K commands to first clear memory (write 0xFFs)
bool SPIFRAM::writeChar(uint32_t address, int8_t data, bool errorCheck) {
	if (!_prepWrite(address))
		return false;

	#ifndef HIGHSPEED
	if(!_notPrevWritten(address, sizeof(data)))
		return false;
	#endif

	_beginWrite(address);
	_writeNextByte(data, NO_CONTINUE);
	_endProcess();

	if (!errorCheck)
		return true;
	else
		return _writeErrorCheck(address, data);

}

// Writes an array of bytes starting from a specific location in a page.
//  Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> An array of bytes to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors
// WARNING: You can write to previously written memory locations (see datasheet).
//      However, this behaviour is disabled by default.
// 			Use the erase()/eraseSector()/eraseBlock32K/eraseBlock64K commands to first clear memory (write 0xFFs)
bool SPIFRAM::writeByteArray(uint32_t address, uint8_t *data_buffer, uint16_t bufferSize, bool errorCheck) {
	if (!_prepWrite(address, bufferSize))
		return false;

	#ifndef HIGHSPEED
	if(!_notPrevWritten(address, bufferSize))
		return false;
	#endif

	_beginWrite(address);

	for (uint16_t i = 0; i < bufferSize; i++) {
		if (i == (bufferSize-1))
			_writeNextByte(data_buffer[i], NO_CONTINUE);
		else
			_writeNextByte(data_buffer[i]);
	}
	_endProcess();

	if (!errorCheck)
		return true;
	else
		return _writeErrorCheck(address, data_buffer);
}

// Writes an array of bytes starting from a specific location in a page.
//  Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> An array of chars to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors
// WARNING: You can write to previously written memory locations (see datasheet).
//      However, this behaviour is disabled by default.
// 			Use the erase()/eraseSector()/eraseBlock32K/eraseBlock64K commands to first clear memory (write 0xFFs)
bool SPIFRAM::writeCharArray(uint32_t address, char *data_buffer, uint16_t bufferSize, bool errorCheck) {
	if (!_prepWrite(address, bufferSize))
		return false;

	#ifndef HIGHSPEED
	if(!_notPrevWritten(address, bufferSize))
		return false;
	#endif

	_beginWrite(address);

	for (uint16_t i = 0; i < bufferSize; i++) {
		if (i == (bufferSize-1))
			_writeNextByte(data_buffer[i], NO_CONTINUE);
		else
			_writeNextByte(data_buffer[i]);
	}
	_endProcess();

	if (!errorCheck)
		return true;
	else
		return _writeErrorCheck(address, data_buffer);
}

// Writes an unsigned int as two bytes starting from a specific location in a page.
//  Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> One unsigned int of data to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors
// WARNING: You can write to previously written memory locations (see datasheet).
//      However, this behaviour is disabled by default.
// 			Use the erase()/eraseSector()/eraseBlock32K/eraseBlock64K commands to first clear memory (write 0xFFs)
bool SPIFRAM::writeWord(uint32_t address, uint16_t data, bool errorCheck) {
	if(!_prepWrite(address))
		return false;

	#ifndef HIGHSPEED
	if(!_notPrevWritten(address, sizeof(data)))
		return false;
	#endif

	union
	{
		uint8_t b[sizeof(data)];
		uint16_t w;
	} var;
	var.w = data;

	_beginWrite(address);
	for (uint16_t j = 0; j < sizeof(data); j++) {
		if (j == (sizeof(data)-1))
  	  		_writeNextByte(var.b[j], NO_CONTINUE);
  		else
      		_writeNextByte(var.b[j]);
	}
	_endProcess();

		if (!errorCheck)
		return true;
	else
		return _writeErrorCheck(address, data);
}

// Writes a signed int as two bytes starting from a specific location in a page.
//  Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> One signed int of data to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors
// WARNING: You can write to previously written memory locations (see datasheet).
//      However, this behaviour is disabled by default.
// 			Use the erase()/eraseSector()/eraseBlock32K/eraseBlock64K commands to first clear memory (write 0xFFs)
bool SPIFRAM::writeShort(uint32_t address, int16_t data, bool errorCheck) {
	if(!_prepWrite(address))
		return false;

	#ifndef HIGHSPEED
	if(!_notPrevWritten(address, sizeof(data)))
		return false;
	#endif

	union
	{
		uint8_t b[sizeof(data)];
		int16_t s;
	} var;
	var.s = data;
	_beginWrite(address);
	for (uint16_t j = 0; j < (sizeof(data)); j++) {
		if (j == (sizeof(data)-1))
		_writeNextByte(var.b[j], NO_CONTINUE);
	else
		_writeNextByte(var.b[j]);
	}
	_endProcess();

	if (!errorCheck)
		return true;
	else
		return _writeErrorCheck(address, data);
}

// Writes an unsigned long as four bytes starting from a specific location in a page.
//  Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> One unsigned long of data to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors
// WARNING: You can write to previously written memory locations (see datasheet).
//      However, this behaviour is disabled by default.
// 			Use the erase()/eraseSector()/eraseBlock32K/eraseBlock64K commands to first clear memory (write 0xFFs)
bool SPIFRAM::writeULong(uint32_t address, uint32_t data, bool errorCheck) {
	if(!_prepWrite(address))
		return false;

	#ifndef HIGHSPEED
	if(!_notPrevWritten(address, sizeof(data)))
		return false;
	#endif

	union
	{
		uint8_t b[sizeof(data)];
		uint32_t l;
	} var;
	var.l = data;
	_beginWrite(address);
	for (uint16_t j = 0; j < (sizeof(data)); j++) {
		if (j == (sizeof(data)-1))
		_writeNextByte(var.b[j], NO_CONTINUE);
	else
		_writeNextByte(var.b[j]);
	}
	_endProcess();

	if (!errorCheck)
		return true;
	else
		return _writeErrorCheck(address, data);
}

// Writes a signed long as four bytes starting from a specific location in a page.
//  Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> One signed long of data to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors
// WARNING: You can write to previously written memory locations (see datasheet).
//      However, this behaviour is disabled by default.
// 			Use the erase()/eraseSector()/eraseBlock32K/eraseBlock64K commands to first clear memory (write 0xFFs)
bool SPIFRAM::writeLong(uint32_t address, int32_t data, bool errorCheck) {
	if(!_prepWrite(address))
		return false;

	#ifndef HIGHSPEED
	if(!_notPrevWritten(address, sizeof(data)))
		return false;
	#endif

	union
	{
		uint8_t b[sizeof(data)];
		int32_t l;
	} var;
	var.l = data;
	_beginWrite(address);
	for (uint16_t j = 0; j < (sizeof(data)); j++) {
		if (j == (sizeof(data)-1))
		_writeNextByte(var.b[j], NO_CONTINUE);
	else
		_writeNextByte(var.b[j]);
	}
	_endProcess();

	if (!errorCheck)
		return true;
	else
		return _writeErrorCheck(address, data);
}

// Writes a float as four bytes starting from a specific location in a page.
//  Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> One float of data to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors
// WARNING: You can write to previously written memory locations (see datasheet).
//      However, this behaviour is disabled by default.
// 			Use the erase()/eraseSector()/eraseBlock32K/eraseBlock64K commands to first clear memory (write 0xFFs)
bool SPIFRAM::writeFloat(uint32_t address, float data, bool errorCheck) {
	if(!_prepWrite(address))
		return false;

	#ifndef HIGHSPEED
	if(!_notPrevWritten(address, sizeof(data)))
		return false;
	#endif

	union
	{
		uint8_t b[sizeof(data)];
		float f;
	} var;
	var.f = data;
	_beginWrite(address);
	for (uint16_t j = 0; j < (sizeof(data)); j++) {
		if (j == (sizeof(data)-1))
		_writeNextByte(var.b[j], NO_CONTINUE);
	else
		_writeNextByte(var.b[j]);
	}
	_endProcess();

	if (!errorCheck)
		return true;
	else
		return _writeErrorCheck(address, data);
}

// Reads a string from a specific location on a page.
//  Takes two arguments -
//  	1. address --> Any address from 0 to maxAddress
//		2. inputString --> String variable to write the data from
//		3. errorCheck --> Turned on by default. Checks for writing errors
// WARNING: You can write to previously written memory locations (see datasheet).
//      However, this behaviour is disabled by default.
// 			Use the erase()/eraseSector()/eraseBlock32K/eraseBlock64K commands to first clear memory (write 0xFFs)
bool SPIFRAM::writeStr(uint32_t address, String &inputStr, bool errorCheck) {
  uint16_t inStrLen = inputStr.length() +1;
  char inputChar[inStrLen];

  union
  {
    uint8_t b[sizeof(inStrLen)];
    uint16_t w;
  }var;
  var.w = inStrLen;
  inputStr.toCharArray(inputChar, inStrLen);

  if(!_prepWrite(address, inStrLen))
		return false;

	#ifndef HIGHSPEED
	if(!_notPrevWritten(address, inStrLen))
		return false;
	#endif

  _beginWrite(address);

  for (uint16_t j = 0; j < sizeof(inStrLen); j++) {
    _writeNextByte(var.b[j]);
  }

  for (uint16_t i = 0; i <inStrLen; i++) {
    if (i == (inStrLen-1)) {
      _writeNextByte(inputChar[i], NO_CONTINUE);
    }
    else {
      _writeNextByte(inputChar[i]);
    }
  }
  _endProcess();

  if (!errorCheck) {
    return true;
  }
  else {
    String tempStr;
    readStr(address, tempStr);

    return inputStr.equals(tempStr);
  }
}

//Erases one 4k sector. Has two variants:
//	A. Takes the address as the argument and erases the sector containing the address.
//	B. Takes page to be erased as the argument and erases the sector containing the page.
//	The sectors are numbered 0 - 255 containing 16 pages each.
//			Page 0-15 --> Sector 0; Page 16-31 --> Sector 1;......Page 4080-4095 --> Sector 255
// Variant A
bool SPIFRAM::eraseSector(uint32_t address) {
	if(!_writeEnable())
 		return false;

	_cmd(SECTORERASE);
	#if defined (ARDUINO_ARCH_SAM)
 	SPI.transfer(csPin, address >> 16, SPI_CONTINUE);
 	SPI.transfer(csPin, address >> 8, SPI_CONTINUE);
 	SPI.transfer(csPin, 0x00);
	#else
	(void)xfer(address >> 16);
	(void)xfer(address >> 8);
	(void)xfer(0);
	CHIP_DESELECT
	#endif

	if(!_notBusy(410L))
		return false;	//Datasheet says erasing a sector takes 400ms max

		//_writeDisable(); //_writeDisable() is not required because the Write Enable Latch (WEL) flag is cleared to 0
		// i.e. to write disable state upon the following conditions:
		// Power-up, Write Disable, Page Program, Quad Page Program, ``Sector Erase``, Block Erase, Chip Erase, Write Status Register,
		// Erase Security Register and Program Security register

	return true;
}
// Variant B
bool SPIFRAM::eraseSector(uint16_t page_number, uint8_t offset) {
	uint32_t address = _getAddress(page_number, offset);
	return eraseSector(address);
}

//Erases one 32k block. Has two variants:
//	A. Takes the address as the argument and erases the block containing the address.
//	B. Takes page to be erased as the argument and erases the block containing the page.
//	The blocks are numbered 0 - 31 containing 128 pages each.
// 			Page 0-127 --> Block 0; Page 128-255 --> Block 1;......Page 3968-4095 --> Block 31
// Variant A
bool SPIFRAM::eraseBlock32K(uint32_t address) {
	if(!_notBusy()||!_writeEnable())
 		return false;

	_cmd(BLOCK32ERASE);
	#if defined (ARDUINO_ARCH_SAM)
 	SPI.transfer(csPin, address >> 16, SPI_CONTINUE);
 	SPI.transfer(csPin, address >> 8, SPI_CONTINUE);
 	SPI.transfer(csPin, 0x00);
	#else
	(void)xfer(address >> 16);
	(void)xfer(address >> 8);
	(void)xfer(0);
	CHIP_DESELECT
	#endif

	if(!_notBusy(410L))
	return false;	//Datasheet says erasing a sector takes 400ms max

	//_writeDisable(); //_writeDisable() is not required because the Write Enable Latch (WEL) flag is cleared to 0
	// i.e. to write disable state upon the following conditions:
	// Power-up, Write Disable, Page Program, Quad Page Program, Sector Erase, ``Block Erase``, Chip Erase, Write Status Register,
	// Erase Security Register and Program Security register

	return true;
}
// Variant B
bool SPIFRAM::eraseBlock32K(uint16_t page_number, uint8_t offset) {
	uint32_t address = _getAddress(page_number, offset);
	return eraseBlock32K(address);
}

//Erases one 64k block. Has two variants:
//	A. Takes the address as the argument and erases the block containing the address.
//	B. Takes page to be erased as the argument and erases the block containing the page.
//	The blocks are numbered 0 - 15 containing 256 pages each.
// 				Page 0-255 --> Block 0; Page 256-511 --> Block 1;......Page 3840-4095 --> Block 15
//	Variant A
bool SPIFRAM::eraseBlock64K(uint32_t address) {
	if(!_notBusy()||!_writeEnable())
 		return false;

	_cmd(BLOCK64ERASE);
	#if defined (ARDUINO_ARCH_SAM)
 	SPI.transfer(csPin, address >> 16, SPI_CONTINUE);
 	SPI.transfer(csPin, address >> 8, SPI_CONTINUE);
 	SPI.transfer(csPin, 0x00);
	#else
	(void)xfer(address >> 16);
	(void)xfer(address >> 8);
	(void)xfer(0);
	CHIP_DESELECT
	#endif

	if(!_notBusy(410L))
		return false;	//Datasheet says erasing a sector takes 400ms max

	//_writeDisable(); //_writeDisable() is not required because the Write Enable Latch (WEL) flag is cleared to 0
	// i.e. to write disable state upon the following conditions:
	// Power-up, Write Disable, Page Program, Quad Page Program, Sector Erase, ``Block Erase``, Chip Erase, Write Status Register,
	// Erase Security Register and Program Security register

	return true;
}
//	Variant B
bool SPIFRAM::eraseBlock64K(uint16_t page_number, uint8_t offset) {
	uint32_t address = _getAddress(page_number, offset);
	return eraseBlock64K(address);
}

//Erases whole chip. Think twice before using.
bool SPIFRAM::eraseChip(void) {
	if(!_notBusy()||!_writeEnable())
 		return false;

	_cmd(CHIPERASE, NO_CONTINUE);
	#if defined (ARDUINO_ARCH_AVR) || defined (ARDUINO_ARCH_ESP8266)
	CHIP_DESELECT
	#endif

	if(!_notBusy(7000L))
		return false; //Datasheet says erasing chip takes 6s max

	//_writeDisable(); //_writeDisable() is not required because the Write Enable Latch (WEL) flag is cleared to 0
	// i.e. to write disable state upon the following conditions:
	// Power-up, Write Disable, Page Program, Quad Page Program, Sector Erase, Block Erase, ``Chip Erase``, Write Status Register,
	// Erase Security Register and Program Security register

	return true;

}

//Suspends current Block Erase/Sector Erase/Page Program. Does not suspend chipErase().
//Page Program, Write Status Register, Erase instructions are not allowed.
//Erase suspend is only allowed during Block/Sector erase.
//Program suspend is only allowed during Page/Quad Page Program
bool SPIFRAM::suspendProg(void) {
	if(_notBusy() || !_noSuspend())
		return false;

	_cmd(SUSPEND, NO_CONTINUE);
	#if defined (ARDUINO_ARCH_AVR) || defined (ARDUINO_ARCH_ESP8266)
	CHIP_DESELECT
	#endif

	_delay_us(20);

	if(!_notBusy() || _noSuspend())	//Max suspend Enable time according to datasheet
		return false;
	return true;
}

//Resumes previously suspended Block Erase/Sector Erase/Page Program.
bool SPIFRAM::resumeProg(void) {
	if(!_notBusy() || _noSuspend())
		return false;

	_cmd(RESUME, NO_CONTINUE);
	#if defined (ARDUINO_ARCH_AVR) || defined (ARDUINO_ARCH_ESP8266)
	CHIP_DESELECT
	#endif

	_delay_us(20);

	if(_notBusy() || !_noSuspend())
		return false;
	return true;

}

//Puts device in low power state. Good for battery powered operations.
//Typical current consumption during power-down is 1mA with a maximum of 5mA. (Datasheet 7.4)
//In powerDown() the chip will only respond to powerUp()
bool SPIFRAM::powerDown(void) {
	if(!_notBusy())
		return false;

	_cmd(POWERDOWN, NO_CONTINUE);
	#if defined (ARDUINO_ARCH_AVR) || defined (ARDUINO_ARCH_ESP8266)
	CHIP_DESELECT
	#endif
	_delay_us(3);							//Max powerDown enable time according to the Datasheet

	uint8_t status1 = _readStat();
	uint8_t status2 = _readStat();
	status1 = _readStat();

	if (status1 != 255 && status2 != 255) {
		if (status1 == status2 || status1 == 0 || status2 == 0) {
			status1 = _readStat();
			status2 = _readStat();
		}
		else if (status1 != status2)
			return true;
	}
	else if (status1 == 255 && status2 == 255)
		return true;
	else if (status1 == 0 && status2 == 0)
		return false;
	return true;
}

//Wakes chip from low power state.
bool SPIFRAM::powerUp(void) {

	_cmd(RELEASE, NO_CONTINUE);
	#if defined (ARDUINO_ARCH_AVR) || defined (ARDUINO_ARCH_ESP8266)
	CHIP_DESELECT
	#endif
	_delay_us(3);						    //Max release enable time according to the Datasheet

	if (_readStat() == 255)
		return false;
	return true;
}
