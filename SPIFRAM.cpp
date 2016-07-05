/* Arduino SPIFRAM Library v.0.0.1
 * Copyright (C) 2015 by Prajwal Bhattaram
 * Modified by Prajwal Bhattaram - 01/07/2016
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
uint8_t SPIFRAM::_readStat1(void) {
	uint8_t stat1;
	_cmd(READSTAT1);
	#if defined (ARDUINO_ARCH_SAM)
	stat1 = SPI.transfer(csPin, 0x00);
	#else
	stat1 = xfer(0);
	CHIP_DESELECT
	#endif
	return stat1;
}

// Checks the erase/program suspend flag before enabling/disabling a program/erase suspend operation
bool SPIFRAM::_noSuspend(void) {
	uint8_t stat2;

	_cmd(READSTAT2);
	#if defined (ARDUINO_ARCH_SAM)
		stat2 = SPI.transfer(csPin, 0x00);
	#else
		stat2 = xfer(0);
		CHIP_DESELECT
	#endif

	if(stat2 & SUS)
		return false;
	return true;
}

// Polls the status register 1 until busy flag is cleared or timeout
bool SPIFRAM::_notBusy(uint32_t timeout) {
	//uint8_t state;
	uint32_t startTime = millis();

	do {
    state = _readStat1();
		if((millis()-startTime) > timeout){
			#ifdef RUNDIAGNOSTIC
			errorcode = CHIPBUSY;
			_troubleshoot(errorcode);
			#endif
			return false;
		}
	} while(state & BUSY);
	return true;
}

//Enables writing to chip by setting the WRITEENABLE bit
bool SPIFRAM::_writeEnable(uint32_t timeout) {
	//uint8_t state;
  uint32_t startTime = millis();
  do{
    if (!(state & WRTEN)){
       #if (ARDUINO_ARCH_SAM)
       SPI.transfer(csPin, WRITEENABLE);
       #else
       _cmd(WRITEENABLE);
       CHIP_DESELECT
       #endif
       state = _readStat1();
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
	_cmd(WRITEDISABLE);
	#if defined (ARDUINO_ARCH_AVR) || defined (ARDUINO_ARCH_ESP8266)
	CHIP_DESELECT
	#endif

	return true;
}

//Gets address from page number and offset. Takes two arguments:
// 1. page_number --> Any page number from 0 to maxPage
// 2. offset --> Any offset within the page - from 0 to 255
uint32_t SPIFRAM::_getAddress(uint16_t page_number, uint8_t offset) {
	uint32_t address = page_number;
	return ((address << 8) + offset);
}

//Checks the device ID to establish storage parameters
bool SPIFRAM::_getManId(uint8_t *b1, uint8_t *b2) {
	if(!_notBusy())
		return false;
	_cmd(MANID);
	#if defined (ARDUINO_ARCH_SAM)
		SPI.transfer(csPin, 0x00, SPI_CONTINUE);
		SPI.transfer(csPin, 0x00, SPI_CONTINUE);
		SPI.transfer(csPin, 0x00, SPI_CONTINUE);
		*b1 = SPI.transfer(csPin, 0x00, SPI_CONTINUE);
		*b2 = SPI.transfer(csPin, 0x00);
	#else
		xfer(0);
		xfer(0);
		xfer(0);
		*b1 = xfer(0);
		*b2 = xfer(0);
		CHIP_DESELECT
	#endif
	return true;
}

//Checks for presence of chip by requesting JEDEC ID
bool SPIFRAM::_getJedecId(uint8_t *b1, uint8_t *b2, uint8_t *b3) {
  if(!_notBusy())
  	return false;
  _cmd(JEDECID);
  #if defined (ARDUINO_ARCH_SAM)
		*b1 = SPI.transfer(csPin, 0x00, SPI_CONTINUE);		// manufacturer id
		*b2 = SPI.transfer(csPin, 0x00, SPI_CONTINUE);		// manufacturer id
		*b3 = SPI.transfer(csPin, 0x00);					// capacity
  #else
		*b1 = xfer(0); 		// manufacturer id
		*b2 = xfer(0); 		// memory type
		*b3 = xfer(0);		// capacity
		CHIP_DESELECT
  #endif
  return true;
}

//Identifies the chip
bool SPIFRAM::_chipID(void) {
	//Get Manfucturer/Device ID so the library can identify the chip
    uint8_t manID, capID, devID ;
    //_getManId(&manID, &devID);
    _getJedecId(&manID, &capID, &devID);
    //Serial.println(manID, HEX);
    //Serial.println(capID, HEX);
    //Serial.println(devID, HEX);

    if (manID != WINBOND_MANID && manID != MICROCHIP_MANID){		//If the chip is not a Winbond Chip
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
    	if (devID == devType[i]) {
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

   	maxPage = capacity/PAGESIZE;

   	/*#ifdef RUNDIAGNOSTIC
    char buffer[64];
    sprintf(buffer, "Manufacturer ID: %02xh\nMemory Type: %02xh\nCapacity: %lu\nmaxPage: %d", manID, devID, capacity, maxPage);
    Serial.println(buffer);
    #endif*/
    return true;
}

//Checks to see if pageOverflow is permitted and assists with determining next address to read/write.
//Sets the global address variable
bool SPIFRAM::_addressCheck(uint32_t address, uint32_t size) {
	#ifdef RUNDIAGNOSTIC
	if (capacity == 0) {
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

//Double checks all parameters before calling a Read
//Has two variants:
//	A. Takes address and returns the address if true, else returns false. Throws an error if there is a problem.
//	B. Takes page_number and offset and returns address. Throws an error if there is a problem
// Variant A
bool SPIFRAM::_prepRead(uint32_t address, uint32_t size) {
	if(!_notBusy()){
    return false;
  }
	if (!_addressCheck(address, size)){
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
// Variant B
bool SPIFRAM::_prepRead(uint16_t page_number, uint8_t offset, uint32_t size) {
	uint32_t address = _getAddress(page_number, offset);
	return _prepRead(address, size);
}

//Initiates read operation - but data is not read yet
void SPIFRAM::_beginRead(uint32_t address) {
	_cmd(READDATA);
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

//Initiates fast read operation (can operate at the highest possible freq of 104 MHz (at 3.0-3.6V) of 80 MHz (at 2.5-3.6V))
//- but data is not read yet
void SPIFRAM::_beginFastRead(uint32_t address) {
	_cmd(FASTREAD);
	#if defined (ARDUINO_ARCH_SAM)
	SPI.transfer(csPin, address >> 16, SPI_CONTINUE);
  	SPI.transfer(csPin, address >> 8, SPI_CONTINUE);
  	SPI.transfer(csPin, address, SPI_CONTINUE);
    SPI.transfer(csPin, 0x00, SPI_CONTINUE);
  	#else
	(void)xfer(address >> 16);
	(void)xfer(address >> 8);
	(void)xfer(address);
  xfer(0);
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
//Has two variants:
//	A. Takes address and returns address if true or 'false' if false. Throws an error if there is a problem.
//	B. Takes page_number and offset and returns address. Throws an error if there is a problem
// Variant A
bool SPIFRAM::_prepWrite(uint32_t address, uint32_t size) {
	if(!_notBusy()||!_writeEnable()){
    return false;
  }

  #ifndef HIGHSPEED
  if(!_notPrevWritten(address, size))
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
// Variant B
bool SPIFRAM::_prepWrite(uint16_t page_number, uint8_t offset, uint32_t size) {
	uint32_t address = _getAddress(page_number, offset);
	return (_prepWrite(address, size));
}

//Initiates write operation - but data is not written yet
bool SPIFRAM::_beginWrite(uint32_t address) {
	_cmd(PAGEPROG);
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
    _delay_us(3);
    #else
    CHIP_DESELECT
    _delay_us(3);
    #endif
	}
	return true;
}

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
		Serial.println("Unable to identify chip. Are you sure this is a Winbond Flash chip");
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
 		Serial.println("please raise an issue at http://www.github.com/Marzogh/SPIFRAM/issues with the details of what your were doing when this error occurred");
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
 		Serial.println("please raise an issue at http://www.github.com/Marzogh/SPIFRAM/issues with the details of what your were doing when this error occurred");
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
 		Serial.println("please raise an issue at http://www.github.com/Marzogh/SPIFRAM/issues with the details of what your were doing when this error occurred");
		#endif
		break;

		default:
		#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega32U4__) || defined (__AVR_ATtiny85__)
 		Serial.print("Error code: 0x");
		Serial.println(UNKNOWNERROR, HEX);
		#else
		Serial.println("Unknown error");
 		Serial.println("Please raise an issue at http://www.github.com/Marzogh/SPIFRAM/issues with the details of what your were doing when this error occurred");
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

uint8_t SPIFRAM::error() {
	return errorcode;
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

//Gets the next available address for use. Has two variants:
//	A. Takes the size of the data as an argument and returns a 32-bit address
//	B. Takes a three variables, the size of the data and two other variables to return a page number value & an offset into.
// All addresses in the in the sketch must be obtained via this function or not at all.
// Variant A
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
// Variant B
bool SPIFRAM::getAddress(uint16_t size, uint16_t &page_number, uint8_t &offset) {
	uint32_t address = getAddress(size);
	offset = (address >> 0);
	page_number = (address >> 8);
	return true;
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

// Reads a byte of data from a specific location in a page.
// Has two variants:
//	A. Takes two arguments -
//		1. address --> Any address from 0 to maxAddress
//		2. fastRead --> defaults to false - executes _beginFastRead() if set to true
//	B. Takes three arguments -
//  	1. page --> Any page number from 0 to maxPage
//  	2. offset --> Any offset within the page - from 0 to 255
//		3. fastRead --> defaults to false - executes _beginFastRead() if set to true
// Variant A
uint8_t SPIFRAM::readByte(uint32_t address, bool fastRead) {
	uint8_t data;
	if (!_prepRead(address, sizeof(data)))
		return false;
	else {
		if(!fastRead)
			_beginRead(_currentAddress);
		else
			_beginFastRead(_currentAddress);

		data = _readNextByte(NO_CONTINUE);
		_endProcess();
		return data;
	}
}
// Variant B
uint8_t SPIFRAM::readByte(uint16_t page_number, uint8_t offset, bool fastRead) {
	uint32_t address = _getAddress(page_number, offset);

	return readByte(address, fastRead);
}

// Reads a char of data from a specific location in a page.
// Has two variants:
//	A. Takes two arguments -
//		1. address --> Any address from 0 to maxAddress
//		2. fastRead --> defaults to false - executes _beginFastRead() if set to true
//	B. Takes three arguments -
//  	1. page --> Any page number from 0 to maxPage
//  	2. offset --> Any offset within the page - from 0 to 255
//		3. fastRead --> defaults to false - executes _beginFastRead() if set to true
// Variant A
int8_t SPIFRAM::readChar(uint32_t address, bool fastRead) {
	int8_t data;
	if (!_prepRead(address, sizeof(data)))
		return false;
	else {
		if(!fastRead)
			_beginRead(_currentAddress);
		else
			_beginFastRead(_currentAddress);

		data = _readNextByte(NO_CONTINUE);
		_endProcess();
		return data;
	}
}
// Variant B
int8_t SPIFRAM::readChar(uint16_t page_number, uint8_t offset, bool fastRead) {
	uint32_t address = _getAddress(page_number, offset);

	return readChar(address, fastRead);
}

// Reads an array of bytes starting from a specific location in a page.// Has two variants:
//	A. Takes three arguments
//		1. address --> Any address from 0 to maxAddress
//		2. data_buffer --> The array of bytes to be read from the flash memory - starting at the address indicated
//		3. fastRead --> defaults to false - executes _beginFastRead() if set to true
//	B. Takes four arguments
//		1. page --> Any page number from 0 to maxPage
//		2. offset --> Any offset within the page - from 0 to 255
//		3. data_buffer --> The array of bytes to be read from the flash memory - starting at the offset on the page indicated
//		4. fastRead --> defaults to false - executes _beginFastRead() if set to true
// Variant A
bool  SPIFRAM::readByteArray(uint32_t address, uint8_t *data_buffer, uint16_t bufferSize, bool fastRead) {
	if (!_prepRead(address, bufferSize)) {
    return false;
	}
	else {
		if(!fastRead)
			_beginRead(_currentAddress);
		else
			_beginFastRead(_currentAddress);

		for (uint16_t a = 0; a < bufferSize; a++) {
			if (a == (bufferSize-1))
				data_buffer[a] = _readNextByte(NO_CONTINUE);
			else
				data_buffer[a] = _readNextByte();
		}
		_endProcess();
	}
	return true;
}
// Variant B
bool  SPIFRAM::readByteArray(uint16_t page_number, uint8_t offset, uint8_t *data_buffer, uint16_t bufferSize, bool fastRead) {
	uint32_t address = _getAddress(page_number, offset);

	return readByteArray(address, data_buffer, bufferSize, fastRead);
}

// Reads an array of chars starting from a specific location in a page.// Has two variants:
//	A. Takes three arguments
//		1. address --> Any address from 0 to maxAddress
//		2. data_buffer --> The array of bytes to be read from the flash memory - starting at the address indicated
//		3. fastRead --> defaults to false - executes _beginFastRead() if set to true
//	B. Takes four arguments
//		1. page --> Any page number from 0 to maxPage
//		2. offset --> Any offset within the page - from 0 to 255
//		3. data_buffer --> The array of bytes to be read from the flash memory - starting at the offset on the page indicated
//		4. fastRead --> defaults to false - executes _beginFastRead() if set to true
// Variant A
bool  SPIFRAM::readCharArray(uint32_t address, char *data_buffer, uint16_t bufferSize, bool fastRead) {
	if (!_prepRead(address, bufferSize)) {
    return false;
  }
  else {
		if(!fastRead)
			_beginRead(_currentAddress);
		else
			_beginFastRead(_currentAddress);

		for (uint16_t a = 0; a < bufferSize; a++) {
			if (a == (bufferSize-1))
				data_buffer[a] = _readNextByte(NO_CONTINUE);
			else
				data_buffer[a] = _readNextByte();
		}
		_endProcess();
		return true;
	}
	return true;
}
// Variant B
bool  SPIFRAM::readCharArray(uint16_t page_number, uint8_t offset, char *data_buffer, uint16_t bufferSize, bool fastRead) {
	uint32_t address = _getAddress(page_number, offset);

	return readCharArray(address, data_buffer, bufferSize, fastRead);
}

// Reads an unsigned int of data from a specific location in a page.
// Has two variants:
//	A. Takes two arguments -
//		1. address --> Any address from 0 to maxAddress
//		2. fastRead --> defaults to false - executes _beginFastRead() if set to true
//	B. Takes three arguments -
//  	1. page --> Any page number from 0 to maxPage
//  	2. offset --> Any offset within the page - from 0 to 255
//		3. fastRead --> defaults to false - executes _beginFastRead() if set to true
// Variant A
uint16_t SPIFRAM::readWord(uint32_t address, bool fastRead) {
	union
	{
		byte b[sizeof(uint16_t)];
		uint16_t I;
	} data;
	if (!_prepRead(address, sizeof(data.I)))
		return false;
	else {
		if(!fastRead)
			_beginRead(_currentAddress);
		else
			_beginFastRead(_currentAddress);

		for (uint16_t i=0; i < (sizeof(int16_t)); i++) {
			if (i == (sizeof(uint16_t)-1))
				data.b[i] = _readNextByte(NO_CONTINUE);
			else
				data.b[i] = _readNextByte();
		}
		_endProcess();
		return data.I;
	}
}
// Variant B
uint16_t SPIFRAM::readWord(uint16_t page_number, uint8_t offset, bool fastRead) {
	uint32_t address = _getAddress(page_number, offset);

	return readWord(address, fastRead);
}

// Reads a signed int of data from a specific location in a page.
// Has two variants:
//	A. Takes two arguments -
//		1. address --> Any address from 0 to maxAddress
//		2. fastRead --> defaults to false - executes _beginFastRead() if set to true
//	B. Takes three arguments -
//  	1. page --> Any page number from 0 to maxPage
//  	2. offset --> Any offset within the page - from 0 to 255
//		3. fastRead --> defaults to false - executes _beginFastRead() if set to true
// Variant A
int16_t SPIFRAM::readShort(uint32_t address, bool fastRead) {
	union
	{
		byte b[sizeof(int16_t)];
		int16_t s;
	} data;
	if (!_prepRead(address, sizeof(data.s)))
		return false;
	else {
		if(!fastRead)
			_beginRead(_currentAddress);
		else
			_beginFastRead(_currentAddress);

		for (uint16_t i=0; i < (sizeof(int16_t)); i++) {
			if (i == (sizeof(int16_t)-1))
				data.b[i] = _readNextByte(NO_CONTINUE);
			else
				data.b[i] = _readNextByte();
		}
		_endProcess();
		return data.s;
	}
}
// Variant B
int16_t SPIFRAM::readShort(uint16_t page_number, uint8_t offset, bool fastRead) {
	uint32_t address = _getAddress(page_number, offset);

	return readShort(address, fastRead);
}

// Reads an unsigned long of data from a specific location in a page.
// Has two variants:
//	A. Takes two arguments -
//		1. address --> Any address from 0 to maxAddress
//		2. fastRead --> defaults to false - executes _beginFastRead() if set to true
//	B. Takes three arguments -
//  	1. page --> Any page number from 0 to maxPage
//  	2. offset --> Any offset within the page - from 0 to 255
//		3. fastRead --> defaults to false - executes _beginFastRead() if set to true
// Variant A
uint32_t SPIFRAM::readULong(uint32_t address, bool fastRead) {
	union
	{
		byte b[sizeof(uint32_t)];
		uint32_t l;
	} data;
	if (!_prepRead(address, sizeof(data.l)))
		return false;
	else {
		if(!fastRead)
			_beginRead(_currentAddress);
		else
			_beginFastRead(_currentAddress);

		for (uint16_t i=0; i < (sizeof(uint32_t)); i++) {
			if (i == (sizeof(uint32_t)-1))
				data.b[i] = _readNextByte(NO_CONTINUE);
			else
				data.b[i] = _readNextByte();
		}
		_endProcess();
		return data.l;
	}
}
// Variant B
uint32_t SPIFRAM::readULong(uint16_t page_number, uint8_t offset, bool fastRead) {
	uint32_t address = _getAddress(page_number, offset);

	return readULong(address, fastRead);
}

// Reads a signed long of data from a specific location in a page.
// Has two variants:
//	A. Takes two arguments -
//		1. address --> Any address from 0 to maxAddress
//		2. fastRead --> defaults to false - executes _beginFastRead() if set to true
//	B. Takes three arguments -
//  	1. page --> Any page number from 0 to maxPage
//  	2. offset --> Any offset within the page - from 0 to 255
//		3. fastRead --> defaults to false - executes _beginFastRead() if set to true
// Variant A
int32_t SPIFRAM::readLong(uint32_t address, bool fastRead) {
	union
	{
		byte b[sizeof(int32_t)];
		int32_t l;
	} data;
	if (!_prepRead(address, sizeof(data.l)))
		return false;
	else {
		if(!fastRead)
			_beginRead(_currentAddress);
		else
			_beginFastRead(_currentAddress);

		for (uint16_t i=0; i < (sizeof(int32_t)); i++) {
			if (i == (sizeof(int32_t)-1))
				data.b[i] = _readNextByte(NO_CONTINUE);
			else
				data.b[i] = _readNextByte();
		}
		_endProcess();
		return data.l;
	}
}
// Variant B
int32_t SPIFRAM::readLong(uint16_t page_number, uint8_t offset, bool fastRead) {
	uint32_t address = _getAddress(page_number, offset);

	return readLong(address, fastRead);
}

// Reads a signed long of data from a specific location in a page.
// Has two variants:
//	A. Takes two arguments -
//		1. address --> Any address from 0 to maxAddress
//		2. fastRead --> defaults to false - executes _beginFastRead() if set to true
//	B. Takes three arguments -
//  	1. page --> Any page number from 0 to maxPage
//  	2. offset --> Any offset within the page - from 0 to 255
//		3. fastRead --> defaults to false - executes _beginFastRead() if set to true
// Variant A
float SPIFRAM::readFloat(uint32_t address, bool fastRead) {
	union
	{
		byte b[(sizeof(float))];
		float f;
	} data;
	if (!_prepRead(address, sizeof(data.f)))
		return false;
	else {
		if(!fastRead)
			_beginRead(_currentAddress);
		else
			_beginFastRead(_currentAddress);

		for (uint16_t i=0; i < (sizeof(float)); i++) {
			if (i == (sizeof(float)-1))
				data.b[i] = _readNextByte(NO_CONTINUE);
			else
				data.b[i] = _readNextByte();
		}
		_endProcess();
		return data.f;
	}
}
// Variant B
float SPIFRAM::readFloat(uint16_t page_number, uint8_t offset, bool fastRead) {
	uint32_t address = _getAddress(page_number, offset);

	return readFloat(address, fastRead);
}

// Reads a string from a specific location on a page.
// Has two variants:
//	A. Takes three arguments
//		1. address --> Any address from 0 to maxAddress
//		2. outputString --> String variable to write the output to
//		3. fastRead --> defaults to false - executes _beginFastRead() if set to true
//	B. Takes four arguments
//		1. page --> Any page number from 0 to maxPage
//		2. offset --> Any offset within the page - from 0 to 255
//		3. outputString --> String variable to write the output to
//		4. fastRead --> defaults to false - executes _beginFastRead() if set to true
// This function first reads a short from the address to figure out the size of the String object stored and
// then reads the String object data
// Variant A
bool SPIFRAM::readStr(uint32_t address, String &outStr, bool fastRead) {
  uint16_t strLen;

  strLen = readShort(address);
  address+=(sizeof(strLen));
  if (!_prepRead(address, (strLen + sizeof(strLen))))
		return false;

  char outputChar[strLen];

  readCharArray(address, outputChar, strLen, fastRead);

  outStr = String(outputChar);
  return true;
}
// Variant B
bool SPIFRAM::readStr(uint16_t page_number, uint8_t offset, String &outStr, bool fastRead) {
  uint32_t address = _getAddress(page_number, offset);
  return readStr(address, outStr, fastRead);
}

// Reads a page of data into a page buffer. Takes three arguments -
//  1. page --> Any page number from 0 to maxPage
//  2. data_buffer --> a data buffer to read the data into (This HAS to be an array of 256 bytes)
//	3. fastRead --> defaults to false - executes _beginFastRead() if set to true
bool  SPIFRAM::readPage(uint16_t page_number, uint8_t *data_buffer, bool fastRead) {
	uint32_t address = _getAddress(page_number);
  if(!_prepRead(address, PAGESIZE)) {
    return false;
  }

	if(!fastRead)
		_beginRead(_currentAddress);
	else
		_beginFastRead(_currentAddress);

	for (int a = 0; a < PAGESIZE; a++) {
		if (a == (PAGESIZE - 1))
				data_buffer[a] = _readNextByte(NO_CONTINUE);
			else
				data_buffer[a] = _readNextByte();
	}
	_endProcess();
	return true;
}

// Writes a byte of data to a specific location in a page.
// Has two variants:
//	A. Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> One byte of data to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors
//	B. Takes four arguments -
//  	1. page --> Any page number from 0 to maxPage
//  	2. offset --> Any offset within the page - from 0 to 255
//  	3. data --> One byte of data to be written to a particular location on a page
//		4. errorCheck --> Turned on by default. Checks for writing errors
// WARNING: You can only write to previously erased memory locations (see datasheet).
// 			Use the eraseSector()/eraseBlock32K/eraseBlock64K commands to first clear memory (write 0xFFs)
// Variant A
bool SPIFRAM::writeByte(uint32_t address, uint8_t data, bool errorCheck) {
	if(!_prepWrite(address, sizeof(data)))
		return false;

	_beginWrite(_currentAddress);
	_writeNextByte(data, NO_CONTINUE);
	_endProcess();

	if (!errorCheck)
		return true;
	else
		return _writeErrorCheck(address, data);
}
// Variant B
bool SPIFRAM::writeByte(uint16_t page_number, uint8_t offset, uint8_t data, bool errorCheck) {
	uint32_t address = _getAddress(page_number, offset);

	return writeByte(address, data, errorCheck);
}

// Writes a char of data to a specific location in a page.
// Has two variants:
//	A. Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> One char of data to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors
//	B. Takes four arguments -
//  	1. page --> Any page number from 0 to maxPage
//  	2. offset --> Any offset within the page - from 0 to 255
//  	3. data --> One char of data to be written to a particular location on a page
//		4. errorCheck --> Turned on by default. Checks for writing errors
// WARNING: You can only write to previously erased memory locations (see datasheet).
// 			Use the eraseSector()/eraseBlock32K/eraseBlock64K commands to first clear memory (write 0xFFs)
// Variant A
bool SPIFRAM::writeChar(uint32_t address, int8_t data, bool errorCheck) {
	if (!_prepWrite(address, sizeof(data)))
		return false;

	_beginWrite(_currentAddress);
	_writeNextByte(data, NO_CONTINUE);
	_endProcess();

	if (!errorCheck)
		return true;
	else
		return _writeErrorCheck(address, data);

}
// Variant B
bool SPIFRAM::writeChar(uint16_t page_number, uint8_t offset, int8_t data, bool errorCheck) {
	uint32_t address = _getAddress(page_number, offset);

	return writeChar(address, data, errorCheck);

}

// Writes an array of bytes starting from a specific location in a page.
// Has two variants:
//	A. Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> An array of bytes to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors
//	B. Takes four arguments -
//  	1. page --> Any page number from 0 to maxPage
//  	2. offset --> Any offset within the page - from 0 to 255
//  	3. data --> An array of bytes to be written to a particular location on a page
//		4. errorCheck --> Turned on by default. Checks for writing errors
// WARNING: You can only write to previously erased memory locations (see datasheet).
// 			Use the eraseSector()/eraseBlock32K/eraseBlock64K commands to first clear memory (write 0xFFs)
// Variant A
bool SPIFRAM::writeByteArray(uint32_t address, uint8_t *data_buffer, uint16_t bufferSize, bool errorCheck) {
	if (!_prepWrite(address, bufferSize))
		return false;

	_beginWrite(_currentAddress);

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
// Variant B
bool SPIFRAM::writeByteArray(uint16_t page_number, uint8_t offset, uint8_t *data_buffer, uint16_t bufferSize, bool errorCheck) {
	uint32_t address = _getAddress(page_number, offset);

	return writeByteArray(address, data_buffer, bufferSize, errorCheck);
}

// Writes an array of bytes starting from a specific location in a page.
// Has two variants:
//	A. Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> An array of chars to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors
//	B. Takes four arguments -
//  	1. page --> Any page number from 0 to maxPage
//  	2. offset --> Any offset within the page - from 0 to 255
//  	3. data --> An array of chars to be written to a particular location on a page
//		4. errorCheck --> Turned on by default. Checks for writing errors
// WARNING: You can only write to previously erased memory locations (see datasheet).
// 			Use the eraseSector()/eraseBlock32K/eraseBlock64K commands to first clear memory (write 0xFFs)
// Variant A
bool SPIFRAM::writeCharArray(uint32_t address, char *data_buffer, uint16_t bufferSize, bool errorCheck) {
	if (!_prepWrite(address, bufferSize))
		return false;

	_beginWrite(_currentAddress);

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
// Variant B
bool SPIFRAM::writeCharArray(uint16_t page_number, uint8_t offset, char *data_buffer, uint16_t bufferSize, bool errorCheck) {
	uint32_t address = _getAddress(page_number, offset);

	return writeCharArray(address, data_buffer, bufferSize, errorCheck);
}

// Writes an unsigned int as two bytes starting from a specific location in a page.
// Has two variants:
//	A. Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> One unsigned int of data to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors
//	B. Takes four arguments -
//  	1. page --> Any page number from 0 to maxPage
//  	2. offset --> Any offset within the page - from 0 to 255
//  	3. data --> One unsigned int of data to be written to a particular location on a page
//		4. errorCheck --> Turned on by default. Checks for writing errors
// WARNING: You can only write to previously erased memory locations (see datasheet).
// 			Use the eraseSector()/eraseBlock32K/eraseBlock64K commands to first clear memory (write 0xFFs)
// Variant A
bool SPIFRAM::writeWord(uint32_t address, uint16_t data, bool errorCheck) {
	if(!_prepWrite(address, sizeof(data)))
		return false;

	union
	{
		uint8_t b[sizeof(data)];
		uint16_t w;
	} var;
	var.w = data;

	_beginWrite(_currentAddress);
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
// Variant B
bool SPIFRAM::writeWord(uint16_t page_number, uint8_t offset, uint16_t data, bool errorCheck) {
	uint32_t address = _getAddress(page_number, offset);

	return writeWord(address, data, errorCheck);
}

// Writes a signed int as two bytes starting from a specific location in a page.
// Has two variants:
//	A. Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> One signed int of data to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors
//	B. Takes four arguments -
//  	1. page --> Any page number from 0 to maxPage
//  	2. offset --> Any offset within the page - from 0 to 255
//  	3. data --> One signed int of data to be written to a particular location on a page
//		4. errorCheck --> Turned on by default. Checks for writing errors
// WARNING: You can only write to previously erased memory locations (see datasheet).
// 			Use the eraseSector()/eraseBlock32K/eraseBlock64K commands to first clear memory (write 0xFFs)
// Variant A
bool SPIFRAM::writeShort(uint32_t address, int16_t data, bool errorCheck) {
	if(!_prepWrite(address, sizeof(data)))
		return false;

	union
	{
		uint8_t b[sizeof(data)];
		int16_t s;
	} var;
	var.s = data;
	_beginWrite(_currentAddress);
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
// Variant B
bool SPIFRAM::writeShort(uint16_t page_number, uint8_t offset, int16_t data, bool errorCheck) {
	uint32_t address = _getAddress(page_number, offset);

	return writeShort(address, data, errorCheck);
}

// Writes an unsigned long as four bytes starting from a specific location in a page.
// Has two variants:
//	A. Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> One unsigned long of data to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors
//	B. Takes four arguments -
//  	1. page --> Any page number from 0 to maxPage
//  	2. offset --> Any offset within the page - from 0 to 255
//  	3. data --> One unsigned long of data to be written to a particular location on a page
//		4. errorCheck --> Turned on by default. Checks for writing errors
// WARNING: You can only write to previously erased memory locations (see datasheet).
// 			Use the eraseSector()/eraseBlock32K/eraseBlock64K commands to first clear memory (write 0xFFs)
// Variant A
bool SPIFRAM::writeULong(uint32_t address, uint32_t data, bool errorCheck) {
	if(!_prepWrite(address, sizeof(data)))
		return false;

	union
	{
		uint8_t b[sizeof(data)];
		uint32_t l;
	} var;
	var.l = data;
	_beginWrite(_currentAddress);
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
// Variant B
bool SPIFRAM::writeULong(uint16_t page_number, uint8_t offset, uint32_t data, bool errorCheck) {
	uint32_t address = _getAddress(page_number, offset);

	return writeULong(address, data, errorCheck);
}

// Writes a signed long as four bytes starting from a specific location in a page.
// Has two variants:
//	A. Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> One signed long of data to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors
//	B. Takes four arguments -
//  	1. page --> Any page number from 0 to maxPage
//  	2. offset --> Any offset within the page - from 0 to 255
//  	3. data --> One signed long of data to be written to a particular location on a page
//		4. errorCheck --> Turned on by default. Checks for writing errors
// WARNING: You can only write to previously erased memory locations (see datasheet).
// 			Use the eraseSector()/eraseBlock32K/eraseBlock64K commands to first clear memory (write 0xFFs)
// Variant A
bool SPIFRAM::writeLong(uint32_t address, int32_t data, bool errorCheck) {
	if(!_prepWrite(address, sizeof(data)))
		return false;

	union
	{
		uint8_t b[sizeof(data)];
		int32_t l;
	} var;
	var.l = data;
	_beginWrite(_currentAddress);
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
// Variant B
bool SPIFRAM::writeLong(uint16_t page_number, uint8_t offset, int32_t data, bool errorCheck) {
	uint32_t address = _getAddress(page_number, offset);

	return writeLong(address, data, errorCheck);
}

// Writes a float as four bytes starting from a specific location in a page.
// Has two variants:
//	A. Takes three arguments -
//  	1. address --> Any address - from 0 to maxAddress
//  	2. data --> One float of data to be written to a particular location on a page
//		3. errorCheck --> Turned on by default. Checks for writing errors
//	B. Takes four arguments -
//  	1. page --> Any page number from 0 to maxPage
//  	2. offset --> Any offset within the page - from 0 to 255
//  	3. data --> One float of data to be written to a particular location on a page
//		4. errorCheck --> Turned on by default. Checks for writing errors
// WARNING: You can only write to previously erased memory locations (see datasheet).
// 			Use the eraseSector()/eraseBlock32K/eraseBlock64K commands to first clear memory (write 0xFFs)
// Variant A
bool SPIFRAM::writeFloat(uint32_t address, float data, bool errorCheck) {
	if(!_prepWrite(address, sizeof(data)))
		return false;

	union
	{
		uint8_t b[sizeof(data)];
		float f;
	} var;
	var.f = data;
	_beginWrite(_currentAddress);
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
// Variant B
bool SPIFRAM::writeFloat(uint16_t page_number, uint8_t offset, float data, bool errorCheck) {
	uint32_t address = _getAddress(page_number, offset);

	return writeFloat(address, data, errorCheck);
}

// Reads a string from a specific location on a page.
// Has two variants:
//	A. Takes two arguments -
//  	1. address --> Any address from 0 to maxAddress
//		2. inputString --> String variable to write the data from
//		3. errorCheck --> Turned on by default. Checks for writing errors
//	B. Takes four arguments -
//  	1. page --> Any page number from 0 to maxPage
//  	2. offset --> Any offset within the page - from 0 to 255
//		3. inputString --> String variable to write the data from
//		4. errorCheck --> Turned on by default. Checks for writing errors
// WARNING: You can only write to previously erased memory locations (see datasheet).
// 			Use the eraseSector()/eraseBlock32K/eraseBlock64K commands to first clear memory (write 0xFFs)
// This function first writes the size of the string as an unsigned int to the address to figure out the size of the String object stored and
// then writes the String object data. Therefore it takes up two bytes more than the size of the String itself.
// Variant A
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

  _beginWrite(_currentAddress);

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
// Variant B
bool SPIFRAM::writeStr(uint16_t page_number, uint8_t offset, String &inputStr, bool errorCheck) {
  uint32_t address = _getAddress(page_number, offset);
  return writeStr(address, inputStr, errorCheck);
}

// Writes a page of data from a data_buffer array. Make sure the sizeOf(uint8_t data_buffer[]) >= PAGESIZE.
//	errorCheck --> Turned on by default. Checks for writing errors.
// WARNING: You can only write to previously erased memory locations (see datasheet).
// 			Use the eraseSector()/eraseBlock32K/eraseBlock64K commands to first clear memory (write 0xFFs)
bool SPIFRAM::writePage(uint16_t page_number, uint8_t *data_buffer, bool errorCheck) {
	uint32_t address = _getAddress(page_number);
  if (!_prepWrite(address, PAGESIZE)) {
    return false;
  }

	_beginWrite(_currentAddress);
	for (uint16_t i = 0; i < PAGESIZE; i++){
		if (i == (PAGESIZE-1)) {
      _writeNextByte(data_buffer[i], NO_CONTINUE);
    }
    else {
      _writeNextByte(data_buffer[i]);
    }
	}
	_endProcess();

	if (!errorCheck)
		return true;
	else {
    if (!_prepRead(address, PAGESIZE)) {
      return false;
    }
    _beginRead(_currentAddress);
    for (uint16_t j = 0; j < PAGESIZE; j++){
      if(j == PAGESIZE-1) {
        if (data_buffer[j] != _readNextByte(NO_CONTINUE)) {
          return false;
        }
      }
      else {
        if (data_buffer[j] != _readNextByte()) {
          return false;
        }
      }
    }
    _endProcess();
    return true;
  }
}


//Erases one 4k sector. Has two variants:
//	A. Takes the address as the argument and erases the sector containing the address.
//	B. Takes page to be erased as the argument and erases the sector containing the page.
//	The sectors are numbered 0 - 255 containing 16 pages each.
//			Page 0-15 --> Sector 0; Page 16-31 --> Sector 1;......Page 4080-4095 --> Sector 255
// Variant A
bool SPIFRAM::eraseSector(uint32_t address) {
	if(!_notBusy()||!_writeEnable())
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

	uint8_t status1 = _readStat1();
	uint8_t status2 = _readStat1();
	status1 = _readStat1();

	if (status1 != 255 && status2 != 255) {
		if (status1 == status2 || status1 == 0 || status2 == 0) {
			status1 = _readStat1();
			status2 = _readStat1();
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

	if (_readStat1() == 255)
		return false;
	return true;
}
