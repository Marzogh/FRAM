/* Arduino SPIFRAM Library v 0.0.1b
 * Copyright (C) 2015 by Prajwal Bhattaram
 * Modified by Prajwal Bhattaram - 13/11/2016
 *
 * This file is part of the Arduino SPIFRAM Library. This library is for
 * Winbond NOR FRAM memory modules. In its current form it enables reading
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

#ifndef SPIFRAM_H
#define SPIFRAM_H

#if defined (ARDUINO_ARCH_SAM)
  #include <malloc.h>
  #include <stdlib.h>
  #include <stdio.h>
#endif
#include <Arduino.h>
#ifndef __AVR_ATtiny85__
  #include <SPI.h>
#endif
#include "defines.h"

#if defined (ARDUINO_ARCH_SAM) || defined (ARDUINO_ARCH_SAMD) || defined (ARDUINO_ARCH_ESP8266) || defined (SIMBLEE) || defined (ARDUINO_ARCH_ESP32)
 #define _delay_us(us) delayMicroseconds(us)
#else
 #include <util/delay.h>
#endif

#ifdef ARDUINO_ARCH_AVR
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
    #define CHIP_SELECT   *cs_port &= ~cs_mask;
    #define CHIP_DESELECT *cs_port |=  cs_mask;
    #define xfer(n)   SPI.transfer(n)
  #endif
#elif defined (ARDUINO_ARCH_SAM)
    #define CHIP_SELECT   digitalWrite(csPin, LOW);
    #define CHIP_DESELECT digitalWrite(csPin, HIGH);
    #define xfer   _dueSPITransfer
#else //#elif defined (ARDUINO_ARCH_ESP8266) || defined (ARDUINO_ARCH_SAMD)
  #define CHIP_SELECT   digitalWrite(csPin, LOW);
  #define CHIP_DESELECT digitalWrite(csPin, HIGH);
  #define xfer(n)   SPI.transfer(n)
#endif

#define LIBVER 0
#define LIBSUBVER 1
#define BUGFIXVER 0

#if defined (ARDUINO_ARCH_SAM)
  extern char _end;
  extern "C" char *sbrk(int i);
  //char *ramstart=(char *)0x20070000;
  //char *ramend=(char *)0x20088000;
#endif

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//     Uncomment the code below to run a diagnostic if your flash 	  //
//                         does not respond                           //
//                                                                    //
//      Error codes will be generated and returned on functions       //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
#define RUNDIAGNOSTIC                                               //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//   Uncomment the code below to increase the speed of the library    //
//                  by disabling _notPrevWritten()                    //
//                                                                    //
// Make sure the sectors being written to have been erased beforehand //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//#define HIGHSPEED                                                   //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

class SPIFRAM {
public:
  //----------------------------------------------Constructor-----------------------------------------------//
  SPIFRAM(uint8_t cs = CS, bool overflow = true);
  //----------------------------------------Initial / Chip Functions----------------------------------------//
  void     begin(uint32_t _size = 0x00);
  void     setClock(uint32_t clockSpeed);
  bool     libver(uint8_t *b1, uint8_t *b2, uint8_t *b3);
  uint8_t  error(void);
  uint32_t getJEDECID(void);
  uint32_t getAddress(uint16_t size);
  uint16_t sizeofStr(String &inputStr);
  uint32_t getCapacity(void);
  uint32_t getMaxPage(void);
  //-------------------------------------------Write / Read Bytes-------------------------------------------//
  bool     writeByte(uint32_t address, uint8_t data, bool errorCheck = true);
  uint8_t  readByte(uint32_t address);
  //----------------------------------------Write / Read Byte Arrays----------------------------------------//
  bool     writeByteArray(uint32_t address, uint8_t *data_buffer, uint16_t bufferSize, bool errorCheck = true);
  bool     readByteArray(uint32_t address, uint8_t *data_buffer, uint16_t bufferSize);
  //-------------------------------------------Write / Read Chars-------------------------------------------//
  bool     writeChar(uint32_t address, int8_t data, bool errorCheck = true);
  int8_t   readChar(uint32_t address);
  //----------------------------------------Write / Read Char Arrays----------------------------------------//
  bool     writeCharArray(uint32_t address, char *data_buffer, uint16_t bufferSize, bool errorCheck = true);
  bool     readCharArray(uint32_t address, char *data_buffer, uint16_t buffer_size);
  //------------------------------------------Write / Read Shorts------------------------------------------//
  bool     writeShort(uint32_t address, int16_t data, bool errorCheck = true);
  int16_t  readShort(uint32_t address);
  //-------------------------------------------Write / Read Words-------------------------------------------//
  bool     writeWord(uint32_t address, uint16_t data, bool errorCheck = true);
  uint16_t readWord(uint32_t address);
  //-------------------------------------------Write / Read Longs-------------------------------------------//
  bool     writeLong(uint32_t address, int32_t data, bool errorCheck = true);
  int32_t  readLong(uint32_t address);
  //--------------------------------------Write / Read Unsigned Longs---------------------------------------//
  bool     writeULong(uint32_t address, uint32_t data, bool errorCheck = true);
  uint32_t readULong(uint32_t address);
  //-------------------------------------------Write / Read Floats------------------------------------------//
  bool     writeFloat(uint32_t address, float data, bool errorCheck = true);
  float    readFloat(uint32_t address);
  //------------------------------------------Write / Read Strings------------------------------------------//
  bool     writeStr(uint32_t address, String &inputStr, bool errorCheck = true);
  bool     readStr(uint32_t address, String &outStr);
  //------------------------------------------Write / Read Anything-----------------------------------------//
  template <class T> bool writeAnything(uint32_t address, const T& value, bool errorCheck = true);
  template <class T> bool readAnything(uint32_t address, T& value);
  //--------------------------------------------Erase functions---------------------------------------------//
  bool     eraseSector(uint32_t address, uint32_t sectorSize = 0x01);
  bool     eraseChip(void);
  //-------------------------------------Public Arduino Due Functions---------------------------------------//
#if defined (ARDUINO_ARCH_SAM)
  uint32_t dueFreeRAM(void);
#endif
  //-------------------------------------------Public variables---------------------------------------------//

private:
#if defined (ARDUINO_ARCH_SAM)
  //-------------------------------------Private Arduino Due Functions--------------------------------------//
  void     _dmac_disable(void);
  void     _dmac_enable(void);
  void     _dmac_channel_disable(uint32_t ul_num);
  void     _dmac_channel_enable(uint32_t ul_num);
  bool     _dmac_channel_transfer_done(uint32_t ul_num);
  void     _dueSPIDmaRX(uint8_t* dst, uint16_t count);
  void     _dueSPIDmaRX(char* dst, uint16_t count);
  void     _dueSPIDmaTX(const uint8_t* src, uint16_t count);
  void     _dueSPIDmaCharTX(const char* src, uint16_t count);
  void     _dueSPIBegin(void);
  void     _dueSPIInit(uint8_t dueSckDivisor);
  uint8_t  _dueSPITransfer(uint8_t b);
  uint8_t  _dueSPIRecByte(void);
  uint8_t  _dueSPIRecByte(uint8_t* buf, size_t len);
  int8_t   _dueSPIRecChar(void);
  int8_t   _dueSPIRecChar(char* buf, size_t len);
  void     _dueSPISendByte(uint8_t b);
  void     _dueSPISendByte(const uint8_t* buf, size_t len);
  void     _dueSPISendChar(char b);
  void     _dueSPISendChar(const char* buf, size_t len);
#endif
  //--------------------------------------------Private functions-------------------------------------------//
  void     _troubleshoot(void);
  void     _cmd(uint8_t cmd, bool _continue = true);
  void     _endProcess(void);
  void     _errorCodeCheck(void);
  void     _endSPI(void);
  bool     _prep(uint8_t opcode, uint32_t address, uint32_t size);
  bool     _prep(uint8_t opcode, uint32_t page_number, uint8_t offset, uint32_t size);
  bool     _startSPIBus(void);
  bool     _beginSPI(uint8_t opcode);
  bool     _notPrevWritten(uint32_t address, uint32_t size = 1);
  bool     _writeEnable(uint32_t timeout = 10L);
  bool     _writeDisable(void);
  bool     _getJedecId(uint8_t *b1, uint8_t *b2, uint8_t *b3, uint8_t *b4);
  bool     _chipID(void);
  bool     _transferAddress(void);
  bool     _addressCheck(uint32_t address, uint32_t size = 1);
  uint8_t  _nextByte(uint8_t data = NULLBYTE);
  uint16_t _nextInt(uint16_t = NULLINT);
  void     _nextBuf(uint8_t opcode, uint8_t *data_buffer, uint32_t size);
  uint8_t  _readStat1(void);
  template <class T> bool _writeErrorCheck(uint32_t address, const T& value);
  //-------------------------------------------Private variables------------------------------------------//
  bool        pageOverflow, SPIBusState;
  volatile uint8_t *cs_port;
  uint8_t     cs_mask, csPin, errorcode, state, _SPCR, _SPSR;
  uint32_t    capacity, currentAddress, _currentAddress = 0;
#ifdef SPI_HAS_TRANSACTION
  SPISettings _settings;
#endif
#ifndef CHIPSIZE
  const uint16_t devType[1]   = {0x04};
  const uint32_t memSize[1]  = {KB64}; //memory size in Kilo and Mega bits
#endif
};

//--------------------------------------------Templates-------------------------------------------//

// Writes any type of data to a specific location in the FRAM memory.
// Takes three arguments -
//    1. address --> Any address from 0 to maxAddress
//    2. T& value --> Variable to write data from
//    4. errorCheck --> Turned on by default. Checks for writing errors
template <class T> bool SPIFRAM::writeAnything(uint32_t address, const T& value, bool errorCheck) {
  if (!_prep(PAGEPROG, address, sizeof(value))) {
    return false;
  }
  const uint8_t* p = ((const uint8_t*)(const void*)&value);
  _beginSPI(PAGEPROG);
  for (uint16_t i = 0; i < sizeof(value); ++i) {
    _nextByte(*p++);
  }
  CHIP_DESELECT
  /*uint32_t size = sizeof(value);
  _beginSPI(PAGEPROG);
  _nextBuf(PAGEPROG, (uint8_t*)&value, size);
  Serial.print("...");
  CHIP_DESELECT*/

  if (!errorCheck) {
    _endSPI();
    return true;
  }
  else {
    return _writeErrorCheck(address, value);
  }
}

// Reads any type of data from a specific location in the FRAM memory.
// Takes three arguments -
//    1. address --> Any address from 0 to maxAddress
//    2. T& value --> Variable to return data into
//    2. fastRead --> defaults to false - executes _beginFastRead() if set to true
template <class T> bool SPIFRAM::readAnything(uint32_t address, T& value) {
  if (!_prep(READDATA, address, sizeof(value)))
    return false;

  uint8_t* p = (uint8_t*)(void*)&value;
  _beginSPI(READDATA);
  for (uint16_t i = 0; i < sizeof(value); i++) {
    *p++ =_nextByte();
  }
  _endSPI();
  return true;
}

// Private template to check for errors in writing to FRAM memory
template <class T> bool SPIFRAM::_writeErrorCheck(uint32_t address, const T& value) {
  uint32_t _size = sizeof(value);
  const byte* p = (const byte*)(const void*)&value;
  _currentAddress = address;
  _beginSPI(READDATA);
  for(uint16_t i = 0; i < _size;i++)
  {
      if(*p++ != _nextByte(READDATA))
      {
        return false;
      }
  }
  _endSPI();
  return true;
}

#endif // _SPIFRAM_H_
