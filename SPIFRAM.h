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

#ifndef SPIFRAM_H
#define SPIFRAM_H

#include <Arduino.h>

class SPIFRAM {
public:
  //----------------------------------------------Constructor-----------------------------------------------//
  SPIFRAM(uint8_t cs = SS, bool overflow = true);
  //----------------------------------------Initial / Chip Functions----------------------------------------//
  void     begin(void);
  uint32_t getJEDECID();
  bool     getAddress(uint16_t size, uint16_t &page_number, uint8_t &offset);
  uint32_t getAddress(uint16_t size);
  uint16_t getChipName();
  uint16_t sizeofStr(String &inputStr);
  uint32_t getCapacity();
  uint32_t getMaxPage();
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
  bool     erase(uint32_t address, uint32_t size);
  bool     eraseSector(uint32_t address);
  bool     eraseBlock32K(uint32_t address);
  bool     eraseBlock64K(uint32_t address);
  bool     eraseChip(void);
  //--------------------------------------------Private functions-------------------------------------------//
private:
  void     _troubleshoot(uint8_t error);
  void     _cmd(uint8_t cmd, bool _continue = true);
  void     _endProcess(void);
  void     _errorCodeCheck(void);
  void     _beginRead(uint32_t address);
  bool     _notPrevWritten(uint32_t address, uint32_t size = 1);
  bool     _addressCheck(uint32_t address, uint32_t size = 1);
  bool     _beginWrite(uint32_t address);
  bool     _writeNextByte(uint8_t c, bool _continue = true);
  bool     _writeEnable(uint32_t timeout = 10L);
  bool     _writeDisable(void);
  bool     _getJedecId(uint8_t *b1, uint8_t *b2, uint8_t *b3, uint8_t *b4);
  bool     _chipID(void);
  uint32_t _prepRead(uint32_t address, uint32_t size);
  bool _prepWrite(uint32_t address, uint32_t size);
  uint8_t  _readStat(void);
  uint8_t  _readNextByte(bool _continue = true);
  template <class T> bool _writeErrorCheck(uint32_t address, const T& value);
  //-------------------------------------------Private variables------------------------------------------//
  bool        pageOverflow;
  volatile uint8_t *cs_port;
  uint8_t     cs_mask, csPin, errorcode, state;
  uint16_t    name;
  uint32_t    capacity;
  uint32_t    currentAddress = 1;
  const uint8_t density[11]   = {0x01, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x43};
  const uint32_t memSize[11]  = {16L * 1024L, 128L * 1024L, 256L * 1024L, 512L * 1024L, 1L * 1024L * 1024L,
                                2L * 1024L * 1024L, 4L * 1024L * 1024L, 8L * 1024L * 1024L, 16L * 1024L * 1024L,
                                32L * 1024L * 1024L, 8L * 1024L * 1024L};
  const uint16_t chipName[11] = {16, 10, 20, 40, 80, 16, 32, 64, 128, 256, 64};
};


  //--------------------------------------------Templates-------------------------------------------//

// Writes any type of data to a specific location in the flash memory.
// Has two variants:
//  A. Takes two arguments -
//    1. address --> Any address from 0 to maxAddress
//    2. T& value --> Variable to return data into
//    4. errorCheck --> Turned on by default. Checks for writing errors
//  B. Takes three arguments -
//    1. page --> Any page number from 0 to maxPage
//    2. offset --> Any offset within the page - from 0 to 255
//    3. const T& value --> Variable with the data to be written
//    4. errorCheck --> Turned on by default. Checks for writing errors
// WARNING: You can only write to previously erased memory locations (see datasheet).
//      Use the eraseSector()/eraseBlock32K/eraseBlock64K commands to first clear memory (write 0xFFs)
// Variant A
template <class T> bool SPIFRAM::writeAnything(uint32_t address, const T& value, bool errorCheck) {
  if (!_prepWrite(address))
    return false;
  else {
    const byte* p = (const byte*)(const void*)&value;
    _beginWrite(address);
    for (uint16_t i = 0; i < sizeof(value); i++) {
      #if defined (__arm__) && defined (__SAM3X8E__)
        if (i == sizeof(value)-1)
          _writeNextByte(*p++, false);
      #endif
        _writeNextByte(*p++);
    }
    _endProcess();
  }

  if (!errorCheck)
    return true;
  else
    return _writeErrorCheck(address, value);
}
// Variant B
template <class T> bool SPIFRAM::writeAnything(uint16_t page_number, uint8_t offset, const T& value, bool errorCheck) {
  uint32_t address = _getAddress(page_number, offset);
  return writeAnything(address, value, errorCheck);
}

// Reads any type of data from a specific location in the flash memory.
// Has two variants:
//  A. Takes two arguments -
//    1. address --> Any address from 0 to maxAddress
//    2. T& value --> Variable to return data into
//    2. fastRead --> defaults to false - executes _beginFastRead() if set to true
//  B. Takes three arguments -
//    1. page --> Any page number from 0 to maxPage
//    2. offset --> Any offset within the page - from 0 to 255
//    3. T& value --> Variable to return data into
//    3. fastRead --> defaults to false - executes _beginFastRead() if set to true
// Variant A
template <class T> bool SPIFRAM::readAnything(uint32_t address, T& value, bool fastRead) {
  if (!_prepRead(address))
    return false;

    byte* p = (byte*)(void*)&value;
    if(!fastRead)
      _beginRead(address);
    else
      _beginFastRead(address);
    for (uint16_t i = 0; i < sizeof(value); i++) {
      #if defined (__arm__) && defined (__SAM3X8E__)
        if (i == sizeof(value)-1)
          *p++ = _readNextByte(false);
      #endif
      *p++ =_readNextByte();
    }
    _endProcess();
    return true;
}
// Variant B
template <class T> bool SPIFRAM::readAnything(uint16_t page_number, uint8_t offset, T& value, bool fastRead)
{
  uint32_t address = _getAddress(page_number, offset);
  return readAnything(address, value, fastRead);
}

// Private template to check for errors in writing to flash memory
template <class T> bool SPIFRAM::_writeErrorCheck(uint32_t address, const T& value) {
/*if (address != 0x00){
  if (!_prepRead(address)) {
    return false;
  }
}
else if (address == 0x00 && !_notBusy()) {
  return false;
}
*/
if (!_prepRead(address) && !_notBusy()) {
  return false;
}

  const byte* p = (const byte*)(const void*)&value;
  _beginRead(address);
  for(uint16_t i = 0; i < sizeof(value);i++)
  {
    if(*p++ != _readNextByte())
    {
      return false;
    }
    /*#if defined (__arm__) && defined (__SAM3X8E__)
      if (i == sizeof(value)-1) {
        if (*p++ != _readNextByte(false))
          return false;
        else
          return true;
      }
      else
        if (*p++ != _readNextByte())
          return false;
    #elif defined (__AVR__)
    if (*p++ != _readNextByte())
      return false;
    #endif*/
  }
  _endProcess();
  return true;
}


#endif // _SPIFRAM_H_
