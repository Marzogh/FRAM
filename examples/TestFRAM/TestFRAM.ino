/*
  ----------------------------------------------------------------------------------------------------------------------------------
  |                                                            Winbond fram                                                         |
  |                                                      SPIFRAM library test v 0.0.1b                                               |
  |----------------------------------------------------------------------------------------------------------------------------------|
  |                                                                Marzogh                                                           |
  |                                                              16.11.2016                                                          |
  |----------------------------------------------------------------------------------------------------------------------------------|
  |                                     (Please make sure your Serial monitor is set to 'No Line Ending')                            |
  |                                     *****************************************************************                            |
  |                                                                                                                                  |
  |                     # Please pick from the following commands and type the command number into the Serial console #              |
  |    For example - to write a byte of data, you would have to use the write_byte function - so type '3' into the serial console.   |
  |                                                    --------------------------------                                              |
  |                                                                                                                                  |
  |  1. getID                                                                                                                        |
  |   '1' gets the JEDEC ID of the chip                                                                                              |
  |                                                                                                                                  |
  |  2. writeByte [address] [byte]                                                                                                   |
  |   '2' followed by '100' and then by '224' writes the byte 224 to address 100                                                     |
  |                                                                                                                                  |
  |  3. readByte [address]                                                                                                           |
  |   '3' followed by '100' returns the byte from address 100                                                                        |
  |                                                                                                                                  |
  |  4. writeWord [address]                                                                                                          |
  |   '4' followed by '55' and then by '633' writes the int 633 to address 55                                                        |
  |                                                                                                                                  |
  |  5. readWord [address]                                                                                                           |
  |   '5' followed by '200'returns the int from address 200                                                                          |
  |                                                                                                                                  |
  |  6. writeStr [address] [inputString]                                                                                             |
  |   '6' followed by '345'and then by 'Test String 1!' writes the String 'Test String 1! to address 345                             |
  |                                                                                                                                  |
  |  7. readStr [address] [outputString]                                                                                             |
  |   '7' followed by '2050' reads the String from address 2050 into the outputString                                                |
  |                                                                                                                                  |
  |  8. printEntireChip                                                                                                              |
  |   '8' reads all addresss and outputs the data to the serial console                                                              |
  |   This function is to extract data from a fram chip onto a computer as a text file.                                              |
  |   Refer to 'Read me.md' in the library for details.                                                                              |
  |                                                                                                                                  |
  |  9. Erase a sector of user defined size                                                                                          |
  |   '9'  followed by 540 followed by 800 erases an 800 Byte block starting at address 540                                          |
  |                                                                                                                                  |
  |  10. Erase Chip                                                                                                                  |
  |   '10' erases the entire chip                                                                                                    |
  |                                                                                                                                  |
  ^----------------------------------------------------------------------------------------------------------------------------------^
*/



#include<SPIFRAM.h>
#define CHIPSIZE KB64
uint8_t addressBuffer[256];
String serialCommand;
char printBuffer[128];
uint16_t address;
uint8_t offset, dataByte;
uint16_t dataInt;
String inputString, outputString;

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
// Required for Serial on Zero based boards
#define Serial SERIAL_PORT_USBVIRTUAL
#endif

#if defined (SIMBLEE)
#define BAUD_RATE 250000
#else
#define BAUD_RATE 115200
#endif

SPIFRAM fram;

void setup() {
  delay(10);
  Serial.begin(BAUD_RATE);
  Serial.print(F("Initialising fram memory"));
  for (int i = 0; i < 10; ++i)
  {
    Serial.print(F("."));
  }
  Serial.println();
  fram.begin(CHIPSIZE);
  Serial.println();
  Serial.println();
  commandList();
}

void loop() {
  while (Serial.available() > 0) {
    uint8_t commandNo = Serial.parseInt();
    if (commandNo == 0) {
      commandList();
    }
    else if (commandNo == 1) {
      printLine();
      Serial.println(F("                                                      Function 1 : Get JEDEC ID"));
      printLine();
      printLine();
      uint8_t b1, b2, b3;
      uint32_t JEDEC = fram.getJEDECID();
      //uint16_t ManID = fram.getManID();
      b1 = (JEDEC >> 16);
      b2 = (JEDEC >> 8);
      b3 = (JEDEC >> 0);
      clearprintBuffer();
      sprintf(printBuffer, "Manufacturer ID: %02xh\nMemory Type: %02xh\nCapacity: %02xh", b1, b2, b3);
      Serial.println(printBuffer);
      clearprintBuffer();
      sprintf(printBuffer, "JEDEC ID: %04lxh", JEDEC);
      Serial.println(printBuffer);
      printLine();
      printNextCMD();
    }
    else if (commandNo == 2) {
      printLine();
      Serial.println(F("                                                       Function 2 : Write Byte"));
      printSplash();
      printLine();
      Serial.print(F("Please enter the address you wish to write to: "));
      while (!Serial.available()) {
      }
      address = Serial.parseInt();
      Serial.println(address);
      Serial.print(F("Please enter the value of the byte (0-255) you wish to save: "));
      while (!Serial.available()) {
      }
      dataByte = Serial.parseInt();
      Serial.println(dataByte);
      if (fram.writeByte(address, dataByte)) {
        clearprintBuffer();
        sprintf(printBuffer, "%d has been written to address %d", dataByte, address);
        Serial.println(printBuffer);
      }
      else {
        writeFail();
      }
      printLine();
      printNextCMD();
    }
    else if (commandNo == 3) {
      printLine();
      Serial.println(F("                                                       Function 3 : Read Byte"));
      printSplash();
      printLine();
      Serial.print(F("Please enter the address that the byte you wish to read is on: "));
      while (!Serial.available()) {
      }
      address = Serial.parseInt();
      Serial.println(address);
      clearprintBuffer();
      sprintf(printBuffer, "The byte at address %d is: ", address);
      Serial.print(printBuffer);
      Serial.println(fram.readByte(address));
      printLine();
      printNextCMD();
    }
    else if (commandNo == 4) {
      printLine();
      Serial.println(F("                                                       Function 4 : Write Word"));
      printSplash();
      printLine();
      Serial.print(F("Please enter the address you wish to write to: "));
      while (!Serial.available()) {
      }
      address = Serial.parseInt();
      Serial.println(address);
      Serial.print(F("Please enter the value of the word (>255) you wish to save: "));
      while (!Serial.available()) {
      }
      dataInt = Serial.parseInt();
      Serial.println(dataInt);
      if (fram.writeWord(address, dataInt)) {
        clearprintBuffer();
        sprintf(printBuffer, "%d has been written to position %d on address %d", dataInt, address);
        Serial.println(printBuffer);
      }
      else {
        writeFail();
      }
      printLine();
      printNextCMD();
    }
    else if (commandNo == 5) {
      printLine();
      Serial.println(F("                                                       Function 5 : Read Word"));
      printSplash();
      printLine();
      Serial.print(F("Please enter the address that the byte you wish to read is on: "));
      while (!Serial.available()) {
      }
      address = Serial.parseInt();
      Serial.println(address);
      clearprintBuffer();
      sprintf(printBuffer, "The unsigned int at address %d is: ", address);
      Serial.print(printBuffer);
      Serial.println(fram.readWord(address));
      printLine();
      printNextCMD();
    }
    else if (commandNo == 6) {
      printLine();
      Serial.println(F("                                                      Function 6 : Write String"));
      printSplash();
      printLine();
      Serial.println(F("This function will write a String of your choice to the address selected."));
      Serial.print(F("Please enter the address you wish to write to: "));
      while (!Serial.available()) {
      }
      address = Serial.parseInt();
      Serial.println(address);
      Serial.println(F("Please enter the String you wish to save: "));
      while (!Serial.available()) {
      }
      readSerialStr(inputString);
      if (fram.writeStr(address,  inputString)) {
        clearprintBuffer();
        Serial.print(F("String '"));
        Serial.print(inputString);
        sprintf(printBuffer, "' has been written to address %d", address);
        Serial.println(printBuffer);
      }
      else {
        writeFail();
      }
      printLine();
      printNextCMD();
    }
    else if (commandNo == 7) {
      printLine();
      Serial.println(F("                                                      Function 7 : Read String"));
      printSplash();
      printLine();
      Serial.print(F("Please enter the address the String you wish to read is on: "));
      while (!Serial.available()) {
      }
      address = Serial.parseInt();
      Serial.println(address);
      clearprintBuffer();
      sprintf(printBuffer, "The String at address %d is: ", address);
      Serial.print(printBuffer);
      fram.readStr(address, outputString);
      Serial.println(outputString);
      printLine();
      printNextCMD();
    }
    else if (commandNo == 8) {
      printLine();
      Serial.println(F("                                                     Function 8 : Read Entire chip"));
      printSplash();
      printLine();
      Serial.println(F("This function will read the entire FRAM memory."));
      Serial.println(F("This will take a long time and might result in memory issues. Do you wish to continue? (Y/N)"));
      char c;
      while (!Serial.available()) {
      }
      c = (char)Serial.read();
      if (c == 'Y' || c == 'y') {
        printOutputChoice();
        while (!Serial.available()) {
        }
        uint8_t outputType = Serial.parseInt();
        Serial.println(outputType);
        printAllPages(outputType);
      }
      printLine();
      printNextCMD();
    }
    else if (commandNo == 9) {
      printLine();
      Serial.println(F("                                                       Function 9 : Erase Sector"));
      printSplash();
      printLine();
      Serial.println(F("This function will erase a sector of user defined size."));
      Serial.print(F("Please enter the address you wish to erase: "));
      while (!Serial.available()) {
      }
      address = Serial.parseInt();
      Serial.println(address);
      Serial.print(F("Please enter the size of the sector (in bytes) you wish to erase: "));
      while (!Serial.available()) {
      }
      uint32_t size = Serial.parseInt();
      Serial.println(size);
      fram.eraseSector(address, size);
      clearprintBuffer();
      sprintf(printBuffer, "A %l B sector starting at address %d has been erased", size, address);
      Serial.println(printBuffer);
      printReadChoice();
      while (!Serial.available()) {
      }
      uint8_t choice = Serial.parseInt();
      Serial.println(choice);
      if (choice == 1) {
        printOutputChoice();
        while (!Serial.available()) {
        }
        uint8_t outputType = Serial.parseInt();
        Serial.println(outputType);
        printPage(address, outputType);
      }
      printLine();
      printNextCMD();
    }
    else if (commandNo == 10) {
      printLine();
      Serial.println(F("                                                      Function 11 : Erase Chip"));
      printSplash();
      printLine();
      Serial.println(F("This function will erase the entire FRAM memory."));
      Serial.println(F("Do you wish to continue? (Y/N)"));
      char c;
      while (!Serial.available()) {
      }
      c = (char)Serial.read();
      if (c == 'Y' || c == 'y') {
        if (fram.eraseChip())
          Serial.println(F("Chip erased"));
        else
          Serial.println(F("Error erasing chip"));
      }
      printLine();
      printNextCMD();
    }
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Functions~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

void clearprintBuffer()
{
  for (uint8_t i = 0; i < 128; i++) {
    printBuffer[i] = 0;
  }
}

//Reads a string from Serial
bool readSerialStr(String &inputStr) {
  if (!Serial)
    Serial.begin(115200);
  while (Serial.available()) {
    inputStr = Serial.readStringUntil('\n');
    return true;
  }
  return false;
}

//Prints hex/dec formatted data from address reads - for debugging
void _printPageBytes(uint8_t *data_buffer, uint8_t outputType) {
  char buffer[10];
  for (int a = 0; a < 16; ++a) {
    for (int b = 0; b < 16; ++b) {
      if (outputType == 1) {
        sprintf(buffer, "%02x", data_buffer[a * 16 + b]);
        Serial.print(buffer);
      }
      else if (outputType == 2) {
        uint8_t x = data_buffer[a * 16 + b];
        if (x < 10) Serial.print("0");
        if (x < 100) Serial.print("0");
        Serial.print(x);
        Serial.print(',');
      }
    }
    Serial.println();
  }
}

//Reads a page of data and prints it to Serial stream. Make sure the sizeOf(uint8_t data_buffer[]) == 256.
void printPage(uint16_t page_number, uint8_t outputType) {
  if (!Serial)
    Serial.begin(115200);

  char buffer[24];
  sprintf(buffer, "Reading page (%04x)", page_number);
  Serial.println(buffer);

  uint8_t data_buffer[256];
  fram.readByteArray(address, &data_buffer[0], 256);
  _printPageBytes(data_buffer, outputType);
}

//Reads all addresss on fram chip and dumps it to Serial stream.
//This function is useful when extracting data from a fram chip onto a computer as a text file.
void printAllPages(uint8_t outputType) {
  if (!Serial)
    Serial.begin(115200);

  Serial.println("Reading entire chip");
  uint8_t data_buffer[256];

  uint32_t maxaddress = fram.getCapacity();
  for (uint32_t a = 0; a < maxaddress; a++) {
    for (uint16_t j = 0; j <256; j++) {
      fram.readByteArray(a+j, &data_buffer[0], 256);
    }
    _printPageBytes(data_buffer, outputType);
    a+=256L;
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Print commands~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

void printLine()
{
  Serial.println(F("----------------------------------------------------------------------------------------------------------------------------------"));
}

void printSplash()
{
  Serial.println(F("                                                        SPIFRAM library test                                                     "));
}

void printNextCMD()
{
  Serial.println(F("Please type the next command. Type 0 to get the list of commands"));
}

void printOutputChoice()
{
  Serial.print("Would you like your output in decimal or hexadecimal? Please indicate with '1' for HEX or '2' for DEC: ");
}

void printReadChoice()
{
  Serial.print("Type 1 to read the address you have just modified. Type 0 to continue: ");
}

void writeSuccess()
{
  Serial.println("Data write successful");
}

void writeFail()
{
  Serial.println("Data write failed");
}
