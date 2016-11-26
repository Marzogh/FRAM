/*
  |~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|
  |                                                             readWriteString.ino                                                               |
  |                                                               SPIFRAM library                                                                 |
  |                                                                   v 1.0.0                                                                     |
  |~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|
  |                                                                    Marzogh                                                                    |
  |                                                                  16.11.2016                                                                   |
  |~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|
  |                                                                                                                                               |
  |                        This program shows the method of reading a string from the console and saving it to fram memory                       |
  |                                                                                                                                               |
  |~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|
*/
#include<SPIFRAM.h>
#define CHIPSIZE KB64
uint16_t strSize;
uint32_t strAddr;

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
// Required for Serial on Zero based boards
#define Serial SERIAL_PORT_USBVIRTUAL
#endif

#if defined (SIMBLEE)
#define BAUD_RATE 250000
#define RANDPIN 1
#else
#define BAUD_RATE 115200
#define RANDPIN A0
#endif

SPIFRAM fram;

bool readSerialStr(String &inputStr);

void setup() {
  Serial.begin(BAUD_RATE);
#if defined (ARDUINO_SAMD_ZERO) || (__AVR_ATmega32U4__)
  while (!Serial) ; // Wait for Serial monitor to open
#endif

  fram.begin(CHIPSIZE);

#if defined (ARDUINO_ARCH_ESP32)
  randomSeed(65535537);
#else
  randomSeed(analogRead(RANDPIN));
#endif
  uint32_t _cap = fram.getCapacity();
  strAddr = random(0, _cap);
  String inputString = "This is a test String";
  fram.writeStr(strAddr, inputString);
  Serial.print(F("Written string: "));
  Serial.print(inputString);
  Serial.print(F(" to address "));
  Serial.println(strAddr);
  String outputString = "";
  fram.readStr(strAddr, outputString);
  Serial.print(F("Read string: "));
  Serial.print(outputString);
  Serial.print(F(" from address "));
  Serial.print(strAddr);

  while (!fram.eraseSector(strAddr, strSize));
}

void loop() {

}

//Reads a string from Serial
bool readSerialStr(String &inputStr) {
  if (!Serial)
    Serial.begin(115200);
  while (Serial.available()) {
    inputStr = Serial.readStringUntil('\n');
    Serial.println(inputStr);
    return true;
  }
  return false;
}
