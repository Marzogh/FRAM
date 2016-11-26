/*
  |~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|
  |                                                               Struct_writer.ino                                                               |
  |                                                               SPIFRAM library                                                                 |
  |                                                                   v 1.0.0                                                                     |
  |~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|
  |                                                                    Marzogh                                                                    |
  |                                                                  16.11.2016                                                                   |
  |~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|
  |                                                                                                                                               |
  |                        This program writes a struct to a random location on your fram memory chip and reads it back.                         |
  |        Uncomment #define SENSOR below to get real world readings. Real world readings require a Light dependant resistor hooked up to A0.     |
  |                   For information on how to hook up an LDR to an Arduino, please refer to Adafruit's excellent tutorial at                    |
  |                                          https://learn.adafruit.com/photocells/using-a-photocell                                              |
  |                                                                                                                                               |
  |~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|
*/

#include<SPIFRAM.h>

#define CHIPSIZE KB64

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
// Required for Serial on Zero based boards
#define Serial SERIAL_PORT_USBVIRTUAL
#endif

#if defined (SIMBLEE)
#define BAUD_RATE 250000
#else
#define BAUD_RATE 115200
#endif

/*
   Uncomment the #define below if you would like real world readings.
   For real world readings, hook up a light dependant resistor to A1.

*/
//#define SENSOR
#if defined (SIMBLEE)
#define BAUD_RATE 250000
#define LDR 1
#else
#define BAUD_RATE 115200
#define LDR A0
#endif



SPIFRAM fram;


struct Configuration {
  float lux;
  float vOut;                   // Voltage ouput from potential divider to Anolg input
  float RLDR;                   // Resistance calculation of potential divider with LDR
  bool light;
  uint8_t adc;
};
Configuration configuration;

void setup() {
  Serial.begin(BAUD_RATE);
#if defined (ARDUINO_SAMD_ZERO) || (__AVR_ATmega32U4__)
  while (!Serial) ; // Wait for Serial monitor to open
#endif
#if defined (ARDUINO_ARCH_ESP32)
  randomSeed(65535537);
#else
  randomSeed(analogRead(LDR));
#endif
  Serial.print(F("Initialising fram memory"));
  for (int i = 0; i < 10; ++i)
  {
    Serial.print(F("."));
  }
  Serial.println();
  Serial.println();
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  fram.begin(CHIPSIZE);
  fram.eraseChip();
  Serial.println("eraseDone");
  delay(2000);
  uint32_t _addr = random(0, fram.getCapacity());

#ifndef SENSOR
  configuration.lux = 98.43;
  configuration.vOut = 4.84;
  configuration.RLDR = 889.32;
  configuration.light = true;
  configuration.adc = 5;
#endif

#ifdef SENSOR
  readLDR();
#endif

  if (fram.writeAnything(_addr, configuration)) {
    Serial.println ("Data write successful");
    Serial.println(configuration.lux);
    Serial.println(configuration.vOut);
    Serial.println(configuration.RLDR);
    Serial.println(configuration.light);
    Serial.println(configuration.adc);
  }
  else {
    Serial.println ("Data write failed");
  }

    Serial.println("Saved!");
    configuration.lux = 0;
    configuration.vOut = 0;
    configuration.RLDR = 0;
    configuration.light = 0;
    configuration.adc = 0;
    Serial.println();
    Serial.println("Local values set to 0");
    Serial.println(configuration.lux);
    Serial.println(configuration.vOut);
    Serial.println(configuration.RLDR);
    Serial.println(configuration.light);
    Serial.println(configuration.adc);
    Serial.println();
    fram.readAnything(_addr, configuration);
    fram.eraseSector(_addr, sizeof(configuration));

    Serial.println("After reading");
    Serial.println(configuration.lux);
    Serial.println(configuration.vOut);
    Serial.println(configuration.RLDR);
    Serial.println(configuration.light);
    Serial.println(configuration.adc);
}

void loop() {
}

#ifdef SENSOR
void readLDR()
{
  configuration.adc = analogRead(LDR);
  configuration.vOut = (configuration.adc * 0.0048828125);                       // vOut = Output voltage from potential Divider. [vOut = ADC * (Vin / 1024)]
  configuration.RLDR = (10.0 * (5 - configuration.vOut)) / configuration.vOut;   // Equation to calculate Resistance of LDR, [R-LDR =(R1 (Vin - vOut))/ vOut]. R1 is in KOhms
  // R1 = 10 KOhms , Vin = 5.0 Vdc.
  configuration.lux = (500 / configuration.RLDR);
}
#endif
