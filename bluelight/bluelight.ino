/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "bluelight.h"
// #include "BluefruitConfig.h"

#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

#include <Adafruit_NeoPixel.h>

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    PIN                       Which pin on the Arduino is connected to the NeoPixels?
    NUMPIXELS                 How many NeoPixels are attached to the Arduino?
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
    #define PIN                     6
    #define NUMPIXELS               60
/*=========================================================================*/
Adafruit_NeoPixel pixel = Adafruit_NeoPixel(NUMPIXELS, PIN);

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(Serial1, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetParser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
//additional variables

//Color
uint8_t red = 100;
uint8_t green = 100;
uint8_t blue = 100;
uint8_t animationState = 1;
uint8_t mode = 1;
bool debug = false;

int pos = 0, dir = 1; // Position, direction of "eye" for larson scanner animation

void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);

  // turn off neopixel
  pixel.begin(); // This initializes the NeoPixel library.
  for(uint8_t i=0; i<NUMPIXELS; i++) {
    pixel.setPixelColor(i, pixel.Color(0,0,0)); // off
  }
  colorWipe(pixel.Color(100, 100, 100), 20); // do a quick colorWipe to show that the pixels are all working, even before Bluefruit connection established
  colorWipe(pixel.Color(0, 0, 0), 20);
  pixel.show();

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Command Mode Example"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  // Enable/Disable command echo from Bluefruit
  if(debug) {
    ble.echo(true);
  } else {
    ble.echo(false);
  }

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();

  // Enable/Disable Bluefruit verbosity
  if (debug) {
    ble.verbose(true);
  } else {
    ble.verbose(false);
  }

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));
  }
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  Serial.println("loop() top");
  Serial.print("animationState = ");
  Serial.println(animationState);
  // Check for user input
  char inputs[BUFSIZE+1];
/**
  if ( getUserInput(inputs, BUFSIZE) )
  {
    // Send characters to Bluefruit
    Serial.print("[Send] ");
    Serial.println(inputs);

    ble.print("AT+BLEUARTTX=");
    ble.println(inputs);

    // check response stastus
    if (! ble.waitForOK() ) {
      Serial.println(F("Failed to send?"));
    }
  }
**/

  // Check for incoming characters from Bluefruit
  ble.println("AT+BLEUARTRX");
  ble.readline();
  // Control for BLE buffer debug
  if (debug) {
    Serial.print("Buffer = ");
    Serial.println(ble.buffer);
  }
  if (strcmp(ble.buffer, "OK") == 0) {
    // no data
    return;
  }
// System control messages
// @todo - add a check for runtime setting of debug
  if (strcmp(ble.buffer, "0") == 0) {
      Serial.println("Setting animationState = 0");
      animationState = 0;
  }
  if (strcmp(ble.buffer, "1") == 0) {
      Serial.println("Setting animationState = 1");
      animationState = 1;
  }
  if (strcmp(ble.buffer, "2") == 0) {
      Serial.println("Setting animationState = 2");
      animationState = 2;
  }
  if (strcmp(ble.buffer, "3") == 0) {
      Serial.println("Setting animationState = 3");
      animationState = 3;
  }
  if (strcmp(ble.buffer, "4") == 0) {
      Serial.println("Setting animationState = 4");
      animationState = 4;
  }
  if (strcmp(ble.buffer, "5") == 0) {
      Serial.println("Setting animationState = 5");
      animationState = 5;
  }
  // Lighting control
  if (strcmp(ble.buffer, "10") == 0) {
      Serial.println("Setting animationState = 10");
      animationState = 10;
  }
  if (strcmp(ble.buffer, "11") == 0) {
      Serial.println("Setting animationState = 11");
      animationState = 11;
  }
  if (strcmp(ble.buffer, "12") == 0) {
      Serial.println("Setting animationState = 12");
      animationState = 12;
  }
  if (strcmp(ble.buffer, "13") == 0) {
      Serial.println("Setting animationState = 13");
      animationState = 13;
  }

  Serial.print("animationState = ");
  Serial.println(animationState);
  if (animationState == 0){
    pixel.begin(); // This initializes the NeoPixel library.
    for(uint8_t i=0; i<NUMPIXELS; i++) {
      pixel.setPixelColor(i, pixel.Color(0,0,0)); // off
    }
   }

   if (animationState == 1){
     colorWipe(pixel.Color(100, 100, 100), 20); // do a quick colorWipe to show that the pixels are all working, even before Bluefruit connection established
     colorWipe(pixel.Color(0, 0, 0), 20);
     pixel.show();
    }

  if (animationState == 10){
    for(uint16_t i=0; i<pixel.numPixels(); i++) { //clear all pixels before displaying new animation
          pixel.setPixelColor(i, pixel.Color(0,0,0));
        }
     flashRandom(5,random(10,30));
     pixel.show(); // This sends the updated pixel color to the hardware.
   }

  if (animationState == 11){
    for(uint16_t i=0; i<pixel.numPixels(); i++) { //clear all pixels before displaying new animation
          pixel.setPixelColor(i, pixel.Color(0,0,0));
        }
    colorWipe(pixel.Color(red, green, blue), 20);
    pixel.show(); // This sends the updated pixel color to the hardware.
    colorWipe(pixel.Color(0, 0, 0), 20);
    pixel.show();
  }

  if (animationState == 12){
    for(uint16_t i=0; i<pixel.numPixels(); i++) { //clear all pixels before displaying new animation
          pixel.setPixelColor(i, pixel.Color(0,0,0));
        }
    larsonScanner(30); // larsonScanner is set to red and does not take color input.
    pixel.show(); // This sends the updated pixel color to the hardware.
  }

  if (animationState == 13){
    for(uint16_t i=0; i<pixel.numPixels(); i++) { //clear all pixels before displaying new animation
          pixel.setPixelColor(i, pixel.Color(0,0,0));
        }
    rainbowCycle(20);
    pixel.show(); // This sends the updated pixel color to the hardware.
  }

  Serial.println("loop() bottom");
  Serial.print("animationState = ");
  Serial.println(animationState);
  ble.waitForOK();
}
// Functions
// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<pixel.numPixels(); i++) {
      pixel.setPixelColor(i, c);
      pixel.show();
      delay(wait);
  }
}

void larsonScanner(uint8_t wait){
   int j;

 for(uint16_t i=0; i<pixel.numPixels()+5; i++) {
  // Draw 5 pixels centered on pos.  setPixelColor() will clip any
  // pixels off the ends of the strip, we don't need to watch for that.
  pixel.setPixelColor(pos - 2, 0x100000); // Dark red
  pixel.setPixelColor(pos - 1, 0x800000); // Medium red
  pixel.setPixelColor(pos    , 0xFF3000); // Center pixel is brightest
  pixel.setPixelColor(pos + 1, 0x800000); // Medium red
  pixel.setPixelColor(pos + 2, 0x100000); // Dark red

  pixel.show();
  delay(wait);

  // Rather than being sneaky and erasing just the tail pixel,
  // it's easier to erase it all and draw a new one next time.
  for(j=-2; j<= 2; j++) pixel.setPixelColor(pos+j, 0);

  // Bounce off ends of strip
  pos += dir;
  if(pos < 0) {
    pos = 1;
    dir = -dir;
  } else if(pos >= pixel.numPixels()) {
    pos = pixel.numPixels() - 2;
    dir = -dir;
  }
 }
}

void flashRandom(int wait, uint8_t howmany) {
 randomSeed(analogRead(0));
  for(uint16_t i=0; i<howmany; i++) {
    // get a random pixel from the list
    int j = random(pixel.numPixels());

    // now we will 'fade' it in 5 steps
    for (int x=0; x < 5; x++) {
      int r = red * (x+1); r /= 5;
      int g = green * (x+1); g /= 5;
      int b = blue * (x+1); b /= 5;

      pixel.setPixelColor(j, pixel.Color(r, g, b));
      pixel.show();
      delay(wait);
    }
    // & fade out in 5 steps
    for (int x=5; x >= 0; x--) {
      int r = red * x; r /= 5;
      int g = green * x; g /= 5;
      int b = blue * x; b /= 5;

      pixel.setPixelColor(j, pixel.Color(r, g, b));
      pixel.show();
      delay(wait);
    }
  }
  // LEDs will be off when done (they are faded to 0)
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<pixel.numPixels(); i++) {
      pixel.setPixelColor(i, Wheel((i+j) & 255));
    }
    pixel.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< pixel.numPixels(); i++) {
      pixel.setPixelColor(i, Wheel(((i * 256 / pixel.numPixels()) + j) & 255));
    }
    pixel.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (int i=0; i < pixel.numPixels(); i=i+3) {
        pixel.setPixelColor(i+q, c);    //turn every third pixel on
      }
      pixel.show();

      delay(wait);

      for (int i=0; i < pixel.numPixels(); i=i+3) {
        pixel.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (int i=0; i < pixel.numPixels(); i=i+3) {
        pixel.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      pixel.show();

      delay(wait);

      for (int i=0; i < pixel.numPixels(); i=i+3) {
        pixel.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return pixel.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return pixel.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixel.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
/**************************************************************************/
/*!
    @brief  Checks for user input (via the Serial Monitor)
*/
/**************************************************************************/
bool getUserInput(char buffer[], uint8_t maxSize)
{
  // timeout in 100 milliseconds
  TimeoutTimer timeout(100);

  memset(buffer, 0, maxSize);
  while( (!Serial.available()) && !timeout.expired() ) { delay(1); }

  if ( timeout.expired() ) return false;

  delay(2);
  uint8_t count=0;
  do
  {
    count += Serial.readBytes(buffer+count, maxSize);
    delay(2);
  } while( (count < maxSize) && (Serial.available()) );

  return true;
}
