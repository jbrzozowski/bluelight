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
#include <Adafruit_NeoPixel.h>
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

// Using strip instead, below is for NeoPixel pixel object
// Adafruit_NeoPixel pixel = Adafruit_NeoPixel(N_PIXELS, PIN);

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
// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
/**
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
**/

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
uint8_t animationState = 0;
uint8_t mode = 1;
bool debug = false;

int pos = 0, dir = 1; // Position, direction of "eye" for larson scanner animation

// Added to support listen() via the microphone
byte
  peak      = 0,      // Used for falling dot
  dotCount  = 0,      // Frame counter for delaying dot-falling speed
  volCount  = 0;      // Frame counter for storing past volume data
int
  vol[SAMPLES],       // Collection of prior volume samples
  lvl       = 10,      // Current "dampened" audio level
  minLvlAvg = 0,      // For dynamic adjustment of graph low & high
  maxLvlAvg = 512;

// @todo - needs to be fixed
// Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
// Adafruit_NeoPixel strip = Adafruit_NeoPixel(150, PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_PIXELS, LED_PIN, NEO_GRB);

void setup(void)
{
  Serial.println("setup -> start");
  // Configure pin for onboard LED blinking
  pinMode(BOARD_PIN, OUTPUT);
  // Setup for RGB LED strip
  // Commenting out since using Neopixel strip
  /**
  pinMode(REDPIN, OUTPUT);
  pinMode(GREENPIN, OUTPUT);
  pinMode(BLUEPIN, OUTPUT);
  **/
  // Comment out to prevent the need to connect serially
  /**
  while (!Serial);  // required for Flora & Micro
  delay(500);
  **/

  // turn off neopixel
  /*
  pixel.begin(); // This initializes the NeoPixel library.
  for(uint8_t i=0; i<N_PIXELS; i++) {
    pixel.setPixelColor(i, pixel.Color(0,0,0)); // off
  }
  colorWipe(pixel.Color(100, 100, 100), 20); // do a quick colorWipe to show that the pixels are all working, even before Bluefruit connection established
  colorWipe(pixel.Color(0, 0, 0), 20);
  pixel.show();
  **/

  strip.begin();
  // strip.setPixelColor(n, red, green, blue);  
  strip.show(); // Initialize all pixels to 'off'
  
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
    ble.sendCommandCheckOK("AT+GAPDEVNAME=JEDI");
    Serial.println(F("******************************"));
  }

  // Added to support listen() via the microphone
  // This is only needed on 5V Arduinos (Uno, Leonardo, etc.).
  // Connect 3.3V to mic AND TO AREF ON ARDUINO and enable this
  // line.  Audio samples are 'cleaner' at 3.3V.
  // COMMENT OUT THIS LINE FOR 3.3V ARDUINOS (FLORA, ETC.):
  //  analogReference(EXTERNAL);

  memset(vol, 0, sizeof(vol));
  // @todo - needs to be fixed
  strip.begin();
  Serial.println("setup -> complete");  
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  digitalWrite(BOARD_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  // delay(1000);              // wait for a second
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
  /**
  if (strcmp(ble.buffer, "OK") == 0) {
    // no data
    return;
  }
  **/
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
  if (strcmp(ble.buffer, "14") == 0) {
      Serial.println("Setting animationState = 14");
      animationState = 14;
  }
  if (strcmp(ble.buffer, "15") == 0) {
      Serial.println("Setting animationState = 15");
      animationState = 15;
  }

  Serial.print("animationState = ");
  Serial.println(animationState);
  if (animationState == 0){
    strip.begin(); // This initializes the NeoPixel library.
    for(uint8_t i=0; i<N_PIXELS; i++) {
      strip.setPixelColor(i, strip.Color(0,0,0)); // off
    }
   }

   if (animationState == 1){
     colorWipe(strip.Color(100, 100, 100), 20); // do a quick colorWipe to show that the pixels are all working, even before Bluefruit connection established
     colorWipe(strip.Color(0, 0, 0), 20);
     strip.show();
    }

  if (animationState == 10){
    for(uint16_t i=0; i<strip.numPixels(); i++) { //clear all pixels before displaying new animation
          strip.setPixelColor(i, strip.Color(0,0,0));
        }
     flashRandom(1,random(10,30));
     strip.show(); // This sends the updated pixel color to the hardware.
   }

  if (animationState == 11){
    colorWipe(strip.Color(255, 0, 0), DELAY); // Red
    colorWipe(strip.Color(0, 255, 0), DELAY); // Green
    colorWipe(strip.Color(0, 0, 255), DELAY); // Blue
    /**
    for(uint16_t i=0; i<strip.numPixels(); i++) { //clear all pixels before displaying new animation
          Serial.println("animation state = 11a");
          Serial.print(strip.numPixels());
          strip.setPixelColor(i, strip.Color(0,0,0));
        }
    Serial.println("animation state = 11b");
    Serial.print(red);
    Serial.print(green);
    Serial.print(blue);    
    colorWipe(strip.Color(red, green, blue), 20);
    // strip.show(); // This sends the updated pixel color to the hardware.
    // colorWipe(strip.Color(0, 0, 0), 20);
    strip.show();
    **/
  }

  if (animationState == 12){
    for(uint16_t i=0; i<strip.numPixels(); i++) { //clear all pixels before displaying new animation
          strip.setPixelColor(i, strip.Color(0,0,0));
        }
    larsonScanner(15); // larsonScanner is set to red and does not take color input.
    strip.show(); // This sends the updated pixel color to the hardware.
  }

  if (animationState == 13){
    for(uint16_t i=0; i<strip.numPixels(); i++) { //clear all pixels before displaying new animation
          strip.setPixelColor(i, strip.Color(0,0,0));
        }
    rainbowCycle(20);
    strip.show(); // This sends the updated pixel color to the hardware.
  }

  if (animationState == 14){
    rainbow(DELAY);
  }

  if (animationState == 15){
    theaterChaseRainbow(DELAY);
  }
  /**
  if (animationState == 15){
    theaterChase();
  }
  **/
  
  Serial.println("loop() bottom");
  Serial.print("animationState = ");
  Serial.println(animationState);
  digitalWrite(BOARD_PIN, LOW);    // turn the LED off by making the voltage LOW
  // delay(1000);              // wait for a second
  ble.waitForOK();
}
// Functions
// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  Serial.println("  color wipe -> start");
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      Serial.print("  color wipe -> loop");    
      Serial.println(strip.numPixels());
      Serial.println(i);      
      strip.setPixelColor(i, c);
      strip.show();
      delay(10);
  }
  Serial.println("  color wipe -> complete");  
}

void larsonScanner(uint8_t wait){
   int j;

 for(uint16_t i=0; i<strip.numPixels()+5; i++) {
  // Draw 5 pixels centered on pos.  setPixelColor() will clip any
  // pixels off the ends of the strip, we don't need to watch for that.
  strip.setPixelColor(pos - 2, 0x100000); // Dark red
  strip.setPixelColor(pos - 1, 0x800000); // Medium red
  strip.setPixelColor(pos    , 0xFF3000); // Center pixel is brightest
  strip.setPixelColor(pos + 1, 0x800000); // Medium red
  strip.setPixelColor(pos + 2, 0x100000); // Dark red

  strip.show();
  delay(wait);

  // Rather than being sneaky and erasing just the tail pixel,
  // it's easier to erase it all and draw a new one next time.
  for(j=-2; j<= 2; j++) strip.setPixelColor(pos+j, 0);

  // Bounce off ends of strip
  pos += dir;
  if(pos < 0) {
    pos = 1;
    dir = -dir;
  } else if(pos >= strip.numPixels()) {
    pos = strip.numPixels() - 2;
    dir = -dir;
  }
 }
}

void flashRandom(int wait, uint8_t howmany) {
 randomSeed(analogRead(0));
  for(uint16_t i=0; i<howmany; i++) {
    // get a random pixel from the list
    int j = random(strip.numPixels());

    // now we will 'fade' it in 5 steps
    for (int x=0; x < 5; x++) {
      int r = red * (x+1); r /= 5;
      int g = green * (x+1); g /= 5;
      int b = blue * (x+1); b /= 5;

      strip.setPixelColor(j, strip.Color(r, g, b));
      strip.show();
      delay(wait);
    }
    // & fade out in 5 steps
    for (int x=5; x >= 0; x--) {
      int r = red * x; r /= 5;
      int g = green * x; g /= 5;
      int b = blue * x; b /= 5;

      strip.setPixelColor(j, strip.Color(r, g, b));
      strip.show();
      delay(wait);
    }
  }
  // LEDs will be off when done (they are faded to 0)
}

/**
void rainbowCycle(uint8_t wait) {
  Serial.println("  rainbowCycle -> start");
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
  Serial.println("  rainbowCycle -> complete");  
}
**/

void rainbow(uint8_t wait) {
  Serial.println("  rainbow -> start");         
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
  Serial.println("  rainbow -> complete");  
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  Serial.println("  rainbowCycle -> start");
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
  Serial.println("  rainbowCycle -> complete");  
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  Serial.println("  theaterChase -> start");
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, c);    //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
  Serial.println("  theaterChase -> complete");  
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  Serial.println("  theaterChaseRainbow -> start");
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
        for (int i=0; i < strip.numPixels(); i=i+3) {
          strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
        }
        strip.show();

        delay(wait);

        for (int i=0; i < strip.numPixels(); i=i+3) {
          strip.setPixelColor(i+q, 0);        //turn every third pixel off
        }
    }
  }
  Serial.println("  theaterChaseRainbow -> complete");  
}

// For RGB LED strips
void rgb() {
    int r, g, b;
 
  // fade from blue to violet
  for (r = 0; r < 256; r++) { 
    analogWrite(REDPIN, r);
    delay(FADESPEED);
  } 
  // fade from violet to red
  for (b = 255; b > 0; b--) { 
    analogWrite(BLUEPIN, b);
    delay(FADESPEED);
  } 
  // fade from red to yellow
  for (g = 0; g < 256; g++) { 
    analogWrite(GREENPIN, g);
    delay(FADESPEED);
  } 
  // fade from yellow to green
  for (r = 255; r > 0; r--) { 
    analogWrite(REDPIN, r);
    delay(FADESPEED);
  } 
  // fade from green to teal
  for (b = 0; b < 256; b++) { 
    analogWrite(BLUEPIN, b);
    delay(FADESPEED);
  } 
  // fade from teal to blue
  for (g = 255; g > 0; g--) { 
    analogWrite(GREENPIN, g);
    delay(FADESPEED);
  }
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

void listen() {
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;



  n   = analogRead(MIC_PIN);                        // Raw reading from mic
  n   = abs(n - 512 - DC_OFFSET); // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);             // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if(height < 0L)       height = 0;      // Clip output
  else if(height > TOP) height = TOP;
  if(height > peak)     peak   = height; // Keep 'peak' dot at top


  // Color pixels based on rainbow gradient
  for(i=0; i<N_PIXELS; i++) {
    if(i >= height)               strip.setPixelColor(i,   0,   0, 0);
    else strip.setPixelColor(i,Wheel(map(i,0,strip.numPixels()-1,30,150)));

  }

  // Draw peak dot
  if(peak > 0 && peak <= N_PIXELS-1) strip.setPixelColor(peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));

   strip.show(); // Update strip

  // Every few frames, make the peak pixel drop by 1:

    if(++dotCount >= PEAK_FALL) { //fall rate

      if(peak > 0) peak--;
      dotCount = 0;
    }

  vol[volCount] = n;                      // Save sample for dynamic leveling
  if(++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for(i=1; i<SAMPLES; i++) {
    if(vol[i] < minLvl)      minLvl = vol[i];
    else if(vol[i] > maxLvl) maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
    minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
    maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}

// Input a value 0 to 255 to get a color value.
// The colors are a transition r - g - b - back to r.

uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
