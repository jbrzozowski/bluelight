/*********************************************************************
Project:blueligh
Author:John Jason Brzozowski
Email:jjmb@jjmb.com

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

// BLE and Bluetooth variable and object definitions
// @brief  Sets up the HW an the BLE module (this function is called automatically on startup)
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

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// Adafruit NeoPixel variable and object definitions
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

// Global variables
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);
extern uint8_t packetbuffer[];
uint8_t mode = 1;
bool debug = true;

// Color and animation state definitions
uint32_t blue = strip.Color(0,0,255);
uint32_t red = strip.Color(0,255,0);
uint32_t green = strip.Color(255,0,0);
uint32_t white = strip.Color(255,255,255);
uint32_t black = strip.Color(0,0,0);
uint8_t animationState = 0;

// Listening to audio to control LED definitions
byte
  peak      = 0,      // Used for falling dot
  dotCount  = 0,      // Frame counter for delaying dot-falling speed
  volCount  = 0;      // Frame counter for storing past volume data
int
  vol[SAMPLES],       // Collection of prior volume samples
  lvl       = 10,      // Current "dampened" audio level
  minLvlAvg = 0,      // For dynamic adjustment of graph low & high
  maxLvlAvg = 512;

// Setup
void setup(void)
{
  Serial.println("setup -> start");
  // Configure pin for onboard LED blinking
  pinMode(BOARD_PIN, OUTPUT);

  // Comment out to prevent the need to connect serially
  /**
  while (!Serial);  // required for Flora & Micro
  delay(500);
  **/

  // Strip setup
  Serial.println("setup -> setting brightness");
  strip.setBrightness(BRIGHTNESS);
  strip.begin();

  // Serial setup
  Serial.print(F("setup -> initialising serial communications"));
  Serial.begin(115200);
  Serial.println(F("setup -> Adafruit Bluefruit command mode"));

  /* Initialise the module */
  Serial.print(F("setup -> initialising the Bluefruit LE module"));

  // BLE setup
  if ( !ble.begin(VERBOSE_MODE) )
  {
    log(F("setup -> couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("setup -> Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      log(F("setup -> Couldn't factory reset"));
    }
  } else {
    Serial.print(F("setup -> factory reset disabled"));
  }

  // Enable/Disable command echo from Bluefruit
  if(debug) {
    ble.echo(true);
  } else {
    ble.echo(false);
  }

  Serial.println("setup -> Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();
  // Serial.print(ble.info());

  Serial.println(F("setup -> Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("setup -> Then Enter characters to send to Bluefruit"));
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
    Serial.println(F("setup -> Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    ble.sendCommandCheckOK("AT+GAPDEVNAME=JEDI");
  }

  // Added to support listen() via the microphone
  // This is only needed on 5V Arduinos (Uno, Leonardo, etc.).
  // Connect 3.3V to mic AND TO AREF ON ARDUINO and enable this
  // line.  Audio samples are 'cleaner' at 3.3V.
  // COMMENT OUT THIS LINE FOR 3.3V ARDUINOS (FLORA, ETC.):
  //  analogReference(EXTERNAL);

  memset(vol, 0, sizeof(vol));
  // Initialize all pixels to 'off'
  strip.show();
  Serial.println("setup -> complete");
}

// Main loop
void loop(void) {
  digitalWrite(BOARD_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  // delay(1000);              // wait for a second
  log(F("loop -> top"));
  log(F("loop -> animationState = "));
  log(F(animationState));
  // Check for user input
  // char inputs[BUFSIZE+1];

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
  log(F("loop -> Buffer = "));
  log(F(ble.buffer));

  /**
  if (strcmp(ble.buffer, "OK") == 0) {
    // no data
    return;
  }
  **/

// Processing of control mode messages received over bluetooth
// @todo - add a check for runtime setting of debug
  if (strcmp(ble.buffer, "off") == 0) {
      log(F("loop -> setting animationState = off"));
      animationState = 0;
  }
  if (strcmp(ble.buffer, "blue") == 0) {
      log(F("loop -> setting animationState = blue"));
      animationState = 8;
  }
  if (strcmp(ble.buffer, "red") == 0) {
      log(F("loop -> setting animationState = red"));
      animationState = 16;
  }
  if (strcmp(ble.buffer, "rainbow") == 0) {
      log(F("loop -> setting animationState = rainbow"));
      animationState = 32;
  }
  if (strcmp(ble.buffer, "wipeblue") == 0) {
      log(F("loop -> setting animationState = wipeblue"));
      animationState = 40;
  }
  if (strcmp(ble.buffer, "wipered") == 0) {
      log(F("loop -> setting animationState = wipered"));
      animationState = 48;
  }
  if (strcmp(ble.buffer, "wipewhite") == 0) {
      log(F("loop -> setting animationState = wipewhite"));
      animationState = 56;
  }
  if (strcmp(ble.buffer, "wipegreen") == 0) {
      log(F("loop -> setting animationState = wipegreen"));
      animationState = 64;
  }
  if (strcmp(ble.buffer, "rainbowcycle") == 0) {
      log(F("loop -> setting animationState = rainbowcycle"));
      animationState = 72;
  }

  if (strcmp(ble.buffer, "rainbowtheater") == 0) {
      log(F("loop -> setting animationState = rainbowtheater"));
      animationState = 80;
  }

  if (strcmp(ble.buffer, "theaterchase") == 0) {
      log(F("loop -> setting animationState = theaterchase"));
      animationState = 88;
  }

  if (strcmp(ble.buffer, "listenred") == 0) {
      log(F("loop -> setting animationState = listenred"));
      animationState = 128;
  }
  if (strcmp(ble.buffer, "listenblue") == 0) {
      log(F("loop -> setting animationState = listenblue"));
      animationState = 136;
  }
  if (strcmp(ble.buffer, "test") == 0) {
      log(F("loop -> setting animationState = test"));
      animationState = 255;
  }
  if (strcmp(ble.buffer, "debugon") == 0) {
      log(F("loop -> setting debug"));
      debug = true;
      ble.echo(true);
      ble.verbose(true);
  }
  if (strcmp(ble.buffer, "debugoff") == 0) {
      log(F("loop -> setting debug"));
      debug = false;
      ble.echo(false);
      ble.verbose(false);
  }

// Light control mode routines
  log(F("loop -> animationState = "));
  log(F(animationState));

// New control processing
  if (animationState == 0){
    // off
    off();
    // strip.show();
   }

  if (animationState == 8){
    // blue
    solidColor(blue, DELAY);
    // strip.show();
  }

  if (animationState == 16){
    // red
    solidColor(red, DELAY);
    // strip.show();
  }

  if (animationState == 32){
    // rainbow
    rainbow(DELAY);
    // strip.show();
  }

  if (animationState == 40){
    // wipeblue
    colorWipe(blue, DELAY);
    colorWipe(black, DELAY);
    // strip.show();
  }

  if (animationState == 48){
    // wipered
    colorWipe(red, DELAY);
    colorWipe(black, DELAY);
    // strip.show();
  }

  if (animationState == 56){
    // wipewhite
    colorWipe(white, DELAY);
    colorWipe(black, DELAY);
    // strip.show();
  }

  if (animationState == 64){
    // wipegreen
    colorWipe(green, DELAY);
    colorWipe(black, DELAY);
    // strip.show();
  }

  if (animationState == 72){
    // rainbow cycle
    rainbowCycle(DELAY);
    // strip.show();
  }

  if (animationState == 80){
    // rainbowtheater
    theaterChaseRainbow(DELAY);
    // strip.show();
  }

  if (animationState == 88){
    // theaterchase
    theaterChase(white, DELAY); // White
    theaterChase(green, DELAY); // Green
    theaterChase(red, DELAY); // Red
    theaterChase(blue, DELAY); // Blue
    // strip.show();
  }

  if (animationState == 128){
    // listenred
    test(DELAY);
    // strip.show();
  }

  if (animationState == 136){
    // listenblue
    test(DELAY);
    // strip.show();
  }

  if (animationState == 255){
    // test
    test(DELAY);
    // strip.show();
  }

  log(F("loop -> bottom"));
  log(F("loop -> animationState = "));
  log(F(animationState));
  // turn the LED off by making the voltage LOW
  digitalWrite(BOARD_PIN, LOW);
  // delay(1000);              // wait for a second
  ble.waitForOK();
}

// Low level common routines
// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
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

// Low level LED control functions
void off() {
  // This initializes the NeoPixel library
  strip.begin();
  for(uint8_t i=0; i<N_PIXELS; i++) {
    strip.setPixelColor(i, black); // off
  }
  strip.show();  
}

// Fill the dots one after the other with a color
void solidColor(uint32_t c, uint8_t wait) {
  log(F("  solid color -> start"));
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      log(F("  solid color -> for_top -> number_of_pixels/i"));
      log(F(strip.numPixels()));
      log(F(i));
      strip.setPixelColor(i, c);
      log(F("  solid color -> set_pixel_color"));
      strip.show();
      log(F("  solid color -> strip_show"));
      delay(wait);
      log(F("  solid color -> for_bottom"));
  }
  log(F("  solid color -> complete"));
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  log(F("  color wipe -> start"));
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
  log(F("  color wipe -> complete"));
}

void rainbow(uint8_t wait) {
  log(F("  rainbow -> start"));
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      log(F("  rainbow -> for_top  -> number_of_pixels/i/j"));
      log(F(strip.numPixels()));
      log(F(i));
      log(F(j));
      strip.setPixelColor(i, Wheel((i+j) & 255));
      log(F("  rainbow -> for_bottom"));
    }
    strip.show();
    delay(wait);
  }
  log(F("  rainbow -> complete"));
}

void rainbowCycle(uint8_t wait) {
  log(F("  rainbowCycle -> start"));
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      log(F("  rainbowCycle -> for_top  -> number_of_pixels/i/j"));
      log(F(strip.numPixels()));
      log(F(i));
      log(F(j));
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
      log(F("  rainbowCycle -> for_bottom"));
    }
    strip.show();
    delay(wait);
  }
  log(F("  rainbowCycle -> complete"));
}

void theaterChase(uint32_t c, uint8_t wait) {
  log(F("  theaterChase -> start"));
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (int i=0; i < strip.numPixels(); i=i+3) {
        log(F("  theaterChase -> for_top  -> number_of_pixels/i/j/q"));
        log(F(strip.numPixels()));
        log(F(i));
        log(F(j));
        log(F(q));
        strip.setPixelColor(i+q, c);    //turn every third pixel on
        log(F("  theaterChase -> for_bottom"));
      }
      strip.show();
      delay(wait);

      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
  log(F("  theaterChase -> complete"));
}

void theaterChaseRainbow(uint8_t wait) {
  log(F("  theaterChaseRainbow -> start"));
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
  log(F("  theaterChaseRainbow -> complete"));
}

void test(uint8_t wait) {
  log(F("  test -> start"));
  // Color wipe
  colorWipe(red, DELAY); // Red
  colorWipe(white, DELAY); // White
  colorWipe(blue, DELAY); // Blue
  colorWipe(green, DELAY); // Green

  // Send a theater pixel chase in...
  theaterChase(red, DELAY); // Red
  theaterChase(white, DELAY); // White
  theaterChase(blue, DELAY); // Blue
  theaterChase(green, DELAY); // Green

  rainbow(DELAY);
  rainbowCycle(DELAY);
  theaterChaseRainbow(DELAY);
  log(F("  test -> complete"));
}

// Low level routines for listening and adapting to audio
void listen() {
  // @todo - need to add support for accepting color as an argument
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

// @brief  Checks for user input (via the Serial Monitor)
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

// error printing routine
void log(const __FlashStringHelper*err) {
  if(debug) {
    Serial.println(err);
    // while (1);
  }
}
