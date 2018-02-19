/*
LED VU meter for Arduino and Adafruit NeoPixel LEDs.

Hardware requirements:
 - Most Arduino or Arduino-compatible boards (ATmega 328P or better).
 - Adafruit Electret Microphone Amplifier (ID: 1063)
 - Adafruit Flora RGB Smart Pixels (ID: 1260)
   OR
 - Adafruit NeoPixel Digital LED strip (ID: 1138)
 - Optional: battery for portable use (else power through USB or adapter)
Software requirements:
 - Adafruit NeoPixel library

Connections:
 - 3.3V to mic amp +
 - GND to mic amp -
 - Analog pin to microphone output (configurable below)
 - Digital pin to LED data input (configurable below)
 See notes in setup() regarding 5V vs. 3.3V boards - there may be an
 extra connection to make and one line of code to enable or disable.

Written by Adafruit Industries.  Distributed under the BSD license.
This paragraph must be included in any redistribution.
*/

#include <Adafruit_NeoPixel.h>

#define DELAY 0
#define N_PIXELS  91  // Number of pixels in strand
#define MIC_PIN   A1  // Microphone is attached to this analog pin
#define LED_PIN    6  // NeoPixel LED strand is connected to this pin
#define DC_OFFSET  0  // DC offset in mic signal - if unusure, leave 0
#define NOISE     10  // Noise/hum/interference in mic signal
#define SAMPLES   90  // Length of buffer for dynamic level adjustment
#define TOP       (N_PIXELS + 2) // Allow dot to go slightly off scale
#define PEAK_FALL 40  // Rate of peak falling dot

byte
  peak      = 0,      // Used for falling dot
  dotCount  = 0,      // Frame counter for delaying dot-falling speed
  volCount  = 0;      // Frame counter for storing past volume data
int
  vol[SAMPLES],       // Collection of prior volume samples
  lvl       = 10,      // Current "dampened" audio level
  minLvlAvg = 0,      // For dynamic adjustment of graph low & high
  maxLvlAvg = 512;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_PIXELS, LED_PIN, NEO_GRB);
// Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

void setup() {
  Serial.println("setup -> start");
  // Configure pin for onboard LED blinking
  // pinMode(13, OUTPUT);

  // This initializes the NeoPixel library
  strip.begin();
  
  for(uint8_t i=0; i<N_PIXELS; i++) {
    strip.setPixelColor(i, strip.Color(0,0,0)); // off
  }
  
  colorWipe(strip.Color(100, 100, 100), 20); // do a quick colorWipe to show that the pixels are all working, even before Bluefruit connection established
  colorWipe(strip.Color(0, 0, 0), 20);

  // Initialize all pixels to 'off'
  strip.show();
  // strip.begin();
  // strip.setPixelColor(n, red, green, blue);  
  // strip.show();

  memset(vol, 0, sizeof(vol));
  // strip.begin();
  Serial.println("setup -> complete");
}

void loop() {
  Serial.println("loop -> start");  
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;

  // Raw reading from mic
  Serial.println(" reading mic");
  n   = analogRead(MIC_PIN);

  // Center on zero
  Serial.println(" center on zero");
  n   = abs(n - 512 - DC_OFFSET);

  // Remove noise/hum
  Serial.println(" remove noise/hum");
  n   = (n <= NOISE) ? 0 : (n - NOISE);

  // "Dampened" reading (else looks twitchy)  
  Serial.println(" dampen");
  lvl = ((lvl * 7) + n) >> 3;

  // Calculate bar height based on dynamic min/max levels (fixed point):
  Serial.println(" setting height");
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  // Clip output
  Serial.println(" clip output");
  if(height < 0L)       height = 0;
  else if(height > TOP) height = TOP;

  // Keep 'peak' dot at top  
  Serial.println(" peak = height");
  if(height > peak)     peak   = height;

  // Color pixels based on rainbow gradient
  for(i=0; i<N_PIXELS; i++) {
    Serial.println("  setting pixel color");      
    if(i >= height)               strip.setPixelColor(i,   0,   0, 0);
    else strip.setPixelColor(i,Wheel(map(i,0,strip.numPixels()-1,30,150)));
  }
  
  // Draw peak dot
  Serial.println(" draw peak dot");
  if(peak > 0 && peak <= N_PIXELS-1) strip.setPixelColor(peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));

  // Update strip
  Serial.println(" update strip");
   strip.show();

  // Every few frames, make the peak pixel drop by 1
  Serial.println("  ...fall rate");    
  if(++dotCount >= PEAK_FALL) {
    //fall rate
    Serial.println(" fall rate...");
    if(peak > 0) peak--;
    dotCount = 0;
  }

  // Save sample for dynamic leveling
  Serial.println(" saving sample for dynamic leveling");
  vol[volCount] = n;

  // Advance/rollover sample counter
  Serial.println(" advance/rollover sample counter");
  if(++volCount >= SAMPLES) volCount = 0;

  // Get volume range of prior frames
  Serial.println(" get volume range");
  minLvl = maxLvl = vol[0];
  for(i=1; i<SAMPLES; i++) {
    if(vol[i] < minLvl)      minLvl = vol[i];
    else if(vol[i] > maxLvl) maxLvl = vol[i];
  }
  
  /**
  minLvl and maxLvl indicate the volume range over prior frames, used
  for vertically scaling the output graph (so it looks interesting
  regardless of volume level).  If they're too close together though
  (e.g. at very low volume levels) the graph becomes super coarse
  and 'jumpy'...so keep some minimum distance between them (this
  also lets the graph go to zero when no sound is playing):
  **/
  if((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  // Dampen min/max levels
  Serial.println(" dampen min/max levels");
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6;
  // (fake rolling average)
  Serial.println(" fake rolling average");
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6;
  Serial.println("loop -> end");      
}

// Input a value 0 to 255 to get a color value
// The colors are a transition r - g - b - back to r
uint32_t Wheel(byte WheelPos) {
  Serial.println("wheel -> start");  
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  Serial.println("wheel -> end");
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  Serial.println("color wipe -> start");
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
  Serial.println("color wipe -> complete");  
}
