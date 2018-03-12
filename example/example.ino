#include <Adafruit_NeoPixel.h>

#define PIN 6
#define DELAY 0

// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
// Adafruit_NeoPixel strip = Adafruit_NeoPixel(150, PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(93, PIN, NEO_GRB);

void setup() {
  Serial.println("setup -> start");
  strip.begin();
  // strip.setPixelColor(n, red, green, blue);  
  strip.show(); // Initialize all pixels to 'off'
  Serial.println("setup -> complete");  
}

void loop() {
  Serial.println("loop -> start");  
  // Some example procedures showing how to display to the pixels:
  Serial.println("color wipe");    
  colorWipe(strip.Color(255, 0, 0), DELAY); // Red
  colorWipe(strip.Color(0, 255, 0), DELAY); // Green
  colorWipe(strip.Color(0, 0, 255), DELAY); // Blue

  // Send a theater pixel chase in...
  Serial.println("theater");      
  theaterChase(strip.Color(127, 127, 127), DELAY); // White
  theaterChase(strip.Color(127,   0,   0), DELAY); // Red
  theaterChase(strip.Color(  0,   0, 127), DELAY); // Blue

  Serial.println("rainbow");
  rainbow(DELAY);
  Serial.println("rainbowCycle");
  rainbowCycle(DELAY);
  Serial.println("theaterChaseRainbow");  
  theaterChaseRainbow(DELAY);
  Serial.println("loop -> complete");    
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  Serial.println("  color wipe -> start");
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
  Serial.println("  color wipe -> complete");  
}

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
