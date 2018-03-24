/*********************************************************************
Project:bluelight
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

#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

#ifdef __AVR__
#include <avr/power.h>   //Includes the library for power reduction registers if your chip supports them.
#endif                   //More info: http://www.nongnu.org/avr-libc/user-manual/group__avr__power.htlm

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
char inputs[BUFSIZE+1];
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);
extern uint8_t packetbuffer[];
uint8_t mode = 1;
bool debug = true;
bool bdebug = false;
String lastcmd = "";
String bleBuffer = "";
bool listening = false;
String ROLE = "JEDI";
int startMs = 0;
int startS = 0;
int timer = 0;
int timerMs = 0;
int currentMs = 0;
int state = 0;
const int DURATION = 307;                     // Duration of the main loop
const int t0 = START_DELAY;                   // from go to t1, show time
const int t1 = START_DELAY+91;                // track1 end
const int t2 = (START_DELAY-10)+157;          // track2 end
const int t3 = (START_DELAY-DRIFT+2)+233;     // track3 end
const int t4 = (START_DELAY-DRIFT+2)+248;     // track4 end
const int t5 = (START_DELAY-DRIFT+1)+307;     // track5 end

// Color and animation state definitions
uint32_t blue = strip.Color(0,0,255);
uint32_t red = strip.Color(0,255,0);
uint32_t green = strip.Color(255,0,0);
uint32_t white = strip.Color(255,255,255);
uint32_t black = strip.Color(0,0,0);
uint8_t animationState = 0;
uint8_t lastAnimationState = 0;

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

uint16_t gradient = 0; //Used to iterate and loop through each color palette gradually
uint8_t volume = 0;    //Holds the volume level read from the sound detector.
uint8_t last = 0;      //Holds the value of volume from the previous loop() pass.
float maxVol = 10;     //Holds the largest volume recorded thus far to proportionally adjust the visual's responsiveness.
float avgVol = 0;      //Holds the "average" volume-level to proportionally adjust the visual experience.
float avgBump = 0;     //Holds the "average" volume-change to trigger a "bump."
bool bump = false;     //Used to pass if there was a "bump" in volume
uint32_t listenColor;

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
  if (!ble.begin(VERBOSE_MODE)) {
    Serial.println("setup -> couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?");
  } else {
    Serial.println("setup -> OK");
  }

  if (FACTORYRESET_ENABLE) {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println("setup -> performing a factory reset");
    if ( ! ble.factoryReset() ){
      Serial.println("setup -> couldn't factory reset");
    }
  } else {
    Serial.print("setup -> factory reset disabled");
  }

  // Enable/Disable command echo from Bluefruit
  if(bdebug) {
    ble.echo(true);
  } else {
    ble.echo(false);
  }

  Serial.println("setup -> requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();
  // Serial.print(ble.info());

  Serial.println("setup -> Please use Adafruit Bluefruit LE app to connect in UART mode");
  Serial.println("setup -> Then Enter characters to send to Bluefruit");

  // Enable/Disable Bluefruit verbosity
  if (bdebug) {
    ble.verbose(true);
  } else {
    ble.verbose(false);
  }

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  // LED Activity command is only supported from 0.6.6
  if (ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION))
  {
    // Change Mode LED Activity
    Serial.println(F("setup -> change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    ble.sendCommandCheckOK("AT+GAPDEVNAME=JEDI");
  }

  // ROLE=String(ble.sendCommandCheckOK("AT+GAPDEVNAME"));

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
  digitalWrite(BOARD_PIN, HIGH);    // turn the LED on (HIGH is the voltage level)
  // delay(1000);                   // wait for a second
  log("loop -> top -> " + String(timer) + "\n");
  log("loop -> animationState = " + String(animationState) + "\n");

  // listen mode definitions
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;

  // Check for user input
  // Echoeing data received back to the sender
  /**
  if (getUserInput(inputs, BUFSIZE)) {
    // Send characters to Bluefruit
    log("loop -> sending = " + String(inputs) + "\n");

    ble.print("AT+BLEUARTTX=");
    ble.println(inputs);

    // check response stastus
    if (! ble.waitForOK()) {
      log("Failed to send?\n");
    }
  }
  **/

  // Check for incoming characters from Bluefruit
  ble.println("AT+BLEUARTRX");
  ble.readline();
  bleBuffer = String(ble.buffer);
  // Control for BLE buffer debug
  log("loop -> received = " + bleBuffer + "\n");

  // if (strcmp(ble.buffer, "OK") == 0) {
  /**
  if (bleBuffer.equals("OK") == 0) {
    // no data
    return;
  }
  **/

// Processing of control mode messages received over bluetooth
  if(!listening) {
    lastAnimationState = animationState;
    if (bleBuffer.equals("off")) {
        log("loop -> setting animationState = off\n");
        animationState = 0;
        lastcmd = bleBuffer;
    }
    if (bleBuffer.equals("start")) {
        log("loop -> setting animationState = start\n");
        animationState = 1;
        lastcmd = bleBuffer;
        startMs = millis();
        startS = startMs/1000;        
    }    
    if (bleBuffer.equals("blue")) {
        log("loop -> setting animationState = blue\n");
        animationState = 8;
        lastcmd = bleBuffer;
        lastAnimationState = animationState;
    }
    if (bleBuffer.equals("red")) {
        log("loop -> setting animationState = red\n");
        animationState = 16;
        lastcmd = bleBuffer;
    }
    if (bleBuffer.equals("rainbow")) {
        log("loop -> setting animationState = rainbow\n");
        animationState = 32;
        lastcmd = bleBuffer;
    }
    if (bleBuffer.equals("wipeblue")) {
        log("loop -> setting animationState = wipeblue\n");
        animationState = 40;
        lastcmd = bleBuffer;
    }
    if (bleBuffer.equals("wipered")) {
        log("loop -> setting animationState = wipered\n");
        animationState = 48;
        lastcmd = bleBuffer;
    }
    if (bleBuffer.equals("wipewhite")) {
        log("loop -> setting animationState = wipewhite\n");
        animationState = 56;
        lastcmd = bleBuffer;
    }
    if (bleBuffer.equals("wipegreen")) {
        log("loop -> setting animationState = wipegreen\n");
        animationState = 64;
        lastcmd = bleBuffer;
    }
    if (bleBuffer.equals("rainbowcycle")) {
        log("loop -> setting animationState = rainbowcycle\n");
        animationState = 72;
        lastcmd = bleBuffer;
    }
    if (bleBuffer.equals("rainbowtheater")) {
        log("loop -> setting animationState = rainbowtheater\n");
        animationState = 80;
        lastcmd = bleBuffer;
    }
    if (bleBuffer.equals("theaterchase")) {
        log("loop -> setting animationState = theaterchase\n");
        animationState = 88;
        lastcmd = bleBuffer;
    }
    if (bleBuffer.equals("test")) {
        log("loop -> setting animationState = test\n");
        animationState = 120;
        lastcmd = bleBuffer;
    }
    if (bleBuffer.equals("debugon")) {
        log("loop -> setting debug\n");
        debug = true;
        lastcmd = bleBuffer;
    }
    if (bleBuffer.equals("debugoff")) {
        log("loop -> setting debug\n");
        debug = false;
        lastcmd = bleBuffer;
    }
    if (bleBuffer.equals("bdebugon")) {
        log("loop -> setting debug\n");
        ble.echo(true);
        ble.verbose(true);
        lastcmd = bleBuffer;
    }
    if (bleBuffer.equals("bdebugoff")) {
        log("loop -> setting debug\n");
        ble.echo(false);
        ble.verbose(false);
        lastcmd = bleBuffer;
    }    
  } else {
      if(!bleBuffer.equals(lastcmd)) {
        lastAnimationState = animationState;
        log("loop -> changing listen mode\n");
        if (bleBuffer.equals("listenred")) {
          log("loop -> setting animationState = listenred\n");
          animationState = 128;
          lastcmd = bleBuffer;
        }
        if (bleBuffer.equals("listenblue")) {
            log("loop -> setting animationState = listenblue\n");
            animationState = 136;
            lastcmd = bleBuffer;
        }
      } else {
        log("loop -> last and current command are equal\n");
      }
    }

  if(animationState) {
    // Calculate time since start for each pass through the loop to control state
    log(("loop -> animationState = " + String(animationState) + " -> startMs = " + String(startMs) + " -> startS = " + String(startS) + "\n"));
    currentMs = millis();
    timerMs = currentMs - startMs;
    timer = timerMs/1000;

    // Checking timers and setting state flags
    if (timer <= t0) {
      // Pre-lightshow countdown, lights off
      state = 0;
    }
    else if ((timer > t0) && (timer <= t1)) {
      // Lightshow has started, phase t1
      state = 1;  
    }
    else if ((timer > t1) && (timer <= t2)) {
      // Lightshow has started, phase t2      
      state = 2;  
    }
    else if ((timer > t2) && (timer <= t3)) {
      // Lightshow has started, phase t3      
      state = 3;  
    }    
    else if ((timer > t3) && (timer <= t4)) {
      // Lightshow has started, phase t4
      state = 4;  
    }
    else if ((timer > t4) && (timer <= t5)) {
      // Lightshow has started, phase t5
      state = 5;
    }    
    else if (timer > DURATION) {
      state = 0;
    }

    // Showtime start
    // t0
    if(state == 0) {
      int actualt0Ms = millis();
      int actualt0S = actualt0Ms/1000;
      log(("loop -> t0 -> timer = " + String(timer) + " -> animationState = " + String(animationState) + " -> actualt0Ms = " + String(actualt0Ms) + " -> actualt0S = " + String(actualt0S) + "\n"));
      // the clock is ticking, everyone is off
      off();
    }
    // t1
    else if(state == 1) {
      int actualt1Ms = millis();
      int actualt1S = actualt1Ms/1000;
      log(("loop -> t1 -> timer = " + String(timer) + " -> animationState = " + String(animationState) + " -> actualt1Ms = " + String(actualt1Ms) + " -> actualt1S = " + String(actualt1S) + "\n"));      
      // the clock is ticking
      if(ROLE.equals("JEDI")) {
        // JEDI @ t1
        log(("loop -> t1 -> JEDI -> animationState = " + String(animationState) + " -> timer = " + String(timer) + "\n"));
        // blue
        solidColor(blue, DELAY);
      }
      if(ROLE.equals("CRED")) {
        // CRED @ t1
        log(("loop -> t1 -> CRED -> animationState = " + String(animationState) + " -> timer = " + String(timer) + "\n"));
        // off
        off();
      }
      if(ROLE.equals("CBLU")) {
        // CRBLU @ t1
        log(("loop -> t1 -> CBLU-> animationState = " + String(animationState) + " -> timer = " + String(timer) + "\n"));
        // off
        off();
      }    
    }  
    // t2
    else if(state == 2) {
      int actualt2Ms = millis();
      int actualt2S = actualt2Ms/1000;
      log(("loop -> t2 -> timer = " + String(timer) + " -> animationState = " + String(animationState) + " -> actualt2Ms = " + String(actualt2Ms) + " -> actualt2S = " + String(actualt2S) + "\n"));            
      // the clock is ticking
      if(ROLE.equals("JEDI")) {
        // JEDI @ t2
        // delay(1000);
        log(("loop -> t2 -> JEDI -> animationState = " + String(animationState) + " -> timer = " + String(timer) + "\n"));
        off();         
      }
      if(ROLE.equals("CRED")) {
        // CRED @ t2
        log(("loop -> t2 -> CRED -> animationState = " + String(animationState) + " -> timer = " + String(timer) + "\n"));
        // rainbow
        delay(2000);
        theaterChaseRainbow(DELAY);
      }
      if(ROLE.equals("CBLU")) {
        // CRBLU @ t2   
        log(("loop -> t2 -> CBLU-> animationState = " + String(animationState) + " -> timer = " + String(timer) + "\n"));
        // rainbow
        delay(2000);        
        theaterChaseRainbow(DELAY);        
      }    
    }
    // t3
    else if(state == 3) {
      int actualt3Ms = millis();
      int actualt3S = actualt3Ms/1000;
      log(("loop -> t3 -> timer = " + String(timer) + " -> animationState = " + String(animationState) + " -> actualt3Ms = " + String(actualt3Ms) + " -> actualt3S = " + String(actualt3S) + "\n"));
      // the clock is ticking
      if(ROLE.equals("JEDI")) {
        // JEDI @ t3
        log(("loop -> t3 -> JEDI -> animationState = " + String(animationState) + " -> timer = " + String(timer) + "\n"));
        // wipeblue
        // delay(5000);
        colorWipe(blue, DELAY);
        colorWipe(black, DELAY);
      }
      if(ROLE.equals("CRED")) {
        // CRED @ t3
        log(("loop -> t3 -> CRED -> animationState = " + String(animationState) + " -> timer = " + String(timer) + "\n"));
        // wipered
        colorWipe(red, DELAY);
        colorWipe(black, DELAY);        
      }
      if(ROLE.equals("CBLU")) {
        // CRBLU @ t3
        log(("loop -> t3 -> CBLU -> animationState = " + String(animationState) + " -> timer = " + String(timer) + "\n"));
        // wipeblue
        colorWipe(blue, DELAY);
        colorWipe(black, DELAY);        
      }    
    }    
   // t4
    else if(state == 4) {
      int actualt4Ms = millis();
      int actualt4S = actualt4Ms/1000;    
      log(("loop -> t4 -> timer = " + String(timer) + " -> animationState = " + String(animationState) + " -> actualt4Ms = " + String(actualt4Ms) + " -> actualt4S = " + String(actualt4S) + "\n"));
      // the clock is ticking
      // off
      off();        
    }        
    // t5
    else if(state == 5) {
      int actualt5Ms = millis();
      int actualt5S = actualt5Ms/1000;
      log(("loop -> t5 -> timer = " + String(timer) + " -> animationState = " + String(animationState) + " -> actualt5Ms = " + String(actualt5Ms) + " -> actualt5S = " + String(actualt5S) + "\n"));
      // the clock is ticking
      if(ROLE.equals("JEDI")) {
        // JEDI @ t5
        log(("loop -> t5 -> JEDI -> animationState = " + String(animationState) + " -> timer = " + String(timer) + "\n"));
        // wipeblue
        delay(4000);
        colorWipe(blue, DELAY);
        colorWipe(black, DELAY);
      } else {
        log(("loop -> t5 -> CRED/CBLU -> animationState = " + String(animationState) + " -> timer = " + String(timer) + "\n"));        
        // wipeblue
        colorWipe(blue, DELAY);
        colorWipe(black, DELAY);                
      }
    } else {
      off();
      timer = 0;
      int endMs = millis();
      int endS = endMs/1000;
      log(("loop -> animationState = " + String(animationState) + " -> startMs = " + String(endMs) + " -> startS = " + String(endS) + "\n"));      
    }
    // timer++;
    // Showtime end
  }
  
  log(("loop -> bottom -> animationState = " + String(animationState) + " -> timer = " + String(timer) + "\n"));  // turn the LED off by making the voltage LOW
  digitalWrite(BOARD_PIN, LOW);
  delay(500);
  ble.waitForOK();
}

// Low level common routines
// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  log(("\tWheel -> top -> " + String(WheelPos) + "\n"));
  if(WheelPos < 85) {
    log(("\tWheel -> 1\n"));
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < 170) {
      WheelPos -= 85;
      log(("\tWheel -> 2\n"));
      return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
      WheelPos -= 170;
      log(("\tWheel -> 3\n"));
      return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  log(("\tWheel -> bottom\n"));
}

// Low level LED control functions
void off() {
  log("\toff -> top\n");
  // This initializes the NeoPixel library
  strip.begin();
  for(uint8_t i=0; i<N_PIXELS; i++) {
    strip.setPixelColor(i, black); // off
  }
  strip.show();
  log("\toff -> bottom\n");
}

// Fill the dots one after the other with a color
void solidColor(uint32_t c, uint8_t wait) {
  log("\tsolid color -> start\n");
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      log("\tsolid color -> for_top -> number_of_pixels/i -> " + String(strip.numPixels()) + "/" + String(i) + "\n");
      strip.setPixelColor(i, c);
      log("\tsolid color -> set_pixel_color\n");
      strip.show();
      log("\tsolid color -> strip_show\n");
      delay(wait);
      log("\tsolid color -> for_bottom\n");
  }
  log("\tsolid color -> complete\n");
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  log("\tcolor wipe -> start\n");
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
  log("\tcolor wipe -> complete\n");
}

void rainbow(uint8_t wait) {
  log("\trainbow -> star\n");
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      log("\trainbow -> for_top  -> number_of_pixels/i/j -> " + String(strip.numPixels()) + "/" + String(i) + "/" + String(j) + "\n");
      strip.setPixelColor(i, Wheel((i+j) & 255));
      log("\trainbow -> for_bottom\n");
    }
    strip.show();
    delay(wait);
  }
  log("\trainbow -> complete\n");
}

void rainbowCycle(uint8_t wait) {
  log("\trainbowCycle -> start\n");
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      log("\trainbowCycle -> for_top  -> number_of_pixels/i/j -> " + String(strip.numPixels()) + "/" + String(i) + "/" + String(j) + "\n");
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
      log("\trainbowCycle -> for_bottom\n");
    }
    strip.show();
    delay(wait);
  }
  log("\trainbowCycle -> complete\n");
}

void theaterChase(uint32_t c, uint8_t wait) {
  log("\ttheaterChase -> start\n");
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (int i=0; i < strip.numPixels(); i=i+3) {
        log("\ttheaterChase -> for_top  -> number_of_pixels/i/j/q -> " + String(strip.numPixels()) + "/" + String(i) + "/" + String(j) + "/" + String(q) + "\n");
        strip.setPixelColor(i+q, c);    //turn every third pixel on
        log("\ttheaterChase -> for_bottom\n");
      }
      strip.show();
      delay(wait);

      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
  log("\ttheaterChase -> complete\n");
}

void theaterChaseRainbow(uint8_t wait) {
  log("\ttheaterChaseRainbow -> start\n");
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
  log("\ttheaterChaseRainbow -> complete\n");
}

void test(uint8_t wait) {
  log("\ttest -> start\n");
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
  log("\ttest -> complete\n");
}

// Low level routines for listening and adapting to audio
void listen(uint32_t c) {
  log("\tlisten -> top\n");
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;

  n   = analogRead(MIC_PIN);            // Raw reading from mic
  n   = abs(n - 512 - DC_OFFSET);       // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE); // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;           // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if(height < 0L)       height = 0;      // Clip output
  else if(height > TOP) height = TOP;
  if(height > peak)     peak   = height; // Keep 'peak' dot at top


  // Color pixels based on rainbow gradient
  for(i=0; i<N_PIXELS; i++) {
    if(i >= height) {
      log("\tlisten -> setting color -> i/N_PIXELS/height -> " + String(i) + "/" + String(N_PIXELS) + "/" + String(height) + "\n");
      strip.setPixelColor(i,black);
    }
    else {
      // strip.setPixelColor(i,Wheel(map(i,0,strip.numPixels()-1,30,150)));
      strip.setPixelColor(i,c);
    }
  }

  // Draw peak dot
  if(peak > 0 && peak <= N_PIXELS-1) {
    // strip.setPixelColor(peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
    strip.setPixelColor(peak,c);
    strip.show(); // Update strip
  }

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
  log("\tlisten -> bottom\n");
}

//Pulse from center of the strand
void pulse() {
  // fade(0.75);   //Listed below, this function simply dims the colors a little bit each pass of loop()

  //Advances the gradient to the next noticeable color if there is a "bump"
  if (bump) gradient += 64;

  //If it's silent, we want the fade effect to take over, hence this if-statement
  if (volume > 0) {
    uint32_t col = getRainbow(gradient); //Our retrieved 32-bit color
    // uint32_t col = getRainbow(listenColor); //Our retrieved 32-bit color

    //These variables determine where to start and end the pulse since it starts from the middle of the strand.
    //  The quantities are stored in variables so they only have to be computed once.
    // int start = LED_HALF - (LED_HALF * (volume / maxVol));
    int start = 0;
    // int finish = LED_HALF + (LED_HALF * (volume / maxVol)) + strip.numPixels() % 2;
    int finish = LED_HALF + (LED_HALF * (volume / maxVol)) + strip.numPixels() *.25;
    // int finish = N_PIXELS;
    //Listed above, LED_HALF is simply half the number of LEDs on your strand. ↑ this part adjusts for an odd quantity.

    for (int i = start; i < finish; i++) {
      //"damp" creates the fade effect of being dimmer the farther the pixel is from the center of the strand.
      //  It returns a value between 0 and 1 that peaks at 1 at the center of the strand and 0 at the ends.
      float damp = float(
                     ((finish - start) / 2.0) -
                     abs((i - start) - ((finish - start) / 2.0))
                   )
                   / float((finish - start) / 2.0);

      //Sets the each pixel on the strand to the appropriate color and intensity
      //  strand.Color() takes 3 values between 0 & 255, and returns a 32-bit integer.
      //  Notice "knob" affecting the brightness, as in the rest of the visuals.
      //  Also notice split() being used to get the red, green, and blue values.
      /**
      strip.setPixelColor(i, strip.Color(
                             split(col, 0) * pow(damp, 2.0),
                             split(col, 1) * pow(damp, 2.0),
                             split(col, 2) * pow(damp, 2.0)
                           ));

      strip.setPixelColor(i, strip.Color(
                             split(col, 0) * pow(damp, 2.0),
                             split(col, 255) * pow(damp, 2.0),
                             split(col, 0) * pow(damp, 2.0)
                           ));      
      **/
                                       
      strip.setPixelColor(i, listenColor);
    }
    //Sets the max brightness of all LEDs. If it's loud, it's brighter.
    //  "knob" was not used here because it occasionally caused minor errors in color display.
    strip.setBrightness(255.0 * pow(volume / maxVol, 2));
  }
  //This command actually shows the lights. If you make a new visualization, don't forget this!
  strip.show();
}

//Fades lights by multiplying them by a value between 0 and 1 each pass of loop().
void fade(float damper) {
  //"damper" must be between 0 and 1, or else you'll end up brightening the lights or doing nothing.
  if (damper >= 1) damper = 0.99;
  for (int i = 0; i < strip.numPixels(); i++) {
    //Retrieve the color at the current position.
    uint32_t col = (strip.getPixelColor(i)) ? strip.getPixelColor(i) : strip.Color(0, 0, 0);
    //If it's black, you can't fade that any further.
    if (col == 0) continue;
    float colors[3]; //Array of the three RGB values
    //Multiply each value by "damper"
    for (int j = 0; j < 3; j++) colors[j] = split(col, j) * damper;
    //Set the dampened colors back to their spot.
    strip.setPixelColor(i, strip.Color(colors[0] , colors[1], colors[2]));
  }
}

uint8_t split(uint32_t color, uint8_t i ) {
  //0 = Red, 1 = Green, 2 = Blue
  if (i == 0) return color >> 16;
  if (i == 1) return color >> 8;
  if (i == 2) return color >> 0;
  return -1;
}

//This function simply take a value and returns a gradient color
//  in the form of an unsigned 32-bit integer

//The gradient returns a different, changing color for each multiple of 255
//  This is because the max value of any of the 3 LEDs is 255, so it's
//  an intuitive cutoff for the next color to start appearing.
//  Gradients should also loop back to their starting color so there's no jumps in color.

uint32_t getRainbow(unsigned int i) {
  if (i > 1529) return getRainbow(i % 1530);
  if (i > 1274) return strip.Color(255, 0, 255 - (i % 255));   //violet -> red
  if (i > 1019) return strip.Color((i % 255), 0, 255);         //blue -> violet
  if (i > 764) return strip.Color(0, 255 - (i % 255), 255);    //aqua -> blue
  if (i > 509) return strip.Color(0, 255, (i % 255));          //green -> aqua
  if (i > 255) return strip.Color(255 - (i % 255), 255, 0);    //yellow -> green
  return strip.Color(255, i, 0);                               //red -> yellow
}

// @brief  Checks for user input (via the Serial Monitor)
bool getUserInput(char buffer[], uint8_t maxSize) {
  log("\tgetUserInput -> top\n");
  // timeout in 100 milliseconds
  TimeoutTimer timeout(100);

  memset(buffer, 0, maxSize);
  while( (!Serial.available()) && !timeout.expired() ) { delay(1); }

  if ( timeout.expired() ) return false;

  delay(2);
  uint8_t count=0;
  do {
    count += Serial.readBytes(buffer+count, maxSize);
    delay(2);
  } while( (count < maxSize) && (Serial.available()) );

  log("\tgetUserInput -> bottom\n");
  return true;
}

// error printing routine
void error(const __FlashStringHelper*err) {
  if(debug) {
    Serial.println(err);
    // while (1);
  }
}

// logging routine
void log(String message) {
  if(debug) {
    Serial.print("[" + String(millis()) + "]:" + "[" + ROLE + "]:" + message);
  }
}
