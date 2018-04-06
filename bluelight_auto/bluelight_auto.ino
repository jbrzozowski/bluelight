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
// Create the bluefruit object, either software serial...uncomment these lines
/**
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
**/
/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);
// Adafruit_BluefruitLE_UART ble(Serial, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
/**
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
**/
// Line below creates functional BLE object
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// Using strip instead, below is for NeoPixel pixel object
// Adafruit_NeoPixel pixel = Adafruit_NeoPixel(N_PIXELS, PIN);
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

// Constants
// Role
String ROLE = "JEDI";
const int T1_ADJ = 0;
const int T2_ADJ = 0;
const int T3_ADJ = -5;
const int T4_ADJ = -19;
const int T1_BASE = 91;
const int T2_BASE = 157;
const int T3_BASE = 216;
const int T4_BASE = 248;
int startDelay = START_DELAY;
// Duration of the main loop
// const int DURATION = 307;
const int DURATION = 260; // used to be 337, 307 is the real duration
// from start to t1, show time - all lights are off
int t0 = startDelay;
// track1 - first song
int t1 = (startDelay+T1_ADJ)+T1_BASE;
// track2 - second song
int t2 = (startDelay+T2_ADJ)+T2_BASE;
// track3 - third song  //-6 && removed DRIFT
// int t3 = (startDelay-13)+233;
int t3 = (startDelay+T3_ADJ)+T3_BASE; // used to be 236
// track4 - all lights off for a short period of time //-13 && removed DRIFT
// int t4 = (startDelay-11)+248; // used to be -15
int t4 = (startDelay+T4_ADJ)+T4_BASE; // used to be -15, used to be 259, -21 used to be -11
// const int t5 = (START_DELAY-DRIFT-1)+307;
// track5 - last song follow up by all lights off
int t5 = (startDelay)+DURATION;

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
int startMs = 0;
int startS = 0;
int timer = 0;
int timerMs = 0;
int currentMs = 0;
int state = 0;
int runningState = 0;
String deviceAddress = "";
String deviceNameBase = "JEDI";
String deviceNameSuffix = " (UNKNOWN)";
String deviceName = "";
String deviceNameCommand = "";
int loopCheck = 0;

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

// listen mode variables
uint8_t  listenc;
uint16_t minLvl, maxLvl;
int      n, height;

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
    ble.println("AT+BLEUARTRX");
    Serial.println("setup -> couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?");
  } else {
    Serial.println("setup -> OK");
  }

  if (FACTORYRESET_ENABLE) {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println("setup -> performing a factory reset");
    if (!ble.factoryReset()){
      ble.println("AT+BLEUARTRX");
      Serial.println("setup -> couldn't factory reset");
    }
  } else {
    Serial.print("setup -> factory reset disabled");
  }

  // Enable/Disable command echo from Bluefruit
  if(bdebug) {
    ble.echo(true);
    ble.println("AT+BLEUARTRX");
  } else {
    ble.echo(false);
    ble.println("AT+BLEUARTRX");
  }

  // Enable/Disable Bluefruit verbosity
  if (bdebug) {
    ble.verbose(true);
    ble.println("AT+BLEUARTRX");
  } else {
    ble.verbose(false);
    ble.println("AT+BLEUARTRX");
  }

  /* Wait for connection */
  while (!ble.isConnected()) {
      ble.println("AT+BLEUARTRX");
      delay(500);
  }

  // LED Activity command is only supported from 0.6.6
  if (ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION)) {
    ble.println("AT+BLEUARTRX");
    ble.println("AT+BLEGETADDR");
    // ble.println("AT+BLEUARTRX");

    while (ble.readline()) {
      Serial.println("setup -> ble.buffer -> " + String(ble.buffer));
      String buffer = String(ble.buffer);
      Serial.println("setup -> buffer -> " + buffer);
      if(buffer.equals("OK")) {
        continue;
      } else {
        deviceAddress += buffer;
      }
      Serial.println("setup -> deviceAddress -> " + String(deviceAddress));
    }

    deviceAddress.replace("OK","");
    deviceAddress.replace(":","");

    Serial.println("setup -> final deviceAddress -> " + String(deviceAddress));

    // deviceAddress to deviceName mapping
    // Controller units
    if(deviceAddress.equals("E1CBAB612F3C")) {
      deviceNameBase = "JEDI";
      deviceNameSuffix = " (_MAIN)";
      ROLE = "JEDI";
    }
    if(deviceAddress.equals("DEADBEEFCAFE")) {
      deviceNameBase = "JEDI";
      deviceNameSuffix = " (_BACKUP)";
      ROLE = "JEDI";
    }
    // JEDI
    if(deviceAddress.equals("E6EFEFFA02C1")) {
      deviceNameBase = "JEDI";
      deviceNameSuffix = " (Johan)";
      ROLE = "JEDI";
    }
    if(deviceAddress.equals("FA4120D2C91E")) {
      deviceNameBase = "JEDI";
      deviceNameSuffix = " (LiamM)";
      ROLE = "JEDI";
    }
    if(deviceAddress.equals("F40D027498BF")) {
      deviceNameBase = "JEDI";
      deviceNameSuffix = " (LiamB)";
      ROLE = "JEDI";
    }
    if(deviceAddress.equals("FD2BC8414E31")) {
      deviceNameBase = "JEDI";
      deviceNameSuffix = " (Katrina)";
      ROLE = "JEDI";
    }
    if(deviceAddress.equals("F177C6C39E1B")) {
      deviceNameBase = "JEDI";
      deviceNameSuffix = " (JackMag)";
      ROLE = "JEDI";
    }
    if(deviceAddress.equals("C56B1E27AE97")) {
      deviceNameBase = "JEDI";
      deviceNameSuffix = " (JackMcL)";
      ROLE = "JEDI";
    }
    if(deviceAddress.equals("EFC103572EAC")) {
      deviceNameBase = "JEDI";
      deviceNameSuffix = " (Joey)";
      ROLE = "JEDI";
    }
    if(deviceAddress.equals("C4A4B1181F4C")) {
      deviceNameBase = "JEDI";
      deviceNameSuffix = " (Greg)";
      ROLE = "JEDI";
    }
    if(deviceAddress.equals("FE74C6D5F6FF")) {
      deviceNameBase = "JEDI";
      deviceNameSuffix = " (Julian)";
      ROLE = "JEDI";
    }
    if(deviceAddress.equals("C0F156EBFD25")) {
      deviceNameBase = "JEDI";
      deviceNameSuffix = " (Ben)";
      ROLE = "JEDI";
    }
    if(deviceAddress.equals("E7EFC57D4394")) {
      deviceNameBase = "JEDI";
      deviceNameSuffix = " (Yemi)";
      ROLE = "JEDI";
    }
    if(deviceAddress.equals("FE499044E396")) {
      deviceNameBase = "JEDI";
      deviceNameSuffix = " (Marlee)";
      ROLE = "JEDI";
    }
    if(deviceAddress.equals("F2B469E62281")) {
      deviceNameBase = "JEDI";
      deviceNameSuffix = " (Callie)";
      ROLE = "JEDI";
    }
    // Crowd red
    if(deviceAddress.equals("E829D03B9468")) {
      deviceNameBase = "CRED";
      deviceNameSuffix = " (MadisonR)";
      ROLE = "CRED";
    }
    if(deviceAddress.equals("E3C5C04E8409")) {
      deviceNameBase = "CRED";
      deviceNameSuffix = " (SofiaB)";
      ROLE = "CRED";
    }
    if(deviceAddress.equals("CCE758AEE102")) {
      deviceNameBase = "CRED";
      deviceNameSuffix = " (Molly)";
      ROLE = "CRED";
    }
    if(deviceAddress.equals("DDB8C012D919")) {
      deviceNameBase = "CRED";
      deviceNameSuffix = " (SophieW)";
      ROLE = "CRED";
    }
    if(deviceAddress.equals("E736AE157CD1")) {
      deviceNameBase = "CRED";
      deviceNameSuffix = " (Quinn)";
      ROLE = "CRED";
    }
    if(deviceAddress.equals("E137E7E43A75")) {
      deviceNameBase = "CRED";
      deviceNameSuffix = " (Anna)";
      ROLE = "CRED";
    }
    if(deviceAddress.equals("F1625AB398EC")) {
      deviceNameBase = "CRED";
      deviceNameSuffix = " (SydneyP)";
      ROLE = "CRED";
    }
    if(deviceAddress.equals("E5900506E163")) {
      deviceNameBase = "CRED";
      deviceNameSuffix = " (Ashley)";
      ROLE = "CRED";
    }
    if(deviceAddress.equals("F98B42202CB3")) {
      deviceNameBase = "CRED";
      deviceNameSuffix = " (SidneyD)";
      ROLE = "CRED";
    }
    if(deviceAddress.equals("D73B200A1044")) {
      deviceNameBase = "CRED";
      deviceNameSuffix = " (Krystal)";
      ROLE = "CRED";
    }
    // Crowd blue
    if(deviceAddress.equals("EBA66841CBCC")) {
      deviceNameBase = "CBLU";
      deviceNameSuffix = " (Unathi)";
      ROLE = "CBLU";
    }
    if(deviceAddress.equals("D8FE84E8B2BE")) {
      deviceNameBase = "CBLU";
      deviceNameSuffix = " (Ava)";
      ROLE = "CBLU";
    }
    if(deviceAddress.equals("E878900748DC")) {
      deviceNameBase = "CBLU";
      deviceNameSuffix = " (Michaela)";
      ROLE = "CBLU";
    }
    if(deviceAddress.equals("C959CC1FB381")) {
      deviceNameBase = "CBLU";
      deviceNameSuffix = " (Julia)";
      ROLE = "CBLU";
    }
    if(deviceAddress.equals("F6D9041FF758")) {
      deviceNameBase = "CBLU";
      deviceNameSuffix = " (Melody)";
      ROLE = "CBLU";
    }
    if(deviceAddress.equals("EF1EAD9E88BC")) {
      deviceNameBase = "CBLU";
      deviceNameSuffix = " (Isabella)";
      ROLE = "CBLU";
    }
    if(deviceAddress.equals("D2CAF26ADD1D")) {
      deviceNameBase = "CBLU";
      deviceNameSuffix = " (Lola)";
      ROLE = "CBLU";
    }
    if(deviceAddress.equals("D2AE8CAAC59E")) {
      deviceNameBase = "CBLU";
      deviceNameSuffix = " (Isa)";
      ROLE = "CBLU";
    }
    if(deviceAddress.equals("EAC11D36CD40")) {
      deviceNameBase = "CBLU";
      deviceNameSuffix = " (Emerson)";
      ROLE = "CBLU";
    }
    if(deviceAddress.equals("E82D6ABE15FC")) {
      deviceNameBase = "CBLU";
      deviceNameSuffix = " (Adrienne)";
      ROLE = "CBLU";
    }
    if(deviceAddress.equals("C1A89FD1ECDB")) {
      deviceNameBase = "CBLU";
      deviceNameSuffix = " (Brianna)";
      ROLE = "CBLU";
    }

    deviceName += deviceNameBase;
    deviceName += deviceNameSuffix;
    deviceNameCommand = "AT+GAPDEVNAME=";
    deviceNameCommand += deviceName;
    Serial.println("setup -> deviceNameCommand -> " + String(deviceNameCommand));
    ble.println(deviceNameCommand);
    // ble.sendCommandCheckOK("AT+GAPDEVNAME=JEDI");

    // Increasing BLE power level
    ble.sendCommandCheckOK("AT+BLEPOWERLEVEL");
    ble.sendCommandCheckOK("AT+BLEPOWERLEVEL=4");
    ble.sendCommandCheckOK("AT+BLEPOWERLEVEL");
    // ble.sendCommandCheckOK("AT+BLEGETADDR");
    // Checking if connectable
    ble.sendCommandCheckOK("AT+GAPCONNECTABLE");

    // Change Mode LED Activity
    Serial.println(F("setup -> change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
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
  Serial.println("setup -> requesting/displaying bluefruit info");
  /* Print Bluefruit information */
  ble.info();

  // Serial.print(ble.info());
  Serial.println("setup -> Software image information -> " + String(__FILE__) + "(" + String(__DATE__) + " " + String(__TIME__) + ")");
  Serial.println("setup -> Setup complete, please use Adafruit Bluefruit LE app to connect in UART mode");
  Serial.println("setup -> Then Enter characters to send to Bluefruit");
}

// Main loop
void loop(void) {
  digitalWrite(BOARD_PIN, LOW);
  loopCheck++;
  // Code to intercept reset button pressing
  /**
  if (digitalRead(1) == LOW) {
    log("loop -> reset pressed\n");
    resetTimerMs = millis();
    //debounce
    delay(200);
  }
  // check if the switch is pressed for longer than 1 second.
  if(digitalRead(1) == LOW && resetTimerMs - millis() >1000) {
      //add 1 Step to next Mode in setup
      resetPressCount++;
      //switch back to 0 after the required modes
      if(resetPressCount==8) {
        resetPressCount=0;
      }
      // if it is a short press <1000
  } else {
    log("loop -> reset long press\n");
  }
  **/
  // log("loop -> milli_check -> " + String(millis()) + " -> loop_check -> " + String(loopCheck) + "\n");
  // digitalWrite(BOARD_PIN, HIGH);    // turn the LED on (HIGH is the voltage level)
  // log("loop -> deviceAddress = " + deviceAddress + "\n");
  log("loop -> top -> " + String(timer) + " -> animationState = " + String(animationState) + " -> listening -> " + String(listening) + "\n");
  // log("loop -> milli_check -> " + String(millis()) + " -> loop_check -> " + String(loopCheck) + "\n");
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
  // log("loop -> milli_check -> " + String(millis()) + " -> loop_check -> " + String(loopCheck) + "\n");
  // if (strcmp(ble.buffer, "OK") == 0) {
  /**
  if (bleBuffer.equals("OK") == 0) {
    // no data
    return;
  }
  **/

  if (bleBuffer.startsWith("delay")) {
      log("loop -> overriding start_delay -> " + String(bleBuffer) + "\n");
      String newDelay = bleBuffer;
      newDelay.replace("delay=","");
      log("loop -> overriding start_delay new start_delay -> " + String(newDelay) + "\n");
      startDelay = newDelay.toInt();
      updateStartDelay(startDelay);
      log("loop -> overriding start_delay is now -> " + String(startDelay) + "\n");
      lastcmd = bleBuffer;
  }
  // log("loop -> milli_check -> " + String(millis()) + " -> loop_check -> " + String(loopCheck) + "\n");
  if (bleBuffer.startsWith("listen")) {
      log("loop -> listening set to true\n");
      listening = true;
      // lastcmd = bleBuffer;
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
  }
  // log("loop -> milli_check -> " + String(millis()) + " -> loop_check -> " + String(loopCheck) + "\n");
  // Processing of control mode messages received over bluetooth
  if(!listening) {
    lastAnimationState = animationState;
    if (bleBuffer.equals("off")) {
        log("loop -> setting animationState = off\n");
        animationState = 0;
        lastcmd = bleBuffer;
    }
    else if (bleBuffer.equals("start")) {
        log("loop -> setting animationState = start\n");
        animationState = 1;
        lastcmd = bleBuffer;
        startMs = millis();
        startS = startMs/1000;
    }
    else if (bleBuffer.equals("blue")) {
        log("loop -> setting animationState = blue\n");
        animationState = 8;
        lastcmd = bleBuffer;
        lastAnimationState = animationState;
    }
    else if (bleBuffer.equals("red")) {
        log("loop -> setting animationState = red\n");
        animationState = 16;
        lastcmd = bleBuffer;
    }
    else if (bleBuffer.equals("rainbow")) {
        log("loop -> setting animationState = rainbow\n");
        animationState = 32;
        lastcmd = bleBuffer;
    }
    else if (bleBuffer.equals("wipeblue")) {
        log("loop -> setting animationState = wipeblue\n");
        animationState = 40;
        lastcmd = bleBuffer;
    }
    else if (bleBuffer.equals("wipered")) {
        log("loop -> setting animationState = wipered\n");
        animationState = 48;
        lastcmd = bleBuffer;
    }
    else if (bleBuffer.equals("wipewhite")) {
        log("loop -> setting animationState = wipewhite\n");
        animationState = 56;
        lastcmd = bleBuffer;
    }
    else if (bleBuffer.equals("wipegreen")) {
        log("loop -> setting animationState = wipegreen\n");
        animationState = 64;
        lastcmd = bleBuffer;
    }
    else if (bleBuffer.equals("rainbowcycle")) {
        log("loop -> setting animationState = rainbowcycle\n");
        animationState = 72;
        lastcmd = bleBuffer;
    }
    else if (bleBuffer.equals("rainbowtheater")) {
        log("loop -> setting animationState = rainbowtheater\n");
        animationState = 80;
        lastcmd = bleBuffer;
    }
    else if (bleBuffer.equals("theaterchase")) {
        log("loop -> setting animationState = theaterchase\n");
        animationState = 88;
        lastcmd = bleBuffer;
    }
    else if (bleBuffer.equals("test")) {
        log("loop -> setting animationState = test\n");
        animationState = 120;
        lastcmd = bleBuffer;
    }
    else if (bleBuffer.equals("debugon")) {
        log("loop -> setting debug\n");
        debug = true;
        lastcmd = bleBuffer;
    }
    else if (bleBuffer.equals("debugoff")) {
        log("loop -> setting debug\n");
        debug = false;
        lastcmd = bleBuffer;
    }
    else if (bleBuffer.equals("bdebugon")) {
        log("loop -> setting debug\n");
        ble.echo(true);
        ble.verbose(true);
        lastcmd = bleBuffer;
    }
    else if (bleBuffer.equals("bdebugoff")) {
        log("loop -> setting debug\n");
        ble.echo(false);
        ble.verbose(false);
        lastcmd = bleBuffer;
    }
  }
  /**
  else {
      if(!bleBuffer.equals(lastcmd)) {
        lastAnimationState = animationState;
        log("loop -> changing listen mode -> " + bleBuffer  + "\n");
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
    **/
  // log("loop -> milli_check -> " + String(millis()) + " -> loop_check -> " + String(loopCheck) + "\n");
  if (animationState == 0){
    // off
    log("loop -> animationState = " + String(animationState) + " -> off\n");
    off();
    // strip.show();
  }
  else if(animationState == 1) {
    // digitalWrite(BOARD_PIN, LOW);
    // delay(10);
    digitalWrite(BOARD_PIN, HIGH);
    // Calculate time since start for each pass through the loop to control state
    currentMs = millis();
    timerMs = currentMs - startMs;
    timer = timerMs/1000;
    log("loop -> animationState = " + String(animationState) + " -> startMs = " + String(startMs) + " -> startS = " + String(startS) + " -> currentMs = " + String(currentMs) + " -> timerMs = " + String(timerMs) + " -> timer = " + String(timer) + "\n");
    log("loop -> startDelay = " + String(startDelay) + " -> t1 = " + String(t1) + " -> t2 = " + String(t2) + " -> t3 = " + String(t3) + " -> t4 = " + String(t4) + " -> t5 = " + String(t5) + "\n");
    // Checking timers and setting state flags
    if (timer <= t0) {
      // Pre-lightshow countdown, lights off
      state = 0;
      log("state_change=0\n");
    }
    if ((timer >= t0) && (timer <= t1)) {
      // Lightshow has started, phase t1
      state = 1;
      log("state_change=1\n");
    }
    if ((timer >= t1) && (timer <= t2)) {
      // Lightshow has started, phase t2
      state = 2;
      log("state_change=2\n");
    }
    if ((timer >= t2) && (timer <= t3)) {
      // Lightshow has started, phase t3
      state = 3;
      log("state_change=3\n");
    }
    if ((timer >= t3) && (timer <= t4)) {
      // Lightshow has started, phase t4
      state = 4;
      log("state_change=4\n");
    }
    if ((timer >= t4) && (timer <= t5)) {
      // Lightshow has started, phase t5
      state = 5;
      log("state_change=5\n");
    }
    if (timer >= t5) {
      state = 6;
      log("state_change=6\n");
    }

    // Showtime start
    // t0
    if(state == 0) {
      int actualt0Ms = millis();
      int actualt0S = actualt0Ms/1000;
      log("loop -> t0 -> timer = " + String(timer) + " -> runningState = " + String(runningState) + " -> animationState = " + String(animationState) + " -> actualt0Ms = " + String(actualt0Ms) + " -> actualt0S = " + String(actualt0S) + "\n");
      // the clock is ticking, everyone is off
      off();
    }
    // t1
    else if(state == 1) {
      int actualt1Ms = millis();
      int actualt1S = actualt1Ms/1000;
      log("loop -> t1 -> timer = " + String(timer) + " -> animationState = " + String(animationState) + " -> actualt1Ms = " + String(actualt1Ms) + " -> actualt1S = " + String(actualt1S) + "\n");
      // the clock is ticking
      if(ROLE.equals("JEDI")) {
        // JEDI @ t1
        log("loop -> t1 -> JEDI -> animationState = " + String(animationState) + " -> timer = " + String(timer) + "\n");
        // blue
        solidColor(blue, DELAY);
      }
      if(ROLE.equals("CRED")) {
        // CRED @ t1
        log("loop -> t1 -> CRED -> animationState = " + String(animationState) + " -> timer = " + String(timer) + "\n");
        // off
        off();
      }
      if(ROLE.equals("CBLU")) {
        // CRBLU @ t1
        log("loop -> t1 -> CBLU-> animationState = " + String(animationState) + " -> timer = " + String(timer) + "\n");
        // off
        off();
      }
    }
    // t2
    else if(state == 2) {
      int actualt2Ms = millis();
      int actualt2S = actualt2Ms/1000;
      log("loop -> t2 -> timer = " + String(timer) + " -> animationState = " + String(animationState) + " -> actualt2Ms = " + String(actualt2Ms) + " -> actualt2S = " + String(actualt2S) + "\n");
      // the clock is ticking
      if(ROLE.equals("JEDI")) {
        // JEDI @ t2
        // delay(1000);
        log("loop -> t2 -> JEDI -> animationState = " + String(animationState) + " -> timer = " + String(timer) + "\n");
        off();
      }
      if(ROLE.equals("CRED")) {
        // CRED @ t2
        log("loop -> t2 -> CRED -> animationState = " + String(animationState) + " -> timer = " + String(timer) + "\n");
        // rainbow
        // delay(2000);
        theaterChaseRainbow(DELAY);
        startMs -= 1000;
      }
      if(ROLE.equals("CBLU")) {
        // CRBLU @ t2
        log("loop -> t2 -> CBLU-> animationState = " + String(animationState) + " -> timer = " + String(timer) + "\n");
        // rainbow
        // delay(2000);
        theaterChaseRainbow(DELAY);
        startMs -= 1000;
      }
    }
    // t3
    else if(state == 3) {
      int actualt3Ms = millis();
      int actualt3S = actualt3Ms/1000;
      // listening = true;
      log("loop -> t3 -> timer = " + String(timer) + " -> animationState = " + String(animationState) + " -> actualt3Ms = " + String(actualt3Ms) + " -> actualt3S = " + String(actualt3S) + " -> listening -> " + String(listening) + "\n");
      // the clock is ticking
      if(ROLE.equals("JEDI")) {
        // JEDI @ t3
        log("loop -> t3 -> JEDI -> animationState = " + String(animationState) + " -> timer = " + String(timer) + "\n");
        // delay(5000);
        // wipeblue
        colorWipe(blue, DELAY);
        colorWipe(black, DELAY);
        // listenblue
        // listen(blue);
      }
      if(ROLE.equals("CRED")) {
        // CRED @ t3
        log("loop -> t3 -> CRED -> animationState = " + String(animationState) + " -> timer = " + String(timer) + "\n");
        // wipered
        colorWipe(red, DELAY);
        colorWipe(black, DELAY);
        // listenred
        // listen(red);
      }
      if(ROLE.equals("CBLU")) {
        // CBLU @ t3
        log("loop -> t3 -> CBLU -> animationState = " + String(animationState) + " -> timer = " + String(timer) + "\n");
        // wipeblue
        colorWipe(blue, DELAY);
        colorWipe(black, DELAY);
        // listenblue
        // listen(blue);
      }
    }
   // t4
    else if(state == 4) {
      int actualt4Ms = millis();
      int actualt4S = actualt4Ms/1000;
      listening = false;
      log("loop -> t4 -> timer = " + String(timer) + " -> animationState = " + String(animationState) + " -> actualt4Ms = " + String(actualt4Ms) + " -> actualt4S = " + String(actualt4S) + " -> listening -> " + String(listening) + "\n");
      // the clock is ticking
      // off
      off();
    }
    // t5
    else if(state == 5) {
      int actualt5Ms = millis();
      int actualt5S = actualt5Ms/1000;
      // listening = true;
      // log("loop -> milli_check -> " + String(millis()) + " -> loop_check -> " + String(loopCheck) + "\n");
      log("loop -> t5 -> timer = " + String(timer) + " -> animationState = " + String(animationState) + " -> actualt5Ms = " + String(actualt5Ms) + " -> actualt5S = " + String(actualt5S) + " -> listening -> " + String(listening) + "\n");
      // the clock is ticking
      if(ROLE.equals("JEDI")) {
        // JEDI @ t5
        // log("loop -> milli_check -> " + String(millis()) + " -> loop_check -> " + String(loopCheck) + "\n");
        log("loop -> t5 -> JEDI -> animationState = " + String(animationState) + " -> timer = " + String(timer) + "\n");
        // delay(4000);
        // wipeblue
        // colorWipe(blue, DELAY);
        // colorWipe(black, DELAY);
        // listenblue
        // listen(blue);
        // rainbowcycle
        rainbowCycle(DELAY);
      } else {
        // log("loop -> milli_check -> " + String(millis()) + " -> loop_check -> " + String(loopCheck) + "\n");
        log("loop -> t5 -> CRED/CBLU -> animationState = " + String(animationState) + " -> timer = " + String(timer) + "\n");
        // wipeblue
        // colorWipe(blue, DELAY);
        // colorWipe(black, DELAY);
        // listenblue
        // listen(blue);
        // rainbowcycle
        rainbowCycle(DELAY);
      }
    }
    else if(state == 6) {
      int actualt6Ms = millis();
      int actualt6S = actualt6Ms/1000;
      listening = true;
      // log("loop -> milli_check -> " + String(millis()) + " -> loop_check -> " + String(loopCheck) + "\n");
      log("loop -> t6 -> timer = " + String(timer) + " -> animationState = " + String(animationState) + " -> actualt6Ms = " + String(actualt6Ms) + " -> actualt6S = " + String(actualt6S) + " -> listening -> " + String(listening) + "\n");
      // the clock is ticking
      // JEDI @ t5
      // log("loop -> milli_check -> " + String(millis()) + " -> loop_check -> " + String(loopCheck) + "\n");
      log("loop -> t6 -> JEDI/CBLU/CRED -> animationState = " + String(animationState) + " -> timer = " + String(timer) + "\n");
      // listenblue
      listen(blue);
    }
    // timer++;
    // Showtime end
    // off();
    // timer = 0;
    int endMs = millis();
    int endS = endMs/1000;
    log("loop -> timer = " + String(timer) + " -> runningState = " + String(runningState) + " -> animationState = " + String(animationState) + " -> startMs = " + String(endMs) + " -> startS = " + String(endS) + "\n");
    // digitalWrite(BOARD_PIN, LOW);
    // delay(10);
    digitalWrite(BOARD_PIN, LOW);
    // delay(10);
  }
  else if (animationState == 8){
    // blue
    log("loop -> animationState = " + String(animationState) + " -> solid_color -> blue\n");
    solidColor(blue, DELAY);
    // strip.show();
  }
  else if (animationState == 16){
    // red
    log("loop -> animationState = " + String(animationState) + " -> solid_color -> red\n");
    solidColor(red, DELAY);
    // strip.show();
  }
  else if (animationState == 32){
    // rainbow
    log("loop -> animationState = " + String(animationState) + " -> rainbow\n");
    rainbow(DELAY);
    // strip.show();
  }
  else if (animationState == 40){
    // wipeblue
    log("loop -> animationState = " + String(animationState) + " -> color_wipe -> blue\n");
    colorWipe(blue, DELAY);
    colorWipe(black, DELAY);
    // strip.show();
  }
  else if (animationState == 48){
    // wipered
    log("loop -> animationState = " + String(animationState) + " -> color_wipe -> red\n");
    colorWipe(red, DELAY);
    colorWipe(black, DELAY);
    // strip.show();
  }
  else if (animationState == 56){
    // wipewhite
    log("loop -> animationState = " + String(animationState) + " -> color_wipe -> white\n");
    colorWipe(white, DELAY);
    colorWipe(black, DELAY);
    // strip.show();
  }
  else if (animationState == 64){
    // wipegreen
    log("loop -> animationState = " + String(animationState) + " -> color_wipe -> green\n");
    colorWipe(green, DELAY);
    colorWipe(black, DELAY);
    // strip.show();
  }
  else if (animationState == 72){
    // rainbow cycle
    log("loop -> animationState = " + String(animationState) + " -> rainbow_cycle\n");
    rainbowCycle(DELAY);
    // strip.show();
  }
  else if (animationState == 80){
    // rainbowtheater
    log("loop -> animationState = " + String(animationState) + " -> rainbow_theater\n");
    theaterChaseRainbow(DELAY);
    // strip.show();
  }
  else if (animationState == 88){
    // theaterchase
    log("loop -> animationState = " + String(animationState) + " -> theater_chase\n");
    theaterChase(white, DELAY); // White
    theaterChase(green, DELAY); // Green
    theaterChase(red, DELAY); // Red
    theaterChase(blue, DELAY); // Blue
    // strip.show();
  }
  else if (animationState == 128) {
    // log("loop -> milli_check -> " + String(millis()) + " -> loop_check -> " + String(loopCheck) + "\n");
    // listenred
    log("loop -> animationState = " + String(animationState) + " -> listen_red\n");
    listen(red);
    // strip.show();
  }
  else if (animationState == 136) {
    // log("loop -> milli_check -> " + String(millis()) + " -> loop_check -> " + String(loopCheck) + "\n");
    // listenblue
    log("loop -> animationState = " + String(animationState) + " -> listen_blue\n");
    listen(blue);
    // strip.show();
  }
  else if (animationState == 255){
    // test
    log("loop -> animationState = " + String(animationState) + " -> test\n");
    test(DELAY);
    // strip.show();
  }
  // log("loop -> milli_check -> " + String(millis()) + " -> loop_check -> " + String(loopCheck) + "\n");
  log("loop -> bottom -> animationState = " + String(animationState) + " -> timer = " + String(timer) + "\n");  // turn the LED off by making the voltage LOW
  // digitalWrite(BOARD_PIN, HIGH);
  // delay(3000);
  if(!listening) {
    // log("loop -> ble_wait_for_ok\n");
    ble.waitForOK();
  }
  // log("loop -> milli_check -> " + String(millis()) + " -> loop_check -> " + String(loopCheck) + "\n");
}

// Low level common routines
// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  // log(("\twheel -> top -> " + String(WheelPos) + "\n"));
  if(WheelPos < 85) {
    // log(("\twheel -> 1\n"));
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
  else if (WheelPos < 170) {
    WheelPos -= 85;
    // log(("\twheel -> 2\n"));
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  else {
    WheelPos -= 170;
    // log(("\twheel -> 3\n"));
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  // log(("\twheel -> bottom\n"));
}

// Low level LED control functions
void off() {
  // log("\toff -> top\n");
  // This initializes the NeoPixel library
  strip.begin();
  for(uint8_t i=0; i<N_PIXELS; i++) {
    strip.setPixelColor(i, black); // off
  }
  strip.show();
  // log("\toff -> bottom\n");
}

// Fill the dots one after the other with a color
void solidColor(uint32_t c, uint8_t wait) {
  // log("\tsolid_color -> start\n");
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      // log("\tsolid_color -> for_top -> number_of_pixels/i -> " + String(strip.numPixels()) + "/" + String(i) + "\n");
      strip.setPixelColor(i, c);
      // log("\tsolid_color -> set_pixel_color\n");
      // strip.show(); // moved outside of the main loop
      // log("\tsolid_color -> strip_show\n");
      delay(wait);
      // log("\tsolid_color -> for_bottom\n");
  }
  strip.show();
  // log("\tsolid_color -> complete\n");
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  // log("\tcolor_wipe -> start\n");
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
  // log("\tcolor_wipe -> complete\n");
}

void rainbow(uint8_t wait) {
  // log("\trainbow -> star\n");
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      // log("\trainbow -> for_top  -> number_of_pixels/i/j -> " + String(strip.numPixels()) + "/" + String(i) + "/" + String(j) + "\n");
      strip.setPixelColor(i, Wheel((i+j) & 255));
      // log("\trainbow -> for_bottom\n");
    }
    strip.show();
    delay(wait);
  }
  // log("\trainbow -> complete\n");
}

void rainbowCycle(uint8_t wait) {
  // log("\trainbow_cycle -> start\n");
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      // log("\trainbow_cycle -> for_top  -> number_of_pixels/i/j -> " + String(strip.numPixels()) + "/" + String(i) + "/" + String(j) + "\n");
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
      // log("\trainbow_cycle -> for_bottom\n");
    }
    strip.show();
    delay(wait);
  }
  // log("\trainbow_cycle -> complete\n");
}

void theaterChase(uint32_t c, uint8_t wait) {
  // log("\ttheater_chase -> start\n");
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (int i=0; i < strip.numPixels(); i=i+3) {
        // log("\ttheater_chase -> for_top  -> number_of_pixels/i/j/q -> " + String(strip.numPixels()) + "/" + String(i) + "/" + String(j) + "/" + String(q) + "\n");
        strip.setPixelColor(i+q, c);    //turn every third pixel on
        // log("\ttheater_chase -> for_bottom\n");
      }
      strip.show();
      delay(wait);
      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
  // log("\ttheater_chase -> complete\n");
}

void theaterChaseRainbow(uint8_t wait) {
  // log("\ttheater_chase_rainbow -> start\n");
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
  // log("\ttheater_chase_rainbow -> complete\n");
}

void test(uint8_t wait) {
  // log("\ttest -> start\n");
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
  // log("\ttest -> complete\n");
}

// Low level routines for listening and adapting to audio
void listen(uint32_t c) {
  log("\tlisten -> top -> " + String(c) + "\n");
  // Raw reading from mic
  n   = analogRead(MIC_PIN);
  // log("loop -> analogRead -> raw -> " + String(n));
  n   = abs(n - 512 - DC_OFFSET); // Center on zero
  //// log("\tlisten -> analogRead -> zeroed -> " + String(n) + "\n");
  n   = (n <= NOISE) ? 0 : (n - NOISE);             // Remove noise/hum
  //// log("\tlisten -> analogRead -> noise removed  -> " + String(n) + "\n");
  //// log("\tlisten -> lvl -> before -> " + String(lvl) + "\n");
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)
  //// log("\tlisten -> lvl -> dampened -> " + String(lvl) + "\n");

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);
  //// log("\tlisten -> height -> " + String(height) + "\n");

  if(height < 0L)       height = 0;      // Clip output
  else if(height > TOP) height = TOP;
  if(height > peak)     peak   = height; // Keep 'peak' dot at top

  // Color pixels based on rainbow gradient
  for(listenc=0; listenc<N_PIXELS; listenc++) {
    // log("\tlisten -> listenc -> " + String(listenc) + "\n");
    if(listenc >= height) {
      // log("\tlisten -> listenc -> set_to_000\n");
      strip.setPixelColor(listenc,0,0,0);
    } else {
      // log("\tlisten -> listenc -> set_to_color\n");
      strip.setPixelColor(listenc,c);
    // else strip.setPixelColor(i,Wheel(map(i,0,strip.numPixels()-1,255,255)));
    }
  }

  // Draw peak dot
  // if(peak > 0 && peak <= N_PIXELS-1) strip.setPixelColor(peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
  if(peak > 0 && peak <= N_PIXELS-1) {
    //// log("\tlisten -> listenc -> set_peak\n");
    strip.setPixelColor(peak,c);
  }

  //// log("\tlisten -> strip_show\n");
  strip.show(); // Update strip

  // Every few frames, make the peak pixel drop by 1:
  if(++dotCount >= PEAK_FALL) { //fall rate
    //// log("\tlisten -> dotCount -> " + String(dotCount) + "\n");
    if(peak > 0) peak--;
    dotCount = 0;
  }

  vol[volCount] = n;                      // Save sample for dynamic leveling
  if(++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter
  //// log("\tlisten -> volCount -> " + String(volCount) + "\n");

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for(listenc=1; listenc<SAMPLES; listenc++) {
    if(vol[listenc] < minLvl)      minLvl = vol[listenc];
    else if(vol[listenc] > maxLvl) maxLvl = vol[listenc];
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
    // col populated by getRainbow() is unused
    // uint32_t col = getRainbow(gradient); //Our retrieved 32-bit color
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
      /** damp is unused, commenting out to suppress compilation warnings
      float damp = float(
                     ((finish - start) / 2.0) -
                     abs((i - start) - ((finish - start) / 2.0))
                   )
                   / float((finish - start) / 2.0);
      **/
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

// Recalcuate delay timers// error printing routine
void updateStartDelay(int newDelay) {
  log("\tupdateStartDelay -> old -> " + String(startDelay)+ "\n");
  startDelay = newDelay;
  t0 = startDelay;
  t1 = (startDelay+T1_ADJ)+T1_BASE;
  t2 = (startDelay+T2_ADJ)+T2_BASE;
  t3 = (startDelay+T3_ADJ)+T3_BASE;
  t4 = (startDelay+T4_ADJ)+T4_BASE;
  t5 = (startDelay)+DURATION;
  log("\tupdateStartDelay -> new -> " + String(startDelay)+ "\n");
}
// @brief  Checks for user input (via the Serial Monitor)
bool getUserInput(char buffer[], uint8_t maxSize) {
  log("\tget_user_input -> top\n");
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

  log("\tget_user_input -> bottom\n");
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
  // int timeStamp = String(millis();
  if(debug) {
    // Serial.print("[" + String(timeStamp) + "]:" + "[" + ROLE + "]:" + message);
    Serial.print("[" + ROLE + "]:" + message);
  }
}
