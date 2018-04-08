# bluelight
Project bluelight  - open source project that enables remote control of bluetooth/BLE enable LED lights
This project was developed using the Arduino IDE which was replaced with Platform.io using the Atom IDE.  The Atom IDE and Platform.io components were used to build and deploy the software from the command line.  More details will follow as to how this was done.

# Components
bluelight_auto and bluelight_remote are two separate projects.  These will most likely be merged into a single project.
## bluelight_auto
After struggling with interference from the BLE controller to the BLE modules that control the lights we developed an auto mode that times LED changes with specific music.  You can change the timers and the music used yourself.  The auto mode also comes pre-made with specific LED patterns that can be used to control the lights directly from a BLE controller.
## bluelight_remote
bluelight_remote was the original mode designed and devloped.  In contains specific LED modes that can used to control the connected LEDs.

# To do
* Merge bluelight_auto and bluelight_remote into a single project
* Post details Atom and Platform.io for automated builds and deployment
* Provide list of components
* Documentation
* Investigate alternative for Bluefruit LE Connect, specifically a combination of hcitool and gattool (https://www.jaredwolff.com/blog/get-started-with-bluetooth-low-energy/).  There are also some open source python projects that are being evaluated.

# Reference projects:
* https://github.com/adafruit/Adafruit_BluefruitLE_nRF51
* https://github.com/adafruit/Adafruit_NeoPixel

# Parts list
* Adafruit Feather M0 Bluefruit LE (https://www.adafruit.com/product/2995) - this is the main board that is used to control the LEDs, it supports BLE which is how we communicate with it to control the LEDs.
* Electret Microphone Amplifier - MAX4466 with Adjustable Gain (https://www.adafruit.com/product/1063) - this is a microphone that is used to listen to sound (music).  There is code in bluelight_auto that will pulse LEDs to sound (music).
* 3 x AAA Battery Holder with On/Off Switch and 2-Pin JST (https://www.adafruit.com/product/727) - this is required to power the Adafruit Feather M0 Bluefruit LE and the LEDs.
* AAA batteries (https://www.amazon.com/gp/product/B01KBEORDK/ref=oh_aui_detailpage_o05_s00?ie=UTF8&psc=1) - this is the power source.
* LEDs (https://www.amazon.com/gp/product/B01K6TOP5K/ref=oh_aui_detailpage_o00_s00?ie=UTF8&psc=1) - these are the LEDs, they are available in packs of 50 LEDs.  These can be cut.
* Heat shrink tubing (https://www.amazon.com/gp/product/B06XS5G492/ref=oh_aui_detailpage_o07_s00?ie=UTF8&psc=1) - when creating some of the wiring we needed to cover exposed wiring that has been soldered to prevent shorts, etc.
* Connectors (https://www.amazon.com/gp/product/B00NBSH4CA/ref=oh_aui_detailpage_o05_s00?ie=UTF8&psc=1) - these are the connectors used to ease the connecting and disconnecting of the LED strands.
* Bluefruit LE Connect for iOS (https://learn.adafruit.com/bluefruit-le-connect-for-ios/ios-setup) - this is the iPad/iPhone application that was used to remotely control the LEDs over BLE.  There is an Android version as well, this was not tested as part of this project.

# Tools Required
Following are the tools that were used for this project:
* Soldering iron (https://www.amazon.com/gp/product/B072FNVQZ6/ref=oh_aui_detailpage_o06_s00?ie=UTF8&psc=1)
* Solder (https://www.amazon.com/gp/product/B071G1J3W6/ref=oh_aui_detailpage_o02_s00?ie=UTF8&psc=1)
* Tweezers (https://www.amazon.com/gp/product/B072XKVSQ5/ref=oh_aui_detailpage_o03_s01?ie=UTF8&psc=1)
* Light and magnifying station (https://www.amazon.com/gp/product/B077337GC5/ref=oh_aui_detailpage_o02_s00?ie=UTF8&psc=1)
* Wire cutters and strippers are also required (https://www.amazon.com/VISE-GRIP-Multi-Tool-Stripper-Crimper-2078309/dp/B000JNNWQ2/ref=sr_1_3?s=power-hand-tools&ie=UTF8&qid=1523205661&sr=1-3&keywords=wire+strippers&dpID=31uY1TVTmaL&preST=_SY300_QL70_&dpSrc=srch)
