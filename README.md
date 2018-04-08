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

# Parts list
To be provided
