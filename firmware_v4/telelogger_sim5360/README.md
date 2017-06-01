This Arduino sketch collects vehicle telematics data (OBD-II, GPS, motion sensors, network status, battery voltage) and transmits the collected data to Freematics Hub via UDP transport protocol. Live data is accessible by HTTP/JSON via Freematics Hub.

Prerequisites
-------------

* Freematics ONE with SIM5360E
* A micro SIM card with active data plan
* Arduino IDE, PlatformIO or [Freematics Arduino Builder](http://freematics.com/software/arduino-builder/) for compiling and uploading the sketch
* A car with OBD-II port or [Freematics OBD-II Emulator](http://freematics.com/pages/products/freematics-obd-emulator-mk2/)
* A Freematics Hub server key if you want to build your own service (demo works without key)
