This Arduino sketch collects vehicle telemetry data including ECU, geolocation, accelerometer, battery voltage and transmits the collected data to Freematics Hub. Live data and historic data are accessible from Freematics Hub server by HTTP/JSON.

Prerequisites
-------------

* Freematics ONE with SIM5360E or SIM5360A
* GPS antenna for SIM5360
* A micro SIM card (e.g. Hologram Global IoT SIM)
* Arduino IDE, PlatformIO or [Freematics Arduino Builder](http://freematics.com/software/arduino-builder/) for compiling and uploading the sketch
* A car with OBD-II port or [Freematics OBD-II Emulator](http://freematics.com/pages/products/freematics-obd-emulator-mk2/)
