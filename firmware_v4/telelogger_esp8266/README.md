This sketch turns Freematics ONE device into a wireless vehicle data collector and transmitter. A ESP8266 xBee module is required to be plugged in Freematics ONE. The live data of a car, including OBD-II, GPS and MEMS sensor data, is collected in realtime, cached in SRAM and pushed to a HTTP server by HTTP POST continuously.

Prerequisites
-------------

* Freematics ONE with ESP8266 - http://freematics.com/products/freematics-one
* Arduino IDE (for compiling and uploading the code)
* A wireless AP or hotspot with known SSID and password
* A car with OBD-II port
