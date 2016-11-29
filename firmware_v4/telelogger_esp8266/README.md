This sketch turns a Freematics ONE device into a wireless (WIFI) vehicle data collector and transmitter. The live data of a car, including OBD-II, GPS and MEMS sensor data, is collected in realtime, cached in SRAM and pushed to a HTTP server by HTTP POST continuously.

Prerequisites
-------------

* Freematics ONE with ESP8266 WIFI module - http://freematics.com/products/freematics-one
* Arduino IDE (for compiling and uploading the code)
* A Freematics Hub server key
* A wireless AP or hotspot with known SSID and password
* A car with OBD-II port
