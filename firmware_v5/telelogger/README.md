This Arduino sketch is designed for running on Freematics ONE+ and Freematics Esprit development board, to collect vehicle telemetry data from OBD-II, GPS and motion sensor and transmit the collected data to remote server running [Freematics Hub](https://freematics.com/hub) software in realtime. It also has a mechansim for executing and responding to commands sent from serverside.

Data Collection
---------------

The sketch implements telematics data collection for Freematics ONE+ including

* OBD-II data (from a OBD-II certified vehicle)
* GPS data (from either internal or external GPS)
* MEMS data (from built-in MPU-9250 motion sensor)

Data Transmissions
------------------

The sketch implements telematics data transmissions for Freematics ONE+ including

* BLE (ESP32 built-in)
* WIFI (ESP32 built-in)
* 2G cellular network (plugged-in SIM800 bee module)
* 3G cellular network (plugged-in SIM5360 bee module)
* Serial USB

Data Storage
------------

The sketch implements telematics data storage for Freematics ONE+ including

* RAM cache
* ESP32 built-in Flash memory storage
* MicroSD card storage

Prerequisites
-------------

* Freematics ONE+
* SIM800 or SIM5360 bee module if cellular network required
* PlatformIO or Arduino IDE
* A Freematics Hub server key
* A micro SIM card
* A car with OBD-II port
