This Arduino sketch is designed for running on [Freematics ONE+](https://freematics.com/products/freematics-one-plus/) and [Freematics Esprit development board](https://freematics.com/products/freematics-esprit-obd-kit/), to collect vehicle telemetry data from OBD-II, GPS and motion sensor and transmit the collected data to remote server running [Freematics Hub](https://freematics.com/hub) software in realtime. It also has a mechansim for executing and responding to commands sent from serverside.

Data Collection
---------------

The sketch collects following data.

* OBD-II data (from a OBD-II certified vehicle)
* GPS data (from either internal or external GPS)
* MEMS data (from built-in MPU-9250 motion sensor)

Data Transmissions
------------------

The sketch implements data transmissions with following networking hardware.

* BLE (ESP32 built-in)
* WIFI (ESP32 built-in)
* GSM/GPRS (optional SIM800 bee module)
* WCDMA (optional SIM5360 bee module)

Data Storage
------------

The sketch implements following data storage.

* MicroSD card storage
* ESP32 built-in Flash memory storage (under development)

Prerequisites
-------------

* [Freematics ONE+](https://freematics.com/products/freematics-one-plus/)
* SIM800 or SIM5360 bee module and a micro SIM card if cellular network required
* [Freematics Hub server software](https://freematics.com/hub) (freeware)
* [PlatformIO](http://platformio.org/), [Arduino IDE](https://github.com/espressif/arduino-esp32#installation-instructions) or [Freematics Arduino Builder](https://freematics.com/software/arduino-builder) for compiling and uploading code
* A car with OBD-II port or [Freematics OBD-II Emulator](https://freematics.com/products/freematics-obd-emulator-mk2/)
