This Arduino sketch is designed for running on [Freematics ONE+](https://freematics.com/products/freematics-one-plus/) and [Freematics Esprit development board](https://freematics.com/products/freematics-esprit-obd-kit/), to collect vehicle telemetry data from OBD-II, GPS and motion sensor and transmit the collected data to remote server running [Freematics Hub](https://freematics.com/hub) software in realtime. It also has a mechansim for executing and responding to commands sent from serverside.

Data Collection
---------------

The sketch can collect following data.

* Vehicle OBD-II PIDs data (from OBD port)
* Battery voltage (from OBD port)
* Geolocation data (from connected GPS receiver)
* Acceleration data (from built-in motion sensor)
* Orientation data (computed from motion sensor data)
* CPU temperature (from ESP32 built-in sensor)

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

Remote Commands
---------------

Commands can be sent to Freematics ONE+ to execute with results responded, through serial terminal (ended by \n), BLE (as a  message) or [Freematics Hub API](https://freematics.com/hub/api/) (remotely). Currently following commands are implemented.

* LED [0/1/2] - setting device LED status (0:auto 1:always off 2:always on)
* REBOOT - performing a reboot immediately
* STANDBY - entering standby mode immediately
* WAKEUP - waking up the device from standby mode
* SET INTERVAL [interval in ms] - setting data sending interval
* SET SYNC [interval in ms] - setting server sync checking interval
* STATS - returning some stats
* OBD [PID] - querying and returning specified OBD-II PID value (raw data)


Prerequisites
-------------

* [Freematics ONE+](https://freematics.com/products/freematics-one-plus/)
* SIM800 or SIM5360 bee module and a micro SIM card if cellular network required
* [Freematics Hub server software](https://freematics.com/hub) (freeware)
* [PlatformIO](http://platformio.org/), [Arduino IDE](https://github.com/espressif/arduino-esp32#installation-instructions) or [Freematics Arduino Builder](https://freematics.com/software/arduino-builder) for compiling and uploading code
* A car with OBD-II port or [Freematics OBD-II Emulator](https://freematics.com/products/freematics-obd-emulator-mk2/)
