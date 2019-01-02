This Arduino sketch is designed for running on [Freematics ONE+](https://freematics.com/products/freematics-one-plus/) and [Freematics Esprit development board](https://freematics.com/products/freematics-esprit-obd-kit/), to collect vehicle telemetry data from OBD-II, GPS and motion sensor and transmit the collected data to remote server running [Freematics Hub](https://freematics.com/hub) software in realtime. It also has a mechansim for executing and responding to commands sent from serverside.

Data Collection
---------------

The sketch collects following data.

* Vehicle OBD-II PIDs data (from OBD port)
* Battery voltage (from OBD port)
* Geolocation data (from cellular module's internal GNSS or external GNSS receiver)
* Acceleration data (from built-in motion sensor)
* Orientation data (computed from motion sensor data)
* CPU temperature (from ESP32 built-in sensor)

Data Transmissions
------------------

The sketch implements data transmissions with following networking hardware.

* WiFi(ESP32 built-in)
* GSM/GPRS (SIM800)
* 3G WCDMA (SIM5360)
* 4G LTE (SIM7600)

Data Storage
------------

The sketch implements following data storage.

* MicroSD card storage
* ESP32 built-in Flash memory storage (SPIFFS)

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

Viewing Live Data
-----------------

Once the sketch is running and data is being submitted to hub.freematics.com, you can open https://hub.freematics.com/dash from any of your devices and enter your device ID (displayed in serial output) to view the live data instantly.

![Freematics Hub Dashboard](https://freematics.com/pages/wp-content/uploads/2019/01/freematics_hub_dash-1024x576.png)

Prerequisites
-------------

* [Freematics ONE+](https://freematics.com/products/freematics-one-plus/) or [Freematics ONE+ Model B](https://freematics.com/products/freematics-one-plus-model-b/) 
* SIM800 or SIM5360 bee module and a micro SIM card if cellular network required
* [PlatformIO](http://platformio.org/), [Arduino IDE](https://github.com/espressif/arduino-esp32#installation-instructions) or [Freematics Arduino Builder](https://freematics.com/software/arduino-builder) for compiling and uploading code
* A car with OBD-II port or [Freematics OBD-II Emulator](https://freematics.com/products/freematics-obd-emulator-mk2/)
