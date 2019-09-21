This Arduino sketch is designed for running on [Freematics ONE+](https://freematics.com/products/freematics-one-plus/) to collect vehicle telemetry data from OBD-II, GPS and motion sensor and transmit the collected data to a remote server running [Freematics Hub](https://freematics.com/hub) software in realtime. It also has a mechansim for executing and responding to commands sent from serverside.

Data Collection
---------------

The sketch collects following data.

* Vehicle OBD-II PIDs data (from OBD port)
* Battery voltage (from OBD port)
* Geolocation data (from cellular module's internal GNSS or external GNSS receiver)
* Acceleration data (from built-in MEMS sensor)
* Orientation data (computed from motion sensor data)
* Device temperature (from MEMS sensor or ESP32 built-in sensor)

Data Transmissions
------------------

Data transmission over UDP and HTTP protocols are implemented with following hardware.

* WiFi (ESP32 built-in)
* WiFi Mesh (ESP-MDF for ESP32)
* GSM/GPRS (SIM800)
* 3G WCDMA (SIM5360)
* 4G LTE (SIM7600)

UDP mode implements a client for [Freematics Hub](https://freematics.com/hub/). HTTP mode implements a client for [Traccar](https://www.traccar.org) under [OsmAnd](https://www.traccar.org/osmand/) protocol.

Data Storage
------------

Following types of data storage are supported.

* MicroSD card storage
* ESP32 built-in Flash memory storage (SPIFFS)

Remote Commands
---------------

Commands can be sent to Freematics ONE+ for execution with results obtained, through serial terminal or by [Freematics Hub API](https://freematics.com/hub/api/) (remotely). Currently following commands are implemented.

* LED [0/1/2] - setting device LED status (0:auto 1:always off 2:always on)
* REBOOT - performing a reboot immediately
* STANDBY - entering standby mode immediately
* WAKEUP - waking up the device from standby mode
* SET SYNC [interval in ms] - setting server sync checking interval
* STATS - returning some stats
* OBD [PID] - querying and returning specified OBD-II PID value (raw data)

Viewing Trip Data
-----------------

Once the sketch is running and data is being submitted to hub.freematics.com, you can open https://hub.freematics.com from any of your devices and enter your device ID (displayed in serial output) to view real-time data and history trip data.

![Freematics Hub Dashboard](https://freematics.com/pages/wp-content/uploads/2019/01/freematics_hub_dash-1024x576.png)

Prerequisites
-------------

* [Freematics ONE+](https://freematics.com/products/freematics-one-plus/) or [Freematics ONE+ Model B](https://freematics.com/products/freematics-one-plus-model-b/)
* A micro SIM card if cellular network required
* [PlatformIO](http://platformio.org/), [Arduino IDE](https://github.com/espressif/arduino-esp32#installation-instructions) or [Freematics Arduino Builder](https://freematics.com/software/arduino-builder) for compiling and uploading code
