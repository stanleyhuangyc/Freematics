This Arduino sketch is designed for [Freematics ONE+](https://freematics.com/products/freematics-one-plus/) to collect vehicle telemetry data from OBD, GPS, motion sensor, to log the data in local storage and to transmit the data to a remote server in real-time.

Data Collection
---------------

The sketch collects following data.

* Vehicle OBD PIDs data (from OBD port)
* Battery voltage (from OBD port)
* Geolocation data (from cellular module's internal GNSS or external GNSS receiver)
* Acceleration data (from built-in MEMS sensor)
* Device temperature (from MEMS sensor or ESP32 built-in sensor)

Data Transmission
-----------------

Data transmission over UDP and HTTP(s) protocols are implemented with following hardware.

* WiFi (ESP32 built-in)
* WiFi Mesh (ESP-MDF for ESP32)
* 3G WCDMA (SIM5360)
* 4G LTE CAT-4 (SIM7600)
* 4G LTE CAT-M1 (SIM7070)

UDP mode implements a full telemetry client for [Freematics Hub](https://hub.freematics.com) and [Traccar](https://www.traccar.org). HTTP mode implements [OsmAnd](https://www.traccar.org/osmand/) protocol.

Data Storage
------------

Following types of data storage are supported.

* MicroSD card storage
* ESP32 built-in Flash memory storage (SPIFFS)

BLE & App
---------

A BLE SPP server is implemented in [FreematicsPlus](https://github.com/stanleyhuangyc/Freematics/blob/master/libraries/FreematicsPlus) library. To enable BLE support, change ENABLE_BLE to 1 [config.h](config.h). This will enable remote control and data monitoring via [Freematics Controller App](https://freematics.com/software/freematics-controller/).

Prerequisites
-------------

* Freematics ONE+ [Model A](https://freematics.com/products/freematics-one-plus/), [Model B](https://freematics.com/products/freematics-one-plus-model-b/), [Model H](https://freematics.com/products/freematics-one-plus-model-h/)
* A micro SIM card if cellular network connectivity required
* [PlatformIO](http://platformio.org/), [Arduino IDE](https://github.com/espressif/arduino-esp32#installation-instructions) or [Freematics Builder](https://freematics.com/software/arduino-builder) for compiling and uploading code
