This Arduino sketch is designed for running on [Freematics Esprit](https://freematics.com/products/freematics-esprit/) to log GPS data and provide access for logged data through WiFi and BLE.

Storage
-------

By default, ESP32's internal flash is used as SPIFFS for data storage. Due to limited flash size, rotation logging is implemented. MicroSD data logging is also supported.

HTTP Server
-----------

The sketch runs a [HTTP server](https://github.com/stanleyhuangyc/Freematics/tree/master/libraries/httpd) through ESP32's WiFi (AP and/or station). Implemented HTTP APIs provide remote access for device running status, statistics and logged/logging data.

Implemented HTTP APIs:

* /api/info - device info
* /api/live - live data (OBD/GPS/MEMS)
* /api/list - list of log files
* /api/log/[file #] - content of CSV format log file
* /api/data/[file #]?pid=[PID # in HEX] - JSON array of PID data

BLE
---

A BLE GATT server is brought up to allow BLE capable mobile device to connect and receive some status and stats wirelessly.

Prerequisites
-------------

* [Freematics Esprit](https://freematics.com/products/freematics-esprit/)
* [GPS Receiver for Arduino](https://freematics.com/store/index.php?route=product/product&path=20&product_id=55)
* [PlatformIO](http://platformio.org/), [Arduino IDE](https://github.com/espressif/arduino-esp32#installation-instructions) or [Freematics Arduino Builder](https://freematics.com/software/arduino-builder) for compiling and uploading code
