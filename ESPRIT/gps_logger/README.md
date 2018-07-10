This Arduino sketch is designed for [Freematics ESP32 GNSS Kit](https://freematics.com/products/esp32-gnss-kit/) to work as a GPS GPS data logger and Traccar client while also providing access for real-time and logged data over WiFi.

Storage
-------

By default, ESP32's internal flash is used as SPIFFS for data storage. Due to limited flash size, rotation logging is implemented. MicroSD data logging is also supported.

HTTP Server
-----------

The sketch runs a [HTTP server](https://github.com/stanleyhuangyc/Freematics/tree/master/libraries/httpd) through ESP32's WiFi (AP and/or station). Implemented HTTP APIs provide remote access for device running status, statistics and logged/logging data.

Implemented HTTP APIs:

* /api/info - device info
* /api/live - live data
* /api/list - list of log files
* /api/log/[file #] - content of CSV format log file
* /api/data/[file #]?pid=[PID # in HEX] - filtered data in JSON array

Traccar Client
--------------

A [Traccar](https://www.traccar.org/) client is implemented. Simply change the host server address and device ID settings in config.h to make the client push data to your Traccar server.

Prerequisites
-------------

* [Freematics Esprit](https://freematics.com/products/freematics-esprit/)
* [GPS Receiver for Arduino](https://freematics.com/store/index.php?route=product/product&path=20&product_id=55)
* [PlatformIO](http://platformio.org/), [Arduino IDE](https://github.com/espressif/arduino-esp32#installation-instructions) or [Freematics Arduino Builder](https://freematics.com/software/arduino-builder) for compiling and uploading code

Galleries
---------

![Freematics ESP32 GNSS Kit](https://freematics.com/pages/wp-content/uploads/2018/07/esprit_gps_kit_2.jpg)

![Freematics ESP32 GNSS Kit](https://freematics.com/pages/wp-content/uploads/2018/07/esprit_gps_kit_3.jpg)
