This Arduino sketch is designed for running on [Freematics ONE+](https://freematics.com/products/freematics-one-plus/) to collect vehicle telemetry data from OBD-II, GPS and motion sensor, store the collected data in Flash or microSD card and provide data access through WiFi and BLE.

Data Collection
---------------

The sketch collects and logs data including following.

* Vehicle OBD-II PIDs data (from OBD port)
* Vehicle battery voltage (from OBD port)
* Geolocation data (from internal GNSS or connected GNSS receiver)
* Acceleration data (from internal motion sensor)

Geolocation
-----------

When GNSS receiver is connected and detected, geolocation data will be obtained and logged.

Orientation
-----------

The device orientation (yaw, pitch roll) can be calculated from collected motion sensor data with quanterion algorithm. This reflects the orientation of the vehicle if the device is firmly fixed to the vehicle. This feature consumes extra computation power and is disabled by default.

Storage
-------

MicroSD is used as data storage by default. Alternatively ESP32's internal flash can be used for data storage with SPIFFS. Simple rotation logging is implemented due to limited storage size.

HTTP Server
-----------

A multiple-connection HTTP server [HTTP server](https://github.com/stanleyhuangyc/Freematics/blob/master/libraries/httpd) runs over ESP32's WiFi (AP and/or station). A set of REST API provides remote access to device status, statistics and real-time data and logged data.

Implemented HTTP APIs:

* /api/info - device info
* /api/live - live data (OBD/GPS/MEMS)
* /api/list - list of log files
* /api/log/[file #] - content of CSV format log file
* /api/delete/[file #] - delete file
* /api/data/[file #]?pid=[PID # in HEX] - JSON array of PID data

BLE
---

A BLE SPP server is implemented in [FreematicsPlus](https://github.com/stanleyhuangyc/Freematics/blob/master/libraries/FreematicsPlus) library. To enable BLE support, change ENABLE_BLE to 1 [config.h](config.h). This will enable remote control and data monitoring via [Freematics Controller App](https://freematics.com/software/freematics-controller/).


Prerequisites
-------------

* [Freematics ONE+](https://freematics.com/products/freematics-one-plus/)
* [PlatformIO](http://platformio.org/), [Arduino IDE](https://github.com/espressif/arduino-esp32#installation-instructions) or [Freematics Builder](https://freematics.com/software/arduino-builder) for compiling and uploading code
* A car with OBD-II port or [Freematics OBD-II Emulator](https://freematics.com/products/freematics-obd-emulator-mk2/)

