This Arduino sketch is designed for running on [Freematics ONE+](https://freematics.com/products/freematics-one-plus/) and the [ESP32 Arduino Telematics Kit](https://freematics.com/products/freematics-esprit-obd-kit/), to collect vehicle telemetry data from OBD-II, GPS and motion sensor, store the collected data in Flash or microSD card and provide access for data through WiFi and BLE.

Data Collection
---------------

The sketch collects follow data.

* Vehicle OBD-II PIDs data (from OBD port)
* Battery voltage (from OBD port)
* Geolocation data (from connected GPS receiver)
* Acceleration data (from built-in motion sensor)
* Orientation data (computed from motion sensor data fusion)

GPS
---

When GPS is detected, the sketch waits for GPS singal for a while on startup before starting data logging. Once GPS time is obtained, the data file will be created with the date and time as file name.

Orientation
-----------

The device orientation (yaw, pitch roll) can be calculated from collected motion sensor data with quanterion algorithm. This normally reflects the orientation of the vehicle if the device is fixed to vehicle (e.g. firmly plugged in OBD port)  This feature consumes extra computation power and is disabled by default.

Storage
-------

By default, ESP32's internal flash is used as SPIFFS for data storage. Due to limited flash size, rotation logging is implemented. MicroSD data logging is also supported.

HTTP Server
-----------

The sketch runs a [HTTP server](https://github.com/stanleyhuangyc/Freematics/tree/master/libraries/httpd) through ESP32's WiFi (AP and/or station). Implemented HTTP APIs provide remote access for device running status, statistics and logged/logging data.

Implemented HTTP APIs:

* /api/info - device info
* /api/list - list of log files
* /api/log - raw CSV format log file
* /api/data - timestamped PID data in JSON array

BLE
---

A BLE GATT server is brought up to allow BLE capable mobile device to connect and receive some status and stats wirelessly.

Prerequisites
-------------

* [Freematics ONE+](https://freematics.com/products/freematics-one-plus/)
* [PlatformIO](http://platformio.org/), [Arduino IDE](https://github.com/espressif/arduino-esp32#installation-instructions) or [Freematics Arduino Builder](https://freematics.com/software/arduino-builder) for compiling and uploading code
* A car with OBD-II port or [Freematics OBD-II Emulator](https://freematics.com/products/freematics-obd-emulator-mk2/)

