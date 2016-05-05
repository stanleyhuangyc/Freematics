Firmware & Libraries for Project Freematics
===========================================

Freematics Vehicle Data Logger (http://freematics.com/pages/products/freematics-one/) is an Arduino compatible device in the form of a OBD-II dongle with main controller of ATmega328p and a bunch of useful peripherals including MEMS motion sensor, microSD seat, BLE module, ESP8266 WIFI module, SIM800L GSM/GPRS module, all accessible with a unified Arduino library.

Directory Descriptions
----------------------
firmware - firmware (Arduino sketch) for Freematics OBD-II Adapter (deprecated)

firmware_v2 - firmware (Arduino sketch) for Freematics OBD-II Data Logger V2 (deprecated)

firmware_v3 - firmware (Arduino sketch) for Freematics OBD-II Data Logger V3 (deprecated)

firmware_v4 - firmware (Arduino sketch) for Freematics OBD-II Data Logger V4 (Freematics ONE)

libraries - Arduino libraries used in firmware

How to view logged data
-----------------------
Data2KML (http://freematics.com/pages/software/data2kml/) is an open-source command line utility which converts data logged by obdlogger or megalogger to KML file loading in Google Earth.

A web service (http://freematics.com/chart/) is provided to view data logged by obdlogger or megalogger.
