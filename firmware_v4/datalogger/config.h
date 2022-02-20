#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

// enable(1)/disable(0) command line interface (for serial control or Freematics App via BLE)
#define ENABLE_CLI 1

// enable(1)/disable(0) data logging (if SD card is present)
#define ENABLE_DATA_LOG 1

// file size limit
#define MAX_DATA_FILE_SIZE 1024 /* KB */

// enable(1)/disable(0) MEMS sensor
#define USE_MEMS 1

// enable(1)/disable(0) external GPS receiver
#define USE_GPS 1

// enable(1)/disable(0) cellular module (SIM5360) internal GPS
#define USE_CELL_GPS 0

// GPS parameters
#define GPS_SERIAL_BAUDRATE 115200L

// time for OBD-II attempt before entering non-OBD mode
#define OBD_ATTEMPT_TIME 180 /* seconds */

// motion threshold for waking up
#define WAKEUP_MOTION_THRESHOLD 0.25f /* in unit of G */

// engine jumpstart voltage
#define JUMPSTART_VOLTAGE 14 /* V */

#define SD_CS_PIN 10

#endif // CONFIG_H_INCLUDED
