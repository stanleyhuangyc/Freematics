/*************************************************************************
* Arduino Library for Freematics ONE
* Distributed under BSD license
* Visit https://freematics.com/products/freematics-one for more information
* (C)2012-2018 Stanley Huang <stanley@freematics.com.au>
*************************************************************************/

#pragma once

#include "FreematicsBase.h"
#include "FreematicsMEMS.h"

#define OBD_TIMEOUT_SHORT 1000 /* ms */
#define OBD_TIMEOUT_LONG 10000 /* ms */
#define OBD_SERIAL_BAUDRATE 38400

#define GPS_READ_TIMEOUT 200 /* ms */
#define GPS_INIT_TIMEOUT 1000 /* ms */

// Mode 1 PIDs
#define PID_ENGINE_LOAD 0x04
#define PID_COOLANT_TEMP 0x05
#define PID_SHORT_TERM_FUEL_TRIM_1 0x06
#define PID_LONG_TERM_FUEL_TRIM_1 0x07
#define PID_SHORT_TERM_FUEL_TRIM_2 0x08
#define PID_LONG_TERM_FUEL_TRIM_2 0x09
#define PID_FUEL_PRESSURE 0x0A
#define PID_INTAKE_MAP 0x0B
#define PID_RPM 0x0C
#define PID_SPEED 0x0D
#define PID_TIMING_ADVANCE 0x0E
#define PID_INTAKE_TEMP 0x0F
#define PID_MAF_FLOW 0x10
#define PID_THROTTLE 0x11
#define PID_AUX_INPUT 0x1E
#define PID_RUNTIME 0x1F
#define PID_DISTANCE_WITH_MIL 0x21
#define PID_COMMANDED_EGR 0x2C
#define PID_EGR_ERROR 0x2D
#define PID_COMMANDED_EVAPORATIVE_PURGE 0x2E
#define PID_FUEL_LEVEL 0x2F
#define PID_WARMS_UPS 0x30
#define PID_DISTANCE 0x31
#define PID_EVAP_SYS_VAPOR_PRESSURE 0x32
#define PID_BAROMETRIC 0x33
#define PID_CATALYST_TEMP_B1S1 0x3C
#define PID_CATALYST_TEMP_B2S1 0x3D
#define PID_CATALYST_TEMP_B1S2 0x3E
#define PID_CATALYST_TEMP_B2S2 0x3F
#define PID_CONTROL_MODULE_VOLTAGE 0x42
#define PID_ABSOLUTE_ENGINE_LOAD 0x43
#define PID_AIR_FUEL_EQUIV_RATIO 0x44
#define PID_RELATIVE_THROTTLE_POS 0x45
#define PID_AMBIENT_TEMP 0x46
#define PID_ABSOLUTE_THROTTLE_POS_B 0x47
#define PID_ABSOLUTE_THROTTLE_POS_C 0x48
#define PID_ACC_PEDAL_POS_D 0x49
#define PID_ACC_PEDAL_POS_E 0x4A
#define PID_ACC_PEDAL_POS_F 0x4B
#define PID_COMMANDED_THROTTLE_ACTUATOR 0x4C
#define PID_TIME_WITH_MIL 0x4D
#define PID_TIME_SINCE_CODES_CLEARED 0x4E
#define PID_ETHANOL_FUEL 0x52
#define PID_FUEL_RAIL_PRESSURE 0x59
#define PID_HYBRID_BATTERY_PERCENTAGE 0x5B
#define PID_ENGINE_OIL_TEMP 0x5C
#define PID_FUEL_INJECTION_TIMING 0x5D
#define PID_ENGINE_FUEL_RATE 0x5E
#define PID_ENGINE_TORQUE_DEMANDED 0x61
#define PID_ENGINE_TORQUE_PERCENTAGE 0x62
#define PID_ENGINE_REF_TORQUE 0x63

typedef enum {
    PROTO_AUTO = 0,
    PROTO_ISO_9141_2 = 3,
    PROTO_KWP2000_5KBPS = 4,
    PROTO_KWP2000_FAST = 5,
    PROTO_CAN_11B_500K = 6,
    PROTO_CAN_29B_500K = 7,
    PROTO_CAN_29B_250K = 8,
    PROTO_CAN_11B_250K = 9,
} OBD_PROTOCOLS;

// states
typedef enum {
    OBD_DISCONNECTED = 0,
    OBD_CONNECTING = 1,
    OBD_CONNECTED = 2,
	OBD_FAILED = 3
} OBD_STATES;

// IO PIN modes
typedef enum {
	IO_PIN_INPUT = 0,
	IO_PIN_OUTPUT = 1
} IO_PIN_MODE;

uint16_t hex2uint16(const char *p);
uint8_t hex2uint8(const char *p);

#define SPI_PIN_CS 7
#define SPI_PIN_READY 6

#define TARGET_OBD 0
#define TARGET_GPS 1
#define TARGET_BEE 2
#define TARGET_RAW 3

class COBDSPI : public CFreematics {
public:
	byte begin();
	void end();
    // initialize OBD-II connection
	bool init(OBD_PROTOCOLS protocol = PROTO_AUTO);
	// reset OBD
	void reset();
	// read specified OBD-II PID value
	bool readPID(byte pid, int& result);
	// test PID reading
	bool testPID(byte pid);
	// send AT command and receive response
	byte sendCommand(const char* cmd, char* buf, int bufsize, unsigned int timeout = OBD_TIMEOUT_LONG);
	// initialize GPS (set baudrate to 0 to power off GPS)
	bool gpsInit(unsigned long baudrate = 115200L);
	// get parsed GPS data
	bool gpsGetData(GPS_DATA* gdata);
	// get GPS NMEA data
	int gpsGetRawData(char* buf, int bufsize);
	// send command string to GPS
	void gpsSendCommand(const char* cmd);
	// start xBee UART communication
	bool xbBegin(unsigned long baudrate = 115200L);
	// read data to xBee UART
	int xbRead(char* buffer, int bufsize);
	// send data to xBee UART
	void xbWrite(const char* cmd);
	// receive data from xBee UART
	int xbReceive(char* buffer, int bufsize, unsigned int timeout, const char** expected = 0, byte expectedCount = 0);
	// purge xBee UART buffer
	void xbPurge();
	// toggle xBee module power
	void xbTogglePower();
	// set I/O pin mode (pin: 1/2, mode: input/output)
	bool ioConfig(byte pin, IO_PIN_MODE mode);
	// set I/O pin level (pin: 1/2, level: 0/1)
	bool ioWrite(byte pin, byte level);
	// get I/O pin level (bit 0 for pin 1, bit 1 for pin 2)
	byte ioRead();
	// get connection state
	OBD_STATES getState() { return m_state; }
	// read diagnostic trouble codes (return number of DTCs read)
	byte readDTC(uint16_t codes[], byte maxCodes = 1);
	// clear diagnostic trouble code
	void clearDTC();
	// get battery voltage (works without ECU)
	float getVoltage();
	// get VIN as a string, buffer length should be >= OBD_RECV_BUF_SIZE
	bool getVIN(char* buffer, int bufsize);
	// determine if the PID is supported
	bool isValidPID(byte pid);
	// enter low power mode
	void lowPowerMode();
	// get firmware version
	byte getVersion();
	// set current PID mode
	byte dataMode = 1;
	// occurrence of errors
	byte errors = 0;
	// bit map of supported PIDs
	byte pidmap[4 * 4] = {0};
protected:
	// write data to SPI bus
	void write(const char* s);
	// receive data from SPI bus
	int receive(char* buffer, int bufsize, unsigned int timeout = OBD_TIMEOUT_SHORT);
	// set SPI data target
	void debugOutput(const char* s);
	int normalizeData(byte pid, char* data);
	virtual void idleTasks() { delay(1); }
	OBD_STATES m_state = OBD_DISCONNECTED;
	byte m_target = TARGET_OBD;
private:
	uint8_t getPercentageValue(char* data)
	{
		return (uint16_t)hex2uint8(data) * 100 / 255;
	}
	uint16_t getLargeValue(char* data)
	{
		return hex2uint16(data);
	}
	uint8_t getSmallValue(char* data)
	{
		return hex2uint8(data);
	}
	int16_t getTemperatureValue(char* data)
	{
		return (int)hex2uint8(data) - 40;
	}
	byte checkErrorMessage(const char* buffer);
};
