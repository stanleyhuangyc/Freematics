/*************************************************************************
* Arduino Library for Freematics ONE
* Distributed under BSD license
* Visit http://freematics.com/products/freematics-one for more information
* (C)2012-2016 Stanley Huang <stanleyhuangyc@gmail.com>
*************************************************************************/

#include "FreematicsMPU6050.h"
#include "FreematicsMPU9250.h"

#define OBD_TIMEOUT_SHORT 1000 /* ms */
#define OBD_TIMEOUT_LONG 10000 /* ms */
#define OBD_TIMEOUT_GPS 200 /* ms */
#define OBD_SERIAL_BAUDRATE 38400

#ifndef OBDUART
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168P__)
#define OBDUART Serial
#else
#define OBDUART Serial1
#endif
#endif

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

// non-OBD/custom PIDs (no mode number)
#define PID_GPS_LATITUDE 0xA
#define PID_GPS_LONGITUDE 0xB
#define PID_GPS_ALTITUDE 0xC
#define PID_GPS_SPEED 0xD
#define PID_GPS_HEADING 0xE
#define PID_GPS_SAT_COUNT 0xF
#define PID_GPS_TIME 0x10
#define PID_GPS_DATE 0x11
#define PID_ACC 0x20
#define PID_GYRO 0x21
#define PID_COMPASS 0x22
#define PID_MEMS_TEMP 0x23
#define PID_BATTERY_VOLTAGE 0x24

// custom PIDs for calculated data
#define PID_TRIP_DISTANCE 0x30

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

typedef struct {
    uint32_t date;
    uint32_t time;
    int32_t lat;
    int32_t lng;
    int16_t alt;
    uint8_t speed;
    uint8_t sat;
    int16_t heading;
} GPS_DATA;

uint16_t hex2uint16(const char *p);
uint8_t hex2uint8(const char *p);

#define SPI_PIN_CS 7
#define SPI_PIN_READY 6

#define TARGET_OBD 0
#define TARGET_GPS 1
#define TARGET_BEE 2
#define TARGET_RAW 3

class COBDSPI {
public:
	COBDSPI():dataMode(1),errors(0),m_state(OBD_DISCONNECTED) {}
	byte begin();
	// set SPI data target
	void setTarget(byte target) { m_target = target; }
	// receive data (up to 255 bytes) from SPI bus
	int receive(char* buffer, int bufsize, int timeout = OBD_TIMEOUT_SHORT);
	// read specified OBD-II PID value
	bool readPID(byte pid, int& result);
	// read multiple (up to 4) OBD-II PID value
	byte readPID(const byte pid[], byte count, int result[]);
	// write data to SPI bus
	void write(const char* s);
	void write(byte* data, int len);
	// send AT command and receive response
	byte sendCommand(const char* cmd, char* buf, byte bufsize, int timeout = OBD_TIMEOUT_LONG);
	// initialize GPS (set baudrate to 0 to power off GPS)
	bool initGPS(unsigned long baudrate = 115200L);
	// get parsed GPS data
	bool getGPSData(GPS_DATA* gdata);
	// get GPS NMEA data
	byte getGPSRawData(char* buf, byte bufsize);
	// send command string to GPS
	void sendGPSCommand(const char* cmd);
	// hardware sleep (timer counter will stop)
	void sleep(int seconds);
	// start xBee UART communication
	bool xbBegin(unsigned long baudrate = 115200L);
	// read data to xBee UART
	byte xbRead(char* buffer, byte bufsize, int timeout = 1000);
	// send data to xBee UART
	void xbWrite(const char* cmd);
	// receive data from xBee UART
	byte xbReceive(char* buffer, int bufsize, int timeout = 1000, const char* expected1 = 0, const char* expected2 = 0);	
	// purge xBee UART buffer
	void xbPurge();
	// initialize OBD-II connection
	bool init(OBD_PROTOCOLS protocol = PROTO_AUTO);
	// un-initialize OBD-II connection
	void end();
	// get connection state
	OBD_STATES getState() { return m_state; }
	// enter low power mode
	void enterLowPowerMode();
	// leave low power mode
	void leaveLowPowerMode();
	// set working protocol (default auto)
	bool setProtocol(OBD_PROTOCOLS h = PROTO_AUTO);
	// clear diagnostic trouble code
	void clearDTC();
	// get battery voltage (works without ECU)
	float getVoltage();
	// get VIN as a string, buffer length should be >= OBD_RECV_BUF_SIZE
	bool getVIN(char* buffer, byte bufsize);
	// send query for specified PID
	void sendQuery(byte pid);
	// retrive and parse the response of specifie PID
	bool getResult(byte& pid, int& result);
	// determine if the PID is supported
	bool isValidPID(byte pid);
	// set current PID mode
	byte dataMode;
	// occurrence of errors
	byte errors;
	// bit map of supported PIDs
	byte pidmap[4 * 4];
	byte version;
protected:
	char* getResponse(byte& pid, char* buffer, byte bufsize);
	void debugOutput(const char* s);
	int normalizeData(byte pid, char* data);
	virtual void dataIdleLoop() { delay(10); }
	OBD_STATES m_state;
private:
	byte getVersion();
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
	byte m_target;
};
