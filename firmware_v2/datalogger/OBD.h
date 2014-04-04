/*************************************************************************
* Arduino Library for OBD-II UART Adapter
* Distributed under GPL v2.0
* Visit http://freematics.com for more information
* (C)2012-2014 Stanley Huang <stanleyhuangyc@gmail.com>
*************************************************************************/

#include <Arduino.h>

#define OBD_TIMEOUT_SHORT 2000 /* ms */
#define OBD_TIMEOUT_LONG 7000 /* ms */
#define OBD_SERIAL_BAUDRATE 38400
#define OBD_RECV_BUF_SIZE 128

#ifndef OBDUART
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega644P__)
#define OBDUART Serial1
#else
#define OBDUART Serial
#endif
#endif

// Mode 1 PIDs
#define PID_ENGINE_LOAD 0x04
#define PID_COOLANT_TEMP 0x05
#define PID_FUEL_PRESSURE 0x0A
#define PID_INTAKE_MAP 0x0B
#define PID_RPM 0x0C
#define PID_SPEED 0x0D
#define PID_TIMING_ADVANCE 0x0E
#define PID_INTAKE_TEMP 0x0F
#define PID_MAF_FLOW 0x10
#define PID_THROTTLE 0x11
#define PID_RUNTIME 0x1F
#define PID_FUEL_LEVEL 0x2F
#define PID_DISTANCE 0x31
#define PID_BAROMETRIC 0x33
#define PID_CONTROL_MODULE_VOLTAGE 0x42
#define PID_ABSOLUTE_ENGINE_LOAD 0x43
#define PID_AMBIENT_TEMP 0x46
#define PID_ETHANOL_PERCENTAGE 0x52
#define PID_FUEL_RAIL_PRESSURE 0x59
#define PID_HYBRID_BATTERY_PERCENTAGE 0x5B
#define PID_ENGINE_OIL_TEMP 0x5C
#define PID_ENGINE_FUEL_RATE 0x5E
#define PID_ENGINE_TORQUE_PERCENTAGE 0x62
#define PID_ENGINE_REF_TORQUE 0x63

// states
#define OBD_DISCONNECTED 0
#define OBD_CONNECTING 1
#define OBD_CONNECTED 2

unsigned int hex2uint16(const char *p);
unsigned char hex2uint8(const char *p);

class COBD
{
public:
	COBD():dataMode(1),errors(0),m_state(OBD_DISCONNECTED) {}
	virtual void begin();
	virtual bool init(byte protocol = 0);
	virtual bool read(byte pid, int& result);
	virtual void sleep();
	virtual void wakeup();
	virtual void setProtocol(byte h = -1);
	// Query and GetResponse for advanced usage only
	virtual void sendQuery(byte pid);
	virtual bool getResult(byte& pid, int& result);
	bool isValidPID(byte pid);
	byte getState() { return m_state; }
	byte dataMode;
	byte errors;
	byte pidmap[4 * 4];
	byte vin[17];
protected:
	virtual char* getResponse(byte& pid, char* buffer);
	virtual byte receive(char* buffer = 0, int timeout = OBD_TIMEOUT_SHORT);
	virtual bool available();
	virtual char read();
	virtual void write(const char* s);
	virtual void write(char c);
	virtual void dataIdleLoop() {}
    void recover();
	void debugOutput(const char* s);
	int normalizeData(byte pid, char* data);
	byte m_state;
private:
	virtual uint8_t getPercentageValue(char* data)
	{
		return (uint16_t)hex2uint8(data) * 100 / 255;
	}
	virtual uint16_t getLargeValue(char* data)
	{
		return hex2uint16(data);
	}
	virtual uint8_t getSmallValue(char* data)
	{
		return hex2uint8(data);
	}
	virtual int16_t getTemperatureValue(char* data)
	{
		return (int)hex2uint8(data) - 40;
	}
};
