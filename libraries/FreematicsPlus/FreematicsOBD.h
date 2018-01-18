/*************************************************************************
* Arduino Library for Freematics ONE/ONE+
* Distributed under BSD license
* Visit http://freematics.com/products/freematics-one for more information
* (C)2012-2017 Stanley Huang <support@freematics.com.au
*************************************************************************/

#include <Arduino.h>
#include "utility/OBD.h"

#ifndef ARDUINO_ARCH_AVR
#define sprintf_P sprintf
#endif

#define OBD_TIMEOUT_SHORT 2000 /* ms */
#define OBD_TIMEOUT_LONG 10000 /* ms */

#define SPI_PIN_CS 2
#define SPI_PIN_READY 13

int dumpLine(char* buffer, int len);
uint16_t hex2uint16(const char *p);
byte hex2uint8(const char *p);

class COBD
{
public:
	COBD():dataMode(1),errors(0),m_state(OBD_DISCONNECTED) {}
	// begin serial UART
	virtual byte begin();
	// terminate communication channel
	virtual void end();
	// initialize OBD-II connection
	virtual bool init(OBD_PROTOCOLS protocol = PROTO_AUTO);
	// reset OBD-II connection
	virtual void reset();
	// un-initialize OBD-II connection
	virtual void uninit();
	// set serial baud rate
	virtual bool setBaudRate(unsigned long baudrate);
	// get connection state
	virtual OBD_STATES getState() { return m_state; }
	// read specified OBD-II PID value
	virtual bool readPID(byte pid, int& result);
	// read multiple (up to 8) OBD-II PID values, return number of values obtained
	virtual byte readPID(const byte pid[], byte count, int result[]);
	// set device into low power mode
	virtual void enterLowPowerMode();
	// wake up device from low power mode
	virtual void leaveLowPowerMode();
	// send AT command and receive response (return bytes received)
	virtual byte sendCommand(const char* cmd, char* buf, byte bufsize, int timeout = OBD_TIMEOUT_LONG);
	// read diagnostic trouble codes (return number of DTCs read)
	virtual byte readDTC(uint16_t codes[], byte maxCodes = 1);
	// clear diagnostic trouble code
	virtual void clearDTC();
	// get battery voltage (works without ECU)
	virtual float getVoltage();
	// get VIN as a string, buffer length should be >= OBD_RECV_BUF_SIZE
	virtual bool getVIN(char* buffer, byte bufsize);
	// send query for specified PID
	virtual void sendQuery(byte pid);
	// retrive and parse the response of specifie PID
	virtual bool getResult(byte& pid, int& result);
	// determine if the PID is supported
	virtual bool isValidPID(byte pid);
	// get adapter firmware version
	virtual byte getVersion();
	// set current PID mode
	byte dataMode;
	// occurrence of errors
	byte errors;
	// bit map of supported PIDs
	byte pidmap[4 * 4];
protected:
	virtual char* getResponse(byte& pid, char* buffer, byte bufsize);
	virtual int receive(char* buffer, int bufsize, unsigned int timeout = OBD_TIMEOUT_SHORT);
	virtual void write(const char* s);
	virtual void idleTask() {}
	void recover();
	void debugOutput(const char* s);
	int normalizeData(byte pid, char* data);
	OBD_STATES m_state;
private:
	byte checkErrorMessage(const char* buffer);
	virtual uint8_t getPercentageValue(char* data);
	virtual uint16_t getLargeValue(char* data);
	virtual uint8_t getSmallValue(char* data);
	virtual int16_t getTemperatureValue(char* data);
	char* getResultValue(char* buf);
};

class COBDSPI : public COBD {
public:
	byte begin();
	// un-initialize OBD-II connection
	void end();
	// set SPI data target
	int receive(char* buffer, int bufsize, unsigned int timeout = OBD_TIMEOUT_SHORT);
	// write data to SPI bus
	void write(const char* s);
	void write(byte* data, int len);
	// send AT command and receive response
	int sendCommand(const char* cmd, char* buf, int bufsize, unsigned int timeout = OBD_TIMEOUT_LONG);
	// delay specified number of ms while still receiving and processing GPS data
	void sleep(unsigned int ms);
};
