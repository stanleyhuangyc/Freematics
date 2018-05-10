/*************************************************************************
* Arduino Library for Freematics ONE/ONE+
* Distributed under BSD license
* Visit http://freematics.com/products/freematics-one for more information
* (C)2012-2017 Stanley Huang <support@freematics.com.au
*************************************************************************/

#include <Arduino.h>
#include "utility/OBD.h"

#define OBD_TIMEOUT_SHORT 1000 /* ms */
#define OBD_TIMEOUT_LONG 10000 /* ms */

#define SPI_PIN_CS 2
#define SPI_PIN_READY 13
#define SPI_FREQ 1000000

int dumpLine(char* buffer, int len);
uint16_t hex2uint16(const char *p);
byte hex2uint8(const char *p);

class COBD
{
public:
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
	// read multiple OBD-II PID values, return number of values obtained
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
	// determine if the PID is supported
	virtual bool isValidPID(byte pid);
	// get adapter firmware version
	virtual byte getVersion();
	// set current PID mode
	byte dataMode = 1;
	// occurrence of errors
	byte errors = 0;
	// bit map of supported PIDs
	byte pidmap[4 * 4] = {0};
	uint16_t pidWaitTime = OBD_TIMEOUT_SHORT;
protected:
	virtual char* getResponse(byte& pid, char* buffer, byte bufsize);
	virtual int receive(char* buffer, int bufsize, unsigned int timeout = OBD_TIMEOUT_SHORT);
	virtual void write(const char* s);
	virtual uint8_t getPercentageValue(char* data);
	virtual uint16_t getLargeValue(char* data);
	virtual uint8_t getSmallValue(char* data);
	virtual int16_t getTemperatureValue(char* data);
	virtual int normalizeData(byte pid, char* data);
	virtual byte checkErrorMessage(const char* buffer);
	virtual char* getResultValue(char* buf);
	virtual void recover();
	OBD_STATES m_state = OBD_DISCONNECTED;
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
	// write data to SPI bus (without header)
	void write(uint8_t* data, int bytes);
	// read specified OBD-II PID value
	bool readPID(byte pid, int& result);
	// send AT command and receive response
	int sendCommand(const char* cmd, char* buf, int bufsize, unsigned int timeout = OBD_TIMEOUT_LONG);
};
