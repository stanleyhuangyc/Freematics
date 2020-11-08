/*************************************************************************
* Arduino Library for Freematics ONE/ONE+
* Distributed under BSD license
* Visit http://freematics.com/products/freematics-one for more information
* (C)2012-2017 Stanley Huang <support@freematics.com.au
*************************************************************************/

#include "utility/OBD.h"

#define OBD_TIMEOUT_SHORT 1000 /* ms */
#define OBD_TIMEOUT_LONG 10000 /* ms */

int dumpLine(char* buffer, int len);
uint16_t hex2uint16(const char *p);
byte hex2uint8(const char *p);

class COBD
{
public:
	void begin(CLink* link) { this->link = link; }
	// initialize OBD-II connection
	bool init(OBD_PROTOCOLS protocol = PROTO_AUTO);
	// reset OBD-II connection
	void reset();
	// un-initialize OBD-II connection
	void uninit();
	// set serial baud rate
	bool setBaudRate(unsigned long baudrate);
	// get connection state
	OBD_STATES getState() { return m_state; }
	// read specified OBD-II PID value
	bool readPID(byte pid, int& result);
	// read multiple OBD-II PID values, return number of values obtained
	byte readPID(const byte pid[], byte count, int result[]);
	// set device into low power mode
	void enterLowPowerMode();
	// wake up device from low power mode
	void leaveLowPowerMode();
	// read diagnostic trouble codes (return number of DTCs read)
	int readDTC(uint16_t codes[], byte maxCodes = 1);
	// clear diagnostic trouble code
	void clearDTC();
	// get battery voltage (works without ECU)
	float getVoltage();
	// get VIN as a string, buffer length should be >= OBD_RECV_BUF_SIZE
	bool getVIN(char* buffer, byte bufsize);
	// determine if the PID is supported
	bool isValidPID(byte pid);
	// specify custom CAN header ID
	void setHeaderID(uint32_t num);
	// toggle CAN sniffing mode, call setHeaderFilter and setHeaderMask before start sniffing
	void sniff(bool enabled = true);
	// set CAN bus header filter
	void setHeaderFilter(uint32_t num);
	// set CAN bus header filter bitmask
	void setHeaderMask(uint32_t bitmask);
	// receive sniffed data
	int receiveData(byte* buf, int len);
	// set current PID mode
	byte dataMode = 1;
	// occurrence of errors
	byte errors = 0;
	// bit map of supported PIDs
	byte pidmap[4 * 8] = {0};
	// link object pointer
	CLink* link = 0;
protected:
	virtual void idleTasks() { delay(5); }
	char* getResponse(byte& pid, char* buffer, byte bufsize);
	uint8_t getPercentageValue(char* data);
	uint16_t getLargeValue(char* data);
	uint8_t getSmallValue(char* data);
	int16_t getTemperatureValue(char* data);
	int normalizeData(byte pid, char* data);
	byte checkErrorMessage(const char* buffer);
	char* getResultValue(char* buf);
	OBD_STATES m_state = OBD_DISCONNECTED;
};

