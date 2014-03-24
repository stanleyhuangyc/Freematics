/*************************************************************************
* Arduino Library for OBD-II UART/I2C Adapter
* Distributed under GPL v2.0
* Visit http://freematics.com for more information
* (C)2012-2014 Stanley Huang <stanleyhuangyc@gmail.com>
*************************************************************************/

#include <Arduino.h>
#include <avr/pgmspace.h>
#include "OBD.h"

//#define DEBUG Serial
//#define REDIRECT Serial

#define MAX_CMD_LEN 6

const char PROGMEM s_initcmd[][MAX_CMD_LEN] = {"ATZ\r","ATE0\r","ATL1\r","0902\r"};
const char PROGMEM s_cmd_fmt[] = "%02X%02X 1\r";
const char PROGMEM s_cmd_sleep[] = "atlp\r";
#define STR_SEARCHING "SEARCHING..."

unsigned int hex2uint16(const char *p)
{
	char c = *p;
	unsigned int i = 0;
	for (char n = 0; c && n < 4; c = *(++p)) {
		if (c >= 'A' && c <= 'F') {
			c -= 7;
		} else if (c>='a' && c<='f') {
			c -= 39;
        } else if (c == ' ') {
            continue;
        } else if (c < '0' || c > '9') {
			break;
        }
		i = (i << 4) | (c & 0xF);
		n++;
	}
	return i;
}

unsigned char hex2uint8(const char *p)
{
	unsigned char c1 = *p;
	unsigned char c2 = *(p + 1);
	if (c1 >= 'A' && c1 <= 'F')
		c1 -= 7;
	else if (c1 >='a' && c1 <= 'f')
	    c1 -= 39;
	else if (c1 < '0' || c1 > '9')
		return 0;

	if (c2 >= 'A' && c2 <= 'F')
		c2 -= 7;
	else if (c2 >= 'a' && c2 <= 'f')
	    c2 -= 39;
	else if (c2 < '0' || c2 > '9')
		return 0;

	return c1 << 4 | (c2 & 0xf);
}

/*************************************************************************
* OBD-II UART Adapter
*************************************************************************/
#include <Wire.h>

void COBD::sendQuery(unsigned char pid)
{
	char cmd[8];
	sprintf_P(cmd, s_cmd_fmt, dataMode, pid);
#ifdef DEBUG
	debugOutput(cmd);
#endif
	write(cmd);
}

bool COBD::read(byte pid, int& result)
{
	// send a query command
	sendQuery(pid);
	// wait for reponse
	bool hasData;
	unsigned long tick = millis();
	do {
		dataIdleLoop();
	} while (!(hasData = available()) && millis() - tick < OBD_TIMEOUT_SHORT);
	if (!hasData) {
		errors++;
		return false;
	}
	// receive and parse the response
	return getResult(pid, result);
}

bool COBD::available()
{
	return OBDUART.available();
}

char COBD::read()
{
	char c = OBDUART.read();
#ifdef REDIRECT
    REDIRECT.write(c);
#endif
	return c;
}

void COBD::write(char* s)
{
	OBDUART.write(s);
}

void COBD::write(char c)
{
	OBDUART.write(c);
}

int COBD::normalizeData(byte pid, char* data)
{
	int result;
	switch (pid) {
	case PID_RPM:
		result = getLargeValue(data) >> 2;
		break;
	case PID_FUEL_PRESSURE:
		result = getSmallValue(data) * 3;
		break;
	case PID_COOLANT_TEMP:
	case PID_INTAKE_TEMP:
	case PID_AMBIENT_TEMP:
	case PID_ENGINE_OIL_TEMP:
		result = getTemperatureValue(data);
		break;
	case PID_THROTTLE:
	case PID_ENGINE_LOAD:
	case PID_FUEL_LEVEL:
	case PID_ABSOLUTE_ENGINE_LOAD:
	case PID_ETHANOL_PERCENTAGE:
	case PID_HYBRID_BATTERY_PERCENTAGE:
		result = getPercentageValue(data);
		break;
	case PID_MAF_FLOW:
		result = getLargeValue(data) / 100;
		break;
	case PID_TIMING_ADVANCE:
		result = (int)(getSmallValue(data) / 2) - 64;
		break;
	case PID_DISTANCE: // km
	case PID_RUNTIME: // second
	case PID_FUEL_RAIL_PRESSURE: // kPa
	case PID_ENGINE_REF_TORQUE: // Nm
		result = getLargeValue(data);
		break;
	case PID_CONTROL_MODULE_VOLTAGE: // V
		result = getLargeValue(data) / 1000;
		break;
	case PID_ENGINE_FUEL_RATE: // L/h
		result = getLargeValue(data) / 20;
		break;
	case PID_ENGINE_TORQUE_PERCENTAGE: // %
		result = (int)getSmallValue(data) - 125;
		break;
	default:
		result = getSmallValue(data);
	}
	return result;
}

char* COBD::getResponse(byte& pid, char* buffer)
{
	receive(buffer);
	char *p = buffer;
	while ((p = strstr(p, "41 "))) {
		p += 3;
		byte curpid = hex2uint8(p);
		if (pid == 0) pid = curpid;
		if (curpid == pid) {
			errors = 0;
			p += 2;
			if (*p == ' ')
				return p + 1;
		} else {
			receive(buffer);
			p = buffer;
		}
	}
	return 0;
}

bool COBD::getResult(byte& pid, int& result)
{
	char buffer[OBD_RECV_BUF_SIZE];
	char* data = getResponse(pid, buffer);
	if (!data) {
		recover();
		return false;
	}
	result = normalizeData(pid, data);
	return true;
}

void COBD::sleep(int seconds)
{
	char cmd[MAX_CMD_LEN];
	strcpy_P(cmd, s_cmd_sleep);
	write(cmd);
	if (seconds) {
		delay((unsigned long)seconds << 10);
		write('\r');
	}
}

bool COBD::isValidPID(byte pid)
{
	if (pid >= 0x7f)
		return false;
	pid--;
	byte i = pid >> 3;
	byte b = 0x80 >> (pid & 0x7);
	return pidmap[i] & b;
}

void COBD::begin()
{
	OBDUART.begin(OBD_SERIAL_BAUDRATE);
}

byte COBD::receive(char* buffer, int timeout)
{
	unsigned long startTime = millis();
	unsigned char n = 0;
	bool prompted = false;

	buffer[0] = 0;
	for (;;) {
	    if (available()) {
	        char c = read();
	        if (n > 2 && c == '>') {
	            // prompt char received
	            prompted = true;
	        } else if (n < OBD_RECV_BUF_SIZE - 1) {
	            buffer[n++] = c;
	            buffer[n] = 0;
	            if (strstr(buffer, STR_SEARCHING)) {
	                strcpy(buffer, buffer + sizeof(STR_SEARCHING));
	                n -= sizeof(STR_SEARCHING);
	                timeout = OBD_TIMEOUT_LONG;
	            }
	        }
	    } else if (prompted) {
	        break;
	    } else {
	        if (millis() - startTime > timeout) {
	            // timeout
	            return 0;
	        }
	        dataIdleLoop();
	    }
	}
	return n;
}

void COBD::recover()
{
	write('\r');
	delay(50);
	while (available()) read();
}

bool COBD::init()
{
	char buffer[OBD_RECV_BUF_SIZE];

	m_state = OBD_CONNECTING;
	recover();

	for (unsigned char i = 0; i < sizeof(s_initcmd) / sizeof(s_initcmd[0]); i++) {
	    char cmd[MAX_CMD_LEN];
	    strcpy_P(cmd, s_initcmd[i]);
#ifdef DEBUG
	    debugOutput(cmd);
#endif
	    write(cmd);
		if (receive(buffer) == 0) {
	        return false;
		}
	}
	while (available()) read();

	// load pid map
	memset(pidmap, 0, sizeof(pidmap));
	for (byte i = 0; i < 4; i++) {
	    byte pid = i * 0x20;
	    sendQuery(pid);
	    char* data = getResponse(pid, buffer);
	    if (!data) break;
	    data--;
	    for (byte n = 0; n < 4; n++) {
	        if (data[n * 3] != ' ')
	            break;
	        pidmap[i * 4 + n] = hex2uint8(data + n * 3 + 1);
	    }
	}
	while (available()) read();

	m_state = OBD_CONNECTED;
	errors = 0;
	return true;
}

#ifdef DEBUG
void COBD::debugOutput(const char *s)
{
	DEBUG.print('[');
	DEBUG.print(millis());
	DEBUG.print(']');
	DEBUG.print(s);
}
#endif
