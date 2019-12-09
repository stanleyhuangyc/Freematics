/*************************************************************************
* Arduino Library for Freematics ONE+
* Distributed under BSD license
* Visit https://freematics.com for more information
* (C)2012-2019 Developed by Stanley Huang <stanley@freematics.com.au>
*************************************************************************/

#include <Arduino.h>
#include "FreematicsBase.h"
#include "FreematicsOBD.h"

int dumpLine(char* buffer, int len)
{
	int bytesToDump = len >> 1;
	for (int i = 0; i < len; i++) {
		// find out first line end and discard the first line
		if (buffer[i] == '\r' || buffer[i] == '\n') {
			// go through all following \r or \n if any
			while (++i < len && (buffer[i] == '\r' || buffer[i] == '\n'));
			bytesToDump = i;
			break;
		}
	}
	memmove(buffer, buffer + bytesToDump, len - bytesToDump);
	return bytesToDump;
}

uint16_t hex2uint16(const char *p)
{
	char c = *p;
	uint16_t i = 0;
	for (uint8_t n = 0; c && n < 4; c = *(++p)) {
		if (c >= 'A' && c <= 'F') {
			c -= 7;
		} else if (c>='a' && c<='f') {
			c -= 39;
        } else if (c == ' ' && n == 2) {
            continue;
        } else if (c < '0' || c > '9') {
			break;
        }
		i = (i << 4) | (c & 0xF);
		n++;
	}
	return i;
}

byte hex2uint8(const char *p)
{
	byte c1 = *p;
	byte c2 = *(p + 1);
	if (c1 >= 'A' && c1 <= 'F')
		c1 -= 7;
	else if (c1 >= 'a' && c1 <= 'f')
		c1 -= 39;
	else if (c1 < '0' || c1 > '9')
		return 0;

	if (c2 == 0)
		return (c1 & 0xf);
	else if (c2 >= 'A' && c2 <= 'F')
		c2 -= 7;
	else if (c2 >= 'a' && c2 <= 'f')
		c2 -= 39;
	else if (c2 < '0' || c2 > '9')
		return 0;

	return c1 << 4 | (c2 & 0xf);
}

/*************************************************************************
* OBD-II UART Bridge
*************************************************************************/

bool COBD::readPID(byte pid, int& result)
{
	char buffer[64];
	char* data = 0;
	sprintf(buffer, "%02X%02X\r", dataMode, pid);
	link->send(buffer);
	idleTasks();
	int ret = link->receive(buffer, sizeof(buffer), OBD_TIMEOUT_SHORT);
	if (ret > 0 && !checkErrorMessage(buffer)) {
		char *p = buffer;
		while ((p = strstr(p, "41 "))) {
			p += 3;
			byte curpid = hex2uint8(p);
			if (curpid == pid) {
				errors = 0;
				while (*p && *p != ' ') p++;
				while (*p == ' ') p++;
				if (*p) {
					data = p;
					break;
				}
			}
		}
	}

	if (!data) {
		errors++;
		return false;
	}
	result = normalizeData(pid, data);
	return true;
}

byte COBD::readPID(const byte pid[], byte count, int result[])
{
	byte results = 0;
	for (byte n = 0; n < count; n++) {
		if (readPID(pid[n], result[n])) {
			results++;
		}
	}
	return results;
}

int COBD::readDTC(uint16_t codes[], byte maxCodes)
{
	/*
	Response example:
	0: 43 04 01 08 01 09
	1: 01 11 01 15 00 00 00
	*/
	int codesRead = 0;
 	for (int n = 0; n < 6; n++) {
		char buffer[128];
		sprintf(buffer, n == 0 ? "03\r" : "03%02X\r", n);
		link->send(buffer);
		if (link->receive(buffer, sizeof(buffer), OBD_TIMEOUT_LONG) > 0) {
			if (!strstr(buffer, "NO DATA")) {
				char *p = strstr(buffer, "43");
				if (p) {
					while (codesRead < maxCodes && *p) {
						p += 6;
						if (*p == '\r') {
							p = strchr(p, ':');
							if (!p) break;
							p += 2;
						}
						uint16_t code = hex2uint16(p);
						if (code == 0) break;
						codes[codesRead++] = code;
					}
				}
				break;
			}
		}
	}
	return codesRead;
}

void COBD::clearDTC()
{
	char buffer[32];
	link->send("04\r");
	link->receive(buffer, sizeof(buffer), OBD_TIMEOUT_LONG);
}

int COBD::normalizeData(byte pid, char* data)
{
	int result;
	switch (pid) {
	case PID_RPM:
	case PID_EVAP_SYS_VAPOR_PRESSURE: // kPa
		result = getLargeValue(data) >> 2;
		break;
	case PID_FUEL_PRESSURE: // kPa
		result = getSmallValue(data) * 3;
		break;
	case PID_COOLANT_TEMP:
	case PID_INTAKE_TEMP:
	case PID_AMBIENT_TEMP:
	case PID_ENGINE_OIL_TEMP:
		result = getTemperatureValue(data);
		break;
	case PID_THROTTLE:
	case PID_COMMANDED_EGR:
	case PID_COMMANDED_EVAPORATIVE_PURGE:
	case PID_FUEL_LEVEL:
	case PID_RELATIVE_THROTTLE_POS:
	case PID_ABSOLUTE_THROTTLE_POS_B:
	case PID_ABSOLUTE_THROTTLE_POS_C:
	case PID_ACC_PEDAL_POS_D:
	case PID_ACC_PEDAL_POS_E:
	case PID_ACC_PEDAL_POS_F:
	case PID_COMMANDED_THROTTLE_ACTUATOR:
	case PID_ENGINE_LOAD:
	case PID_ABSOLUTE_ENGINE_LOAD:
	case PID_ETHANOL_FUEL:
	case PID_HYBRID_BATTERY_PERCENTAGE:
		result = getPercentageValue(data);
		break;
	case PID_MAF_FLOW: // grams/sec
		result = getLargeValue(data) / 100;
		break;
	case PID_TIMING_ADVANCE:
		result = (int)(getSmallValue(data) / 2) - 64;
		break;
	case PID_DISTANCE: // km
	case PID_DISTANCE_WITH_MIL: // km
	case PID_TIME_WITH_MIL: // minute
	case PID_TIME_SINCE_CODES_CLEARED: // minute
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
	case PID_ENGINE_TORQUE_DEMANDED: // %
	case PID_ENGINE_TORQUE_PERCENTAGE: // %
		result = (int)getSmallValue(data) - 125;
		break;
	case PID_SHORT_TERM_FUEL_TRIM_1:
	case PID_LONG_TERM_FUEL_TRIM_1:
	case PID_SHORT_TERM_FUEL_TRIM_2:
	case PID_LONG_TERM_FUEL_TRIM_2:
	case PID_EGR_ERROR:
		result = ((int)getSmallValue(data) - 128) * 100 / 128;
		break;
	case PID_FUEL_INJECTION_TIMING:
		result = ((int32_t)getLargeValue(data) - 26880) / 128;
		break;
	case PID_CATALYST_TEMP_B1S1:
	case PID_CATALYST_TEMP_B2S1:
	case PID_CATALYST_TEMP_B1S2:
	case PID_CATALYST_TEMP_B2S2:
		result = getLargeValue(data) / 10 - 40;
		break;
	case PID_AIR_FUEL_EQUIV_RATIO: // 0~200
		result = (long)getLargeValue(data) * 200 / 65536;
		break;
	default:
		result = getSmallValue(data);
	}
	return result;
}

char* COBD::getResponse(byte& pid, char* buffer, byte bufsize)
{
	while (link->receive(buffer, bufsize, OBD_TIMEOUT_SHORT) > 0) {
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
		    }
		}
	}
	return 0;
}

void COBD::enterLowPowerMode()
{
  	char buf[32];
	link->sendCommand("ATLP\r", buf, sizeof(buf), 1000);
}

void COBD::leaveLowPowerMode()
{
	// send any command to wake up
	char buf[32];
	for (byte n = 0; n < 30 && !link->sendCommand("ATI\r", buf, sizeof(buf), 1000); n++);
}

char* COBD::getResultValue(char* buf)
{
	char* p = buf;
	for (;;) {
		if (isdigit(*p) || *p == '-') {
			return p;
		}
		p = strchr(p, '\r');
		if (!p) break;
		if (*(++p) == '\n') p++;
	}
	return 0;
}

float COBD::getVoltage()
{
    char buf[32];
	if (link->sendCommand("ATRV\r", buf, sizeof(buf), 500) > 0) {
		char* p = getResultValue(buf);
		if (p) return (float)atof(p);
    }
    return 0;
}

bool COBD::getVIN(char* buffer, byte bufsize)
{
	for (byte n = 0; n < 2; n++) {
		if (link->sendCommand("0902\r", buffer, bufsize, OBD_TIMEOUT_LONG)) {
			int len = hex2uint16(buffer);
			char *p = strstr(buffer + 4, "0: 49 02 01");
			if (p) {
				char *q = buffer;
				p += 11; // skip the header
				do {
					while (*(++p) == ' ');
					for (;;) {
						*(q++) = hex2uint8(p);
						while (*p && *p != ' ') p++;
						while (*p == ' ') p++;
						if (!*p || *p == '\r') break;
					}
					p = strchr(p, ':');
				} while(p);
				*q = 0;
				if (q - buffer == len - 3) {
					return true;
				}
			}
		}
		delay(100);
	}
    return false;
}

bool COBD::isValidPID(byte pid)
{
	if (pid >= 0x7f)
		return true;
	pid--;
	byte i = pid >> 3;
	byte b = 0x80 >> (pid & 0x7);
	return (pidmap[i] & b) != 0;
}

bool COBD::init(OBD_PROTOCOLS protocol)
{
	const char *initcmd[] = {"ATE0\r", "ATH0\r"};
	char buffer[64];
	byte stage;

	if (!link) {
		return false;
	}

	m_state = OBD_DISCONNECTED;
	stage = 0;
	for (byte n = 0; n < 10; n++) {
		if (link->sendCommand("ATZ\r", buffer, sizeof(buffer), OBD_TIMEOUT_SHORT)) {
			stage = 1;
			break;
		}
	}
	if (stage == 0) return false;
	for (byte i = 0; i < sizeof(initcmd) / sizeof(initcmd[0]); i++) {
		link->sendCommand(initcmd[i], buffer, sizeof(buffer), OBD_TIMEOUT_SHORT);
	}
	if (protocol != PROTO_AUTO) {
		sprintf(buffer, "ATSP %X\r", protocol);
		if (!link->sendCommand(buffer, buffer, sizeof(buffer), OBD_TIMEOUT_SHORT) || !strstr(buffer, "OK")) {
			return false;
		}
	}
	stage = 2;
	if (protocol == PROTO_J1939) {
		m_state = OBD_CONNECTED;
		errors = 0;
		return true;
	}

	for (byte n = 0; n < 2; n++) {
		int value;
		if (readPID(PID_SPEED, value)) {
			stage = 3;
			break;
		}
	}
	if (stage != 3) return false;

	// load pid map
	memset(pidmap, 0xff, sizeof(pidmap));
	for (byte i = 0; i < 4; i++) {
		byte pid = i * 0x20;
		sprintf(buffer, "%02X%02X\r", dataMode, pid);
		link->send(buffer);
		if (!link->receive(buffer, sizeof(buffer), OBD_TIMEOUT_LONG) || checkErrorMessage(buffer)) {
			Serial.print("NO PID MAP ");
			break;
		}
		for (char *p = buffer; (p = strstr(p, "41 ")); ) {
			p += 3;
			if (hex2uint8(p) == pid) {
				p += 2;
				for (byte n = 0; n < 4 && *(p + n * 3) == ' '; n++) {
					pidmap[i * 4 + n] = hex2uint8(p + n * 3 + 1);
				}
			}
		}
	}
	m_state = OBD_CONNECTED;
	errors = 0;
	return true;
}

void COBD::reset()
{
	char buf[32];
	link->sendCommand("ATR\r", buf, sizeof(buf), OBD_TIMEOUT_SHORT);
}

void COBD::uninit()
{
	char buf[32];
	link->sendCommand("ATPC\r", buf, sizeof(buf), OBD_TIMEOUT_SHORT);
}

byte COBD::checkErrorMessage(const char* buffer)
{
	const char *errmsg[] = {"UNABLE", "ERROR", "TIMEOUT", "NO DATA"};
	for (byte i = 0; i < sizeof(errmsg) / sizeof(errmsg[0]); i++) {
		if (strstr(buffer, errmsg[i])) return i + 1;
	}
	return 0;
}

uint8_t COBD::getPercentageValue(char* data)
{
  return (uint16_t)hex2uint8(data) * 100 / 255;
}

uint16_t COBD::getLargeValue(char* data)
{
  return hex2uint16(data);
}

uint8_t COBD::getSmallValue(char* data)
{
  return hex2uint8(data);
}

int16_t COBD::getTemperatureValue(char* data)
{
  return (int)hex2uint8(data) - 40;
}

void COBD::setHeaderID(uint32_t num)
{
	char buf[32];
	sprintf(buf, "ATSH %X\r", num & 0xffffff);
	link->sendCommand(buf, buf, sizeof(buf), 1000);
	sprintf(buf, "ATCP %X\r", num & 0x1f);
	link->sendCommand(buf, buf, sizeof(buf), 1000);
}

void COBD::sniff(bool enabled)
{
	char buf[32];
	link->sendCommand(enabled ? "ATM1\r" : "ATM0\r", buf, sizeof(buf), 1000);
}

void COBD::setHeaderFilter(uint32_t num)
{
	char buf[32];
	sprintf(buf, "ATCF %X\r", num);
	link->sendCommand(buf, buf, sizeof(buf), 1000);
}
	
void COBD::setHeaderMask(uint32_t bitmask)
{
	char buf[32];
	sprintf(buf, "ATCM %X\r", bitmask);
	link->sendCommand(buf, buf, sizeof(buf), 1000);
}

int COBD::receiveData(byte* buf, int len)
{
	int n = 0;
	for (n = 0; n < len; ) {
		int c = link->read();
		if (c == -1 || c == '\r') break;
		buf[n++] = c;
	}
	len = n;
	int m = 0;
	for (n = 0; n < len; m++) {
		buf[m] = hex2uint8((const char*)buf + n);
		n += 2;
		if (buf[n++] != ' ') break;
	}
	return m;
}