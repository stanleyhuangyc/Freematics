/*************************************************************************
* Arduino Library for Freematics ONE
* Distributed under BSD license
* Visit http://freematics.com/products/freematics-one for more information
* (C)2012-2018 Stanley Huang <stanley@freematics.com.au>
*************************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "FreematicsONE.h"

//#define XBEE_DEBUG
//#define DEBUG Serial

static const uint8_t targets[][4] = {
	{0x24, 0x4f, 0x42, 0x44},
	{0x24, 0x47, 0x50, 0x53},
	{0x24, 0x47, 0x53, 0x4D}
};

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

byte COBDSPI::readDTC(uint16_t codes[], byte maxCodes)
{
	/*
	Response example:
	0: 43 04 01 08 01 09 
	1: 01 11 01 15 00 00 00
	*/ 
	byte codesRead = 0;
 	for (byte n = 0; n < 6; n++) {
		char buffer[128];
		sprintf_P(buffer, n == 0 ? PSTR("03\r") : PSTR("03%02X\r"), n);
		write(buffer);
		if (receive(buffer, sizeof(buffer)) > 0) {
			if (!strstr_P(buffer, PSTR("NO DATA"))) {
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

void COBDSPI::clearDTC()
{
	char buffer[32];
    setTarget(TARGET_OBD);
	write("04\r");
	receive(buffer, sizeof(buffer));
}

void COBDSPI::sendQuery(byte pid)
{
	char cmd[8];
	sprintf_P(cmd, PSTR("%02X%02X\r"), dataMode, pid);
    setTarget(TARGET_OBD);
	write(cmd);
}

bool COBDSPI::readPID(byte pid, int& result)
{
	// send a single query command
	sendQuery(pid);
	// receive and parse the response
	char buffer[64];
	char* data = 0;
	idleTasks();
	if (receive(buffer, sizeof(buffer)) > 0 && !checkErrorMessage(buffer)) {
		char *p = buffer;
		while ((p = strstr(p, "41 "))) {
			p += 3;
			byte curpid = hex2uint8(p);
			if (curpid == pid) {
				errors = 0;
				p += 2;
				if (*p == ' ') {
					data = p + 1;
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

int COBDSPI::normalizeData(byte pid, char* data)
{
	int result;
	switch (pid) {
	case PID_RPM:
	case PID_EVAP_SYS_VAPOR_PRESSURE:
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
	case PID_MAF_FLOW:
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

float COBDSPI::getVoltage()
{
	char buf[32];
	setTarget(TARGET_OBD);
	if (sendCommand("ATRV\r", buf, sizeof(buf)) > 0) {
		return atof(buf + 4);
	}
	return 0;
}

bool COBDSPI::getVIN(char* buffer, int bufsize)
{
	setTarget(TARGET_OBD);
	for (byte n = 0; n < 5; n++) {
		if (sendCommand("0902\r", buffer, bufsize)) {
			int len = hex2uint16(buffer + sizeof(targets[0]));
			char *p = strstr_P(buffer + 4, PSTR("0: 49 02 01"));
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
		delay(200);
	}
	return false;
}

bool COBDSPI::isValidPID(byte pid)
{
	if (pid >= 0x7f)
		return true;
	pid--;
	byte i = pid >> 3;
	byte b = 0x80 >> (pid & 0x7);
	return pidmap[i] & b;
}

void COBDSPI::reset()
{
	char buf[32];
	sendCommand("ATR\r", buf, sizeof(buf));
	delay(3000);
	errors = 0;
}

void COBDSPI::end()
{
	char buf[32];
	sendCommand("ATPC\r", buf, sizeof(buf));
	SPI.end();
}

byte COBDSPI::checkErrorMessage(const char* buffer)
{
	const char *errmsg[] = {"UNABLE", "ERROR", "TIMEOUT", "NO DATA"};
	for (byte i = 0; i < sizeof(errmsg) / sizeof(errmsg[0]); i++) {
		if (strstr(buffer, errmsg[i])) return i + 1;
	}
	return 0;
}

bool COBDSPI::init(OBD_PROTOCOLS protocol)
{
	const char *initcmd[] = {"ATZ\r", "ATE0\r", "ATH0\r"};
	char buffer[64];
	byte stage;

	m_state = OBD_DISCONNECTED;
	for (byte n = 0; n < 3; n++) {
		stage = 0;
		if (n != 0) reset();
		for (byte i = 0; i < sizeof(initcmd) / sizeof(initcmd[0]); i++) {
			delay(10);
			if (!sendCommand(initcmd[i], buffer, sizeof(buffer), OBD_TIMEOUT_SHORT)) {
				continue;
			}
		}
		stage = 1;
		if (protocol != PROTO_AUTO) {
			sprintf(buffer, "ATSP%u\r", protocol);
			delay(10);
			if (!sendCommand(buffer, buffer, sizeof(buffer), OBD_TIMEOUT_SHORT) || !strstr(buffer, "OK")) {
				continue;
			}
		}
		stage = 2;
		delay(10);
		if (!sendCommand("010D\r", buffer, sizeof(buffer), OBD_TIMEOUT_LONG) || checkErrorMessage(buffer)) {
			continue;
		}
		stage = 3;
		// load pid map
		memset(pidmap, 0xff, sizeof(pidmap));
		for (byte i = 0; i < 4; i++) {
			byte pid = i * 0x20;
			sprintf(buffer, "%02X%02X\r", dataMode, pid);
			delay(10);
			write(buffer);
			delay(10);
			if (!receive(buffer, sizeof(buffer), OBD_TIMEOUT_LONG) || checkErrorMessage(buffer)) break;
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
		break;
	}
	if (stage == 3) {
		m_state = OBD_CONNECTED;
		errors = 0;
		return true;
	} else {
#ifdef DEBUG
		DEBUG.print("Stage:");
		DEBUG.println(stage);
#endif
		reset();
		return false;
	}
}

#ifdef DEBUG
void COBDSPI::debugOutput(const char *s)
{
	DEBUG.print('[');
	DEBUG.print(millis());
	DEBUG.print(']');
	DEBUG.println(s);
}
#endif


byte COBDSPI::begin()
{
	// turn off ADC
#ifdef ARDUINO_ARCH_AVR
	ADCSRA &= ~(1 << ADEN);
#endif

	m_target = TARGET_OBD;
	pinMode(SPI_PIN_READY, INPUT);
	pinMode(SPI_PIN_CS, OUTPUT);
	digitalWrite(SPI_PIN_CS, HIGH);
	SPI.begin();
	SPI.setClockDivider(SPI_CLOCK_DIV64);
	return getVersion();
}

byte COBDSPI::getVersion()
{
	byte version = 0;
	setTarget(TARGET_OBD);
	for (byte n = 0; n < 10; n++) {
		char buffer[32];
		if (sendCommand("ATI\r", buffer, sizeof(buffer), 1000)) {
			char *p = strstr(buffer, "OBDUART");
			if (p) {
				p += 9;
				version = (*p - '0') * 10 + (*(p + 2) - '0');
				if (version) break;
			}
		}
	}
	return version;
}

int COBDSPI::receive(char* buffer, int bufsize, unsigned int timeout)
{
	int n = 0;
	bool eos = false;
	bool matched = false;
	uint32_t t = millis();
	do {
		while (digitalRead(SPI_PIN_READY) == HIGH) {
			delay(1);
			if (millis() - t > timeout) {
				Serial.println("NO SPI DATA");
				break;
			}
		}
		delay(1);
		digitalWrite(SPI_PIN_CS, LOW);
		while (digitalRead(SPI_PIN_READY) == LOW && millis() - t < timeout) {
			char c = SPI.transfer(' ');
			if (eos) continue;
			if (!matched) {
				if (c == '$')  {
					buffer[0] = c;
					n = 1;
					matched = true;	
				}
				continue;
			}
			if (n > 6 && c == '.' && buffer[n - 1] == '.' && buffer[n - 2] == '.') {
				// SEARCHING...
				n = 4;
				timeout += OBD_TIMEOUT_LONG;
			} else if (c == 0x9) {
				eos = true;
			} else if (c != 0 && c != 0xff && n < bufsize - 1) {
				buffer[n++] = c;
			}
		}
		delay(1);
		digitalWrite(SPI_PIN_CS, HIGH);
		delay(1);
	} while (!eos && millis() - t < timeout);
#ifdef DEBUG
	if (!eos && millis() - t >= timeout) {
		// timed out
		debugOutput("RECV TIMEOUT");
	}
#endif
	buffer[n] = 0;
#ifdef DEBUG
	if (m_target == TARGET_OBD) {
		debugOutput(buffer);
	}
#endif
	// wait for READY pin to restore high level so SPI bus is released
	while (digitalRead(SPI_PIN_READY) == LOW);
	return n;
}

void COBDSPI::write(const char* s)
{
#ifdef DEBUG
	debugOutput(s);
#endif
	delay(1);
	digitalWrite(SPI_PIN_CS, LOW);
	delay(5);
	//SPI.beginTransaction(spiSettings);
	if (*s != '$') {
		for (byte i = 0; i < sizeof(targets[0]); i++) {
			SPI.transfer(targets[m_target][i]);
		}
	}
	byte c = 0;
	for (; *s ;s++) {
		c = *s;
		SPI.transfer((byte)c);
	}
	if (c != '\r') SPI.transfer('\r');
	// send terminating byte
	SPI.transfer(0x1B);
	delay(1);
	//SPI.endTransaction();
	digitalWrite(SPI_PIN_CS, HIGH);
	delay(1);
}

byte COBDSPI::readPID(const byte pid[], byte count, int result[])
{
	byte results = 0;
	for (byte n = 0; n < count; n++) {
		if (readPID(pid[n], result[n])) {
			results++;
		}
		sleep(10);
	}
	return results;
}

byte COBDSPI::sendCommand(const char* cmd, char* buf, int bufsize, unsigned int timeout)
{
	uint32_t t = millis();  
	byte n;
	do {
		write(cmd);
		delay(1);
		n = receive(buf, bufsize, timeout);
		if (n == 0 || (buf[1] != 'O' && !memcmp_P(buf + 5, PSTR("NO DATA"), 7))) {
			// data not ready
			sleep(20);
		} else {
	  		break;
		}
	} while (millis() - t < timeout);
	return n;
}

void COBDSPI::sleep(unsigned int ms)
{
	uint32_t t = millis();
	if (ms >= 10) idleTasks();
	while (millis() - t < ms);
}

bool COBDSPI::gpsInit(unsigned long baudrate)
{
	bool success = false;
	char buf[64];
	m_target = TARGET_OBD;
	if (baudrate) {
		if (sendCommand("ATGPSON\r", buf, sizeof(buf))) {
			sprintf_P(buf, PSTR("ATBR2%lu\r"), baudrate);
			sendCommand(buf, buf, sizeof(buf));
			uint32_t t = millis();
			delay(100);
			do {
				if (gpsGetRawData(buf, sizeof(buf)) && strstr(buf, "S$G")) {
					return true;
				}
			} while (millis() - t < GPS_INIT_TIMEOUT);
		}
	} else {
		success = true;
	}
	//turn off GPS power
	sendCommand("ATGPSOFF\r", buf, sizeof(buf));
	return success;
}

bool COBDSPI::gpsGetData(GPS_DATA* gdata)
{
	char buf[128];
	m_target = TARGET_OBD;
	if (sendCommand("ATGPS\r", buf, sizeof(buf), GPS_READ_TIMEOUT) == 0) {
		return false;
	}
	if (memcmp(buf, "$GPS,", 5)) {
		return false;
	}

	byte index = 0;
	bool valid = true;
	char *s = buf + 5;
	for (char* p = s; *p && valid; p++) {
		char c = *p;
		if (c == ',' || c == '>' || c <= 0x0d) {
			int32_t value = atol(s);
			switch (index) {
			case 0:
				if (value == 0)
					valid = false;
				else
					gdata->date = (uint32_t)value;
				break;
			case 1:
				gdata->time = (uint32_t)value;
				break;
			case 2:
				gdata->lat = value;
				break;
			case 3:
				gdata->lng = value;
				break;
			case 4:
				gdata->alt = value / 100;
				break;
			case 5:
				gdata->speed = value;
				break;
			case 6:
				gdata->heading = value;
				break;
			case 7:
				gdata->sat = value;
				break;
			default:
				valid = false;
			}
			index++;
			if (c != ',') break;
			s = p + 1;
		}
	}
	return valid && index > 7;
}

int COBDSPI::gpsGetRawData(char* buf, int bufsize)
{
	m_target = TARGET_OBD;
	int n = sendCommand("ATGRR\r", buf, bufsize, GPS_READ_TIMEOUT);
	if (n > 2) {
		n -= 2;
		buf[n] = 0;
	}
	return n;
}

void COBDSPI::gpsSendCommand(const char* cmd)
{
	setTarget(TARGET_GPS);
	write(cmd);
}

bool COBDSPI::xbBegin(unsigned long baudrate)
{
	char buf[16];
	sprintf_P(buf, PSTR("ATBR1%lu\r"), baudrate);
	setTarget(TARGET_OBD);
	if (sendCommand(buf, buf, sizeof(buf))) {
		xbPurge();
		return true;
	} else {
		return false;
	}
}
	
void COBDSPI::xbWrite(const char* cmd)
{
#ifdef XBEE_DEBUG
	Serial.print("[SEND]");
	Serial.println(cmd);
	Serial.println("[/SEND]");
#endif
	setTarget(TARGET_BEE);
	write(cmd);
}

int COBDSPI::xbRead(char* buffer, int bufsize)
{
	setTarget(TARGET_OBD);
	write("ATGRD\r");
	sleep(10);
	int bytes = receive(buffer, bufsize, 500);
	if (bytes < 4 || memcmp(buffer, targets[TARGET_BEE], 4)) {
		bytes = -1;
	} else if (bytes >= 11 && !memcmp_P(buffer + 4, PSTR("NO DATA"), 7)) {
		buffer[0] = 0;
		bytes = 0;
	}
	return bytes;
}

int COBDSPI::xbReceive(char* buffer, int bufsize, unsigned int timeout, const char** expected, byte expectedCount)
{
	int bytesRecv = 0;
	uint32_t t = millis();
	buffer[0] = 0;
	do {
		while (bytesRecv >= bufsize / 2) {
			bytesRecv -= dumpLine(buffer, bytesRecv);
		}
		int n = xbRead(buffer + bytesRecv, bufsize - bytesRecv);
		if (n > 0) {
			buffer[bytesRecv + n] = 0;
			// skip 4-byte header
			char *p = buffer + bytesRecv + 4;
			n -= 4;
			if (bytesRecv == 0 && *p == '\n') {
				// strip first \n
				p++;
				n--;
			}
			memmove(buffer + bytesRecv, p, n);
			bytesRecv += n;
			buffer[bytesRecv] = 0;
#ifdef XBEE_DEBUG
			if (n > 0) {
				Serial.print("[RECV]");
				Serial.print(buffer + bytesRecv - n);
				Serial.println("[/RECV]");
			}
#endif
			if (expectedCount == 0 && bytesRecv > 0) {
				return bytesRecv;
			}
			for (byte i = 0; i < expectedCount; i++) {
				// match expected string(s)
				if (expected[i] && strstr(buffer, expected[i])) return i + 1;
			}
			sleep(50);
		} if (n < 0) {
			break;
		} else {
			if (millis() - t + 200 < timeout)
				sleep(200);
			else
				break;
		}
	} while (millis() - t < timeout);
	return 0;
}

void COBDSPI::xbPurge()
{
	char buf[16];
	setTarget(TARGET_OBD);
	sendCommand("ATCLRGSM\r", buf, sizeof(buf));
}

void COBDSPI::xbTogglePower()
{
	setTarget(TARGET_OBD);
	char buffer[64];
	sendCommand("ATGSMPWR\r", buffer, sizeof(buffer));
}
