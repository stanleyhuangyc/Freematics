/*************************************************************************
* Arduino Library for Freematics ONE
* Distributed under BSD license
* Visit http://freematics.com/products/freematics-one for more information
* (C)2012-2016 Stanley Huang <support@freematics.com.au
*************************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/common.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include "FreematicsONE.h"

//#define DEBUG Serial

SIGNAL(WDT_vect) {
  wdt_disable();
  wdt_reset();
  WDTCSR &= ~_BV(WDIE);
}

uint16_t hex2uint16(const char *p)
{
	char c = *p;
	uint16_t i = 0;
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

void COBDSPI::sendQuery(byte pid)
{
	char cmd[8];
	sprintf(cmd, "%02X%02X\r", dataMode, pid);
#ifdef DEBUG
	debugOutput(cmd);
#endif
       setTarget(TARGET_OBD);
	write(cmd);
}

bool COBDSPI::readPID(byte pid, int& result)
{
	// send a single query command
	sendQuery(pid);
	// receive and parse the response
	return getResult(pid, result);
}

void COBDSPI::clearDTC()
{
	char buffer[32];
       setTarget(TARGET_OBD);
	write("04\r");
	receive(buffer, sizeof(buffer));
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

char* COBDSPI::getResponse(byte& pid, char* buffer, byte bufsize)
{
	if (receive(buffer, bufsize) > 0) {
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

bool COBDSPI::getResult(byte& pid, int& result)
{
	char buffer[64];
	char* data = getResponse(pid, buffer, sizeof(buffer));
	if (!data) {
		errors++;
		return false;
	}
	result = normalizeData(pid, data);
	return true;
}

bool COBDSPI::setProtocol(OBD_PROTOCOLS h)
{
	char buf[32];
       setTarget(TARGET_OBD);
	if (h == PROTO_AUTO) {
		write("ATSP00\r");
	} else {
		sprintf(buf, "ATSP%d\r", h);
		write(buf);
	}
	if (receive(buf, sizeof(buf), OBD_TIMEOUT_LONG) > 0 && strstr(buf, "OK"))
		return true;
	else
		return false;
}

void COBDSPI::lowPowerMode()
{
	char buf[32];
	setTarget(TARGET_OBD);
	sendCommand("ATLP\r", buf, sizeof(buf));
}

float COBDSPI::getVoltage()
{
	char buf[32];
	setTarget(TARGET_OBD);
	if (sendCommand("ATRV\r", buf, sizeof(buf)) > 0) {
		return atof(buf);
	}
	return 0;
}

bool COBDSPI::getVIN(char* buffer, byte bufsize)
{
	setTarget(TARGET_OBD);
	if (sendCommand("0902\r", buffer, bufsize)) {
	    char *p = strstr(buffer, "49 02");
	    if (p) {
	        char *q = buffer;
	        p += 10;
	        do {
	            for (++p; *p == ' '; p += 3) {
	                if (*q = hex2uint8(p + 1)) q++;
	            }
	            p = strchr(p, ':');
	        } while(p);
	        *q = 0;
	        return true;
	    }
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

bool COBDSPI::init(OBD_PROTOCOLS protocol)
{
	const char *initcmd[] = {"ATZ\r","ATE0\r","ATL1\r","0100\r"};
	char buffer[64];

	if (!getVersion()) return false;
	for (unsigned char i = 0; i < sizeof(initcmd) / sizeof(initcmd[0]); i++) {
#ifdef DEBUG
		debugOutput(initcmd[i]);
#endif
		write(initcmd[i]);
		if (receive(buffer, sizeof(buffer), OBD_TIMEOUT_LONG) == 0) {
			m_state = OBD_DISCONNECTED;
			return false;
		}
		delay(50);
	}

	if (protocol != PROTO_AUTO) {
		setProtocol(protocol);
	}

	// load pid map
	memset(pidmap, 0, sizeof(pidmap));
	bool success = false;
	for (byte i = 0; i < 4; i++) {
		byte pid = i * 0x20;
		sendQuery(pid);
		char* data = getResponse(pid, buffer, sizeof(buffer));
		if (!data) break;
		data--;
		for (byte n = 0; n < 4; n++) {
			if (data[n * 3] != ' ')
				break;
			pidmap[i * 4 + n] = hex2uint8(data + n * 3 + 1);
		}
		success = true;
		delay(50);
	}

	m_state = OBD_CONNECTED;
	errors = 0;
	return success;
}


#ifdef DEBUG
void COBDSPI::debugOutput(const char *s)
{
	DEBUG.print('[');
	DEBUG.print(millis());
	DEBUG.print(']');
	DEBUG.print(s);
}
#endif


static const char PROGMEM targets[][4] = {
	{'$','O','B','D'},
	{'$','G','P','S'},
	{'$','G','S','M'}
};

byte COBDSPI::begin()
{
	m_target = TARGET_OBD;
	pinMode(SPI_PIN_READY, INPUT);
	pinMode(SPI_PIN_CS, OUTPUT);
	digitalWrite(SPI_PIN_CS, HIGH);
	delay(50);
	SPI.begin();
	SPI.setClockDivider(2);
	delay(50);
	if (!getVersion()) {
		m_state = OBD_FAILED;
	} else {
		m_state = OBD_DISCONNECTED;
	}
	return version;
}

void COBDSPI::end()
{
	SPI.end();
}

byte COBDSPI::getVersion()
{
	version = 0;
    setTarget(TARGET_OBD);
	for (byte n = 0; n < 3; n++) {
		write("ATI\r");
		char buffer[64];
		if (receive(buffer, sizeof(buffer), 100)) {
			char *p = strstr(buffer, "OBDUART");
			if (p) {
				p += 9;
				version = (*p - '0') * 10 + (*(p + 2) - '0');
				if (version) break;
			}
		}
		delay(200);
	}
	return version;
}

int COBDSPI::receive(char* buffer, int bufsize, int timeout)
{
	int n = 0;
	bool eof = false;
	uint32_t t = millis();
	do {
		while (digitalRead(SPI_PIN_READY) == HIGH) {
			dataIdleLoop();
			if (millis() - t > timeout) {
				//Serial.println("TIMEOUT!");
				return 0;
			}
		}
		digitalWrite(SPI_PIN_CS, LOW);
		while (!eof && digitalRead(SPI_PIN_READY) == LOW) {
			if (n == bufsize - 1) {
				int bytesToDiscard = bufsize >> 1;
				n -= bytesToDiscard;
				memmove(buffer, buffer + bytesToDiscard, n); 
			}
			buffer[n] = SPI.transfer(' ');
			eof = n >= 2 && buffer[n] == 0x9 && buffer[n - 1] =='>';
			n++;
		}
		digitalWrite(SPI_PIN_CS, HIGH);
	} while (!eof &&  millis() - t < timeout);
	if (m_target != TARGET_RAW) {
		// eliminate ending char
		if (eof) n--;
	}
	buffer[n] = 0;
	if (m_target == TARGET_OBD && !memcmp(buffer, "$OBDTIMEOUT", 11)) {
		// ECU not responding
		return 0;
	}
	return n;
}

void COBDSPI::write(const char* s)
{
	digitalWrite(SPI_PIN_CS, LOW);
	delay(1);
	if (*s != '$') {
		for (byte i = 0; i < sizeof(targets[0]); i++) {
			SPI.transfer(pgm_read_byte(&targets[m_target][i]));
			delayMicroseconds(5);
		}
	}
	for (; *s ;s++) {
		SPI.transfer((byte)*s);
		delayMicroseconds(5);
	}
	// send terminating byte (ESC)
	if (version != 10) {
		SPI.transfer(0x1B);
		digitalWrite(SPI_PIN_CS, HIGH);
	} else {
		digitalWrite(SPI_PIN_CS, HIGH);
	}
}

void COBDSPI::write(byte* data, int len)
{
	digitalWrite(SPI_PIN_CS, LOW);
	delay(1);
	for (unsigned int i = 0; i < len; i++) {
		SPI.transfer(data[i]);
		delayMicroseconds(5);
	}
	digitalWrite(SPI_PIN_CS, HIGH);
}

byte COBDSPI::readPID(const byte pid[], byte count, int result[])
{
	if (version > 10) {
		// send a multiple query command
		char buffer[128];
		char *p = buffer;
		byte results = 0;
		for (byte n = 0; n < count; n++) {
			p += sprintf(p, "$OBD%02X%02X\r", dataMode, pid[n]);
			if (version > 10) {
				*(p++) = 0x1b;
			}
		}
		*(p - 1) = 0;
		write(buffer);
		// receive and parse the response
		for (byte n = 0; n < count; n++) {
			byte curpid = pid[n];
			if (getResult(curpid, result[n]))
				results++;
		}
		return results;
	}
	else {
		byte results = 0;
		for (byte n = 0; n < count; n++) {
			if (readPID(pid[n], result[n])) {
				results++;
			}
		}
		return results;
	}
}

byte COBDSPI::sendCommand(const char* cmd, char* buf, byte bufsize, int timeout)
{
	uint32_t t = millis();  
	byte n;
	do {
		write(cmd);
		n = receive(buf, bufsize, timeout);
		if (n == 0 || (buf[1] != 'O' && !memcmp(buf + 5, "NO DATA", 7))) {
			// data not ready
			dataIdleLoop();
		} else {
	  		break;
		}
	} while (millis() - t < timeout);
	return n;
}

bool COBDSPI::initGPS(unsigned long baudrate)
{
	bool success = false;
	char buf[128];
	m_target = TARGET_OBD;
	if (baudrate) {
		if (sendCommand("ATGPSON\r", buf, sizeof(buf))) {
			sprintf(buf, "ATBR2%lu\r", baudrate);
			if (sendCommand(buf, buf, sizeof(buf))) {
				delay(200);
				if (getGPSRawData(buf, sizeof(buf)) && strstr(buf, "S$G")) {
					success = true;
				}
			}
		}
	} else {
		if (sendCommand("ATGPSOFF\r", buf, sizeof(buf))) {
			success = true;
		}  
	}
	return success;
}

bool COBDSPI::getGPSData(GPS_DATA* gdata)
{
	char buf[128];
	m_target = TARGET_OBD;
	if (sendCommand("ATGPS\r", buf, sizeof(buf), OBD_TIMEOUT_GPS) == 0 && memcmp(buf, "$GPS,", 5)) {
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
	return index > 7;
}

byte COBDSPI::getGPSRawData(char* buf, byte bufsize)
{
	m_target = TARGET_OBD;
	byte n = sendCommand("ATGRR\r", buf, bufsize, OBD_TIMEOUT_GPS);
	if (n > 2) {
		n -= 2;
		buf[n] = 0;
	}
	return n;
}

void COBDSPI::sendGPSCommand(const char* cmd)
{
	setTarget(TARGET_GPS);
	write(cmd);
}

void COBDSPI::sleep(uint8_t seconds) {
	uint8_t wdt_period;
	switch (seconds) {
	case 1:
		wdt_period = WDTO_1S;
		break;
	case 2:
		wdt_period = WDTO_2S;
		break;
	case 3:
	case 4:
		wdt_period = WDTO_4S;
		break;
	default:
		wdt_period = WDTO_8S;
	}
	wdt_enable(wdt_period);
	wdt_reset();
	WDTCSR |= _BV(WDIE);
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_mode();
	wdt_disable();
	WDTCSR &= ~_BV(WDIE);
}

bool COBDSPI::xbBegin(unsigned long baudrate)
{
	char buf[16];
	sprintf(buf, "ATBR1%lu\r", baudrate);
	setTarget(TARGET_OBD);
	if (sendCommand(buf, buf, sizeof(buf))) {
		//xbPurge();
		return true;
	} else {
		return false;
	}
}
	
void COBDSPI::xbWrite(const char* cmd)
{
	setTarget(TARGET_BEE);
	write(cmd);
}

byte COBDSPI::xbRead(char* buffer, byte bufsize, int timeout)
{
	setTarget(TARGET_OBD);
	write("ATGRD\r");
	dataIdleLoop();
	return receive(buffer, bufsize, timeout);
}

byte COBDSPI::xbReceive(char* buffer, int bufsize, int timeout, const char* expected1, const char* expected2)
{
	int bytesRecv = 0;
	uint32_t t = millis();
	setTarget(TARGET_OBD);
	do {
		if (bytesRecv >= bufsize - 16) {
			int bytesToDiscard = bufsize >> 1; 
			bytesRecv -= bytesToDiscard;
			memmove(buffer, buffer + bytesToDiscard, bytesRecv); 
		}
		byte n = xbRead(buffer + bytesRecv, bufsize - bytesRecv, timeout);
		if (n > 0) {
			if (!memcmp(buffer + bytesRecv, "$GSMNO DATA", 11)) {
				delay(100);
			} else {
				memmove(buffer + bytesRecv, buffer + bytesRecv + 4, n - 5);
				//Serial.print(buffer + bytesRecv);
				bytesRecv += n - 5;
				buffer[bytesRecv] = 0;
				if (!expected1)
					return 1;
				else if (strstr(buffer, expected1))
					return 1;
				else if (expected2 && strstr(buffer, expected2))
					return 2;
			}
		}
	} while (millis() - t < timeout);
	buffer[bytesRecv] = 0;
	return 0;
}

void COBDSPI::xbPurge()
{
	char buf[16];
	setTarget(TARGET_OBD);
	sendCommand("ATCLRGSM\r", buf, sizeof(buf));
}
