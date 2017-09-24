/*************************************************************************
* Arduino Library for Freematics ONE/ONE+
* Distributed under BSD license
* Visit http://freematics.com/products/freematics-one for more information
* (C)2012-2017 Stanley Huang <support@freematics.com.au
*************************************************************************/

#ifdef ARDUINO_ARCH_AVR
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/common.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#endif
#include "FreematicsONE.h"

//#define XBEE_DEBUG
//#define DEBUG Serial

#ifdef ARDUINO_ARCH_AVR
SIGNAL(WDT_vect) {
  wdt_disable();
  wdt_reset();
  WDTCSR &= ~_BV(WDIE);
}
#endif

#ifdef ESP32
bool gps_decode_start();
bool gps_get_data(GPS_DATA* gdata);
int gps_write_string(const char* string);
void gps_decode_task(int timeout);
void bee_start();
int bee_write_string(const char* string);
int bee_write_data(uint8_t* data, int len);
int bee_read(uint8_t* buffer, size_t bufsize, unsigned int timeout);
void bee_flush();
#endif

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
	sleep(20);
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

void COBDSPI::enterLowPowerMode()
{
#ifdef ESP32
	reset();
	end();
#else
	setTarget(TARGET_OBD);
	write("ATLP\r");
#endif
}

void COBDSPI::leaveLowPowerMode()
{
#ifdef ESP32
	begin();
#else
	char buf[16];
	sendCommand("AT\r", buf, sizeof(buf));
#endif
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

bool COBDSPI::getVIN(char* buffer, byte bufsize)
{
	setTarget(TARGET_OBD);
	if (sendCommand("0902\r", buffer, bufsize, OBD_TIMEOUT_LONG * 2)) {
	    char *p = strstr(buffer, "49 02");
	    if (p) {
	        char *q = buffer;
	        p += 10;
	        do {
	            for (++p; *p == ' '; p += 3) {
	                if ((*q = hex2uint8(p + 1))) q++;
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

void COBDSPI::reset()
{
	char buf[32];
	sendCommand("ATR\r", buf, sizeof(buf));
	delay(3000);
}

void COBDSPI::uninit()
{
	char buf[32];
	sendCommand("ATPC\r", buf, sizeof(buf));
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
		for (byte i = 0; i < sizeof(initcmd) / sizeof(initcmd[0]); i++) {
			if (!sendCommand(initcmd[i], buffer, sizeof(buffer), OBD_TIMEOUT_SHORT)) {
				continue;
			}
		}
		stage = 1;
		if (protocol != PROTO_AUTO) {
			sprintf_P(buffer, PSTR("ATSP%u\r"), protocol);
			if (!sendCommand(buffer, buffer, sizeof(buffer), OBD_TIMEOUT_SHORT) || !strstr_P(buffer, PSTR("OK"))) {
				continue;
			}
		}
		stage = 2;
		if (!sendCommand("010D\r", buffer, sizeof(buffer), OBD_TIMEOUT_LONG) || checkErrorMessage(buffer)) {
			continue;
		}
		stage = 3;
		// load pid map
		memset(pidmap, 0, sizeof(pidmap));
		bool success = false;
		for (byte i = 0; i < 4; i++) {
			byte pid = i * 0x20;
			sendQuery(pid);
			if (receive(buffer, sizeof(buffer), OBD_TIMEOUT_LONG) > 0) {
				if (checkErrorMessage(buffer)) {
					break;
				}
				char *p = buffer;
				while ((p = strstr(p, "41 "))) {
					p += 3;
					if (hex2uint8(p) == pid) {
						p += 2;
						for (byte n = 0; n < 4 && *(p + n * 3) == ' '; n++) {
							pidmap[i * 4 + n] = hex2uint8(p + n * 3 + 1);
						}
						success = true;
					}
				}
			} else {
				break;
			}
		}
		if (success) {
			stage = 0xff;
			break;
		}
	}
	if (stage == 0xff) {
		m_state = OBD_CONNECTED;
		errors = 0;
		return true;
	} else {
		Serial.print("Stage:");
		Serial.println(stage);
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


static const char targets[][4] = {
	{'$','O','B','D'},
	{'$','G','P','S'},
	{'$','G','S','M'}
};

//SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0);

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
#ifdef ESP32
	SPI.setFrequency(1000000);
#else
	SPI.setClockDivider(SPI_CLOCK_DIV64);
#endif
	delay(50);
	return getVersion();
}

void COBDSPI::end()
{
#ifdef ESP32
	SPI.end();
#endif
}

byte COBDSPI::getVersion()
{
	byte version = 0;
	setTarget(TARGET_OBD);
	for (byte n = 0; n < 3; n++) {
		char buffer[32];
		if (sendCommand("ATI\r", buffer, sizeof(buffer), 500)) {
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
	uint32_t t = millis();
	do {
		while (digitalRead(SPI_PIN_READY) == HIGH) {
			sleep(1);
			if (millis() - t > timeout) {
#ifdef DEBUG
				debugOutput("NO READY SIGNAL");
#endif
				break;
			}
		}
		sleep(10);
		digitalWrite(SPI_PIN_CS, LOW);
		while (digitalRead(SPI_PIN_READY) == LOW && millis() - t < timeout) {
			char c = SPI.transfer(' ');
			if (eos) continue;
			if (n == 0) {
				// match header char before we can move forward
				if (c == '$') {
					buffer[0] = c;
					n = 1;
				}
				continue;
			}
			if (n > 6 && buffer[1] == 'O' && c == '.' && buffer[n - 1] == '.' && buffer[n - 2] == '.') {
				// $OBDSEARCHING...
				n = 4;
				timeout += OBD_TIMEOUT_LONG;
			} else if (c != 0 && c != 0xff) {
				if (n == bufsize - 1) {
					int bytesDumped = dumpLine(buffer, n);
					n -= bytesDumped;
#ifdef DEBUG
					debugOutput("BUFFER FULL");
#endif
				}
				buffer[n] = c;
				eos = (c == 0x9 && buffer[n - 1] =='>');
				n++;
#ifndef ESP32
				if (eos) break;
#endif
			}
		}
#ifndef ESP32
		sleep(1);
#endif
		digitalWrite(SPI_PIN_CS, HIGH);
#ifndef ESP32
		sleep(1);
#endif
	} while (!eos && millis() - t < timeout);
#ifdef DEBUG
	if (!eos && millis() - t >= timeout) {
		// timed out
		debugOutput("RECV TIMEOUT");
	}
#endif
	if (m_target != TARGET_RAW && eos) {
		// eliminate ending char
		n--;
	}
	buffer[n] = 0;
#ifdef DEBUG
	if (m_target == TARGET_OBD) {
		debugOutput(buffer);
	}
#endif
	// wait for READY pin to restore high level so SPI bus is released
	while (digitalRead(SPI_PIN_READY) == LOW) sleep(1);
	return n;
}

void COBDSPI::write(const char* s)
{
#ifdef DEBUG
	debugOutput(s);
#endif
#if ESP32
	char buf[256];
	int len = strlen(s);
	if (len > sizeof(buf) - 6) len = sizeof(buf) - 6;
	memcpy(buf, targets[m_target], 4);
	memcpy(buf + 4, s, len);
	len += 4;
	// add terminating byte (ESC)
	buf[len++] = 0x1B;
	sleep(10);
#else
	sleep(1);
#endif
	digitalWrite(SPI_PIN_CS, LOW);
	sleep(1);
	//SPI.beginTransaction(spiSettings);
#ifdef ESP32
	SPI.writeBytes((uint8_t*)buf, len);
#else
	if (*s != '$') {
		for (byte i = 0; i < sizeof(targets[0]); i++) {
			SPI.transfer(targets[m_target][i]);
		}
	}
	for (; *s ;s++) {
		SPI.transfer((byte)*s);
	}
	// send terminating byte (ESC)
	SPI.transfer(0x1B);
#endif
	sleep(1);
	//SPI.endTransaction();
	digitalWrite(SPI_PIN_CS, HIGH);
	sleep(1);
}

void COBDSPI::write(byte* data, int len)
{
	digitalWrite(SPI_PIN_CS, LOW);
	//SPI.beginTransaction(spiSettings);
#ifdef ESP32
	SPI.writeBytes((uint8_t*)data, len);
#else
	for (int i = 0; i < len; i++) {
		SPI.transfer(data[i]);
	}
#endif
	sleep(1);
	//SPI.endTransaction();
	digitalWrite(SPI_PIN_CS, HIGH);
	sleep(1);
}

byte COBDSPI::readPID(const byte pid[], byte count, int result[])
{
	byte results = 0;
	for (byte n = 0; n < count; n++) {
		if (readPID(pid[n], result[n])) {
			results++;
		}
		sleep(5);
	}
	return results;
}

byte COBDSPI::sendCommand(const char* cmd, char* buf, byte bufsize, unsigned int timeout)
{
	uint32_t t = millis();
	byte n;
	do {
		write(cmd);
		sleep(20);
		n = receive(buf, bufsize, timeout);
		if (n == 0 || (buf[1] != 'O' && !memcmp(buf + 7, " DATA", 5))) {
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
#ifdef ESP32
	uint32_t t = millis();
	for (;;) {
        uint32_t elapsed = millis() - t;
        if (elapsed >= ms) break;
		gps_decode_task(ms - elapsed);
	}
#else
	delay(ms);
#endif
}

void COBDSPI::sleepSec(unsigned int seconds)
{
#ifdef ARDUINO_ARCH_AVR
	while (seconds > 0) {
		uint8_t wdt_period;
		if (seconds >= 8) {
			wdt_period = WDTO_8S;
			seconds -= 8;
		} else {
			wdt_period = WDTO_1S;
			seconds--;
		}
		wdt_enable(wdt_period);
		wdt_reset();
		WDTCSR |= _BV(WDIE);
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
		sleep_mode();
		wdt_disable();
		WDTCSR &= ~_BV(WDIE);
	 }
#else
	sleep(seconds * 1000);
#endif
}

bool COBDSPI::gpsInit(unsigned long baudrate)
{
	bool success = false;
	char buf[64];
	m_target = TARGET_OBD;
	m_internalGPS = false;
	if (baudrate) {
#ifdef ESP32
		pinMode(PIN_GPS_POWER, OUTPUT);
		// turn on GPS power
		digitalWrite(PIN_GPS_POWER, HIGH);
		if (gps_decode_start()) {
			// success
			return true;
		}
#endif
		if (sendCommand("ATGPSON\r", buf, sizeof(buf))) {
			sprintf_P(buf, PSTR("ATBR2%lu\r"), baudrate);
			sendCommand(buf, buf, sizeof(buf));
			uint32_t t = millis();
			delay(100);
			do {
				if (gpsGetRawData(buf, sizeof(buf)) && strstr(buf, "S$G")) {
#ifdef ESP32
					m_internalGPS = true;
#endif
					return true;
				}
			} while (millis() - t < GPS_INIT_TIMEOUT);
		}
	} else {
		success = true;
	}
	//turn off GPS power
#ifdef ESP32
	digitalWrite(PIN_GPS_POWER, LOW);
#endif
	sendCommand("ATGPSOFF\r", buf, sizeof(buf));
	return success;
}

bool COBDSPI::gpsGetData(GPS_DATA* gdata)
{
#ifdef ESP32
	if (!m_internalGPS) {
		return gps_get_data(gdata);
	}
#endif

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
#if 0 //def ESP32
	if (!m_internalGPS && uartGPS) {
		int bytes = 0;
		for (uint32_t t = millis(); uartGPS.available() && millis() - t < GPS_READ_TIMEOUT && bytes < bufsize - 1;) {
			char c = uartGPS.read();
			gps.encode(c);
			buf[bytes++] = c;
		}
		buf[bytes] = 0;
		return bytes;
	}
#endif
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
#ifdef ESP32
	if (!m_internalGPS) {
		gps_write_string(cmd);
		return;
	}
#endif
	setTarget(TARGET_GPS);
	write(cmd);
}

bool COBDSPI::xbBegin(unsigned long baudrate)
{
#ifdef PIN_XBEE_PWR
	pinMode(PIN_XBEE_PWR, OUTPUT);
	digitalWrite(PIN_XBEE_PWR, HIGH);
#endif
#ifdef ESP32
	bee_start();
#else
	char buf[16];
	sprintf_P(buf, PSTR("ATBR1%lu\r"), baudrate);
	setTarget(TARGET_OBD);
	if (sendCommand(buf, buf, sizeof(buf))) {
		xbPurge();
		return true;
	} else {
		return false;
	}
#endif
}

void COBDSPI::xbWrite(const char* data)
{
#ifdef ESP32
	bee_write_string(data);
#else
	setTarget(TARGET_BEE);
	write(data);
	sleep(10);
#endif
#ifdef XBEE_DEBUG
	Serial.print("[SENT]");
	Serial.print(data);
	Serial.println("[/SENT]");
#endif
}

void COBDSPI::xbWrite(const char* data, int len)
{
#ifdef ESP32
	bee_write_data((uint8_t*)data, len);
#else
	xbWrite((const char*)data);
#endif
}

int COBDSPI::xbRead(char* buffer, int bufsize, unsigned int timeout)
{
#ifdef ESP32
	return bee_read((uint8_t*)buffer, bufsize, timeout);
#else
	setTarget(TARGET_OBD);
	write("ATGRD\r");
	return receive(buffer, bufsize, timeout);
#endif
}

int COBDSPI::xbReceive(char* buffer, int bufsize, unsigned int timeout, const char** expected, byte expectedCount)
{
	int bytesRecv = 0;
	uint32_t t = millis();
	do {
		if (bytesRecv >= bufsize - 16) {
			bytesRecv -= dumpLine(buffer, bytesRecv);
		}
		int n = xbRead(buffer + bytesRecv, bufsize - bytesRecv - 1, 100);
		if (n > 0) {
#ifdef ESP32
#ifdef XBEE_DEBUG
			Serial.print("[RECV]");
			buffer[bytesRecv + n] = 0;
			Serial.print(buffer + bytesRecv);
			Serial.println("[/RECV]");
#endif
			bytesRecv += n;
			buffer[bytesRecv] = 0;
			for (byte i = 0; i < expectedCount; i++) {
				// match expected string(s)
				if (expected[i] && strstr(buffer, expected[i])) return i + 1;
			}
#else
			if (!memcmp(buffer + bytesRecv, "$GSM", 4) && memcmp(buffer + bytesRecv + 6, " DATA", 5)) {
				n -= 4;
				memmove(buffer + bytesRecv, buffer + bytesRecv + 4, n);
				bytesRecv += n;
				buffer[bytesRecv] = 0;
#ifdef XBEE_DEBUG
				Serial.print("[RECV]");
				Serial.print(buffer + bytesRecv - n);
				Serial.println("[/RECV]");
#endif
				if (expectedCount == 0) {
					break;
				}
				for (byte i = 0; i < expectedCount; i++) {
					// match expected string(s)
					if (expected[i] && strstr(buffer, expected[i])) return i + 1;
				}
			}
#endif
		} else if (n == -1) {
			// an erroneous reading
#ifdef XBEE_DEBUG
			Serial.print("RECV ERROR");
#endif
			break;
		}
#ifndef ESP32
		sleep(100);
#endif
	} while (millis() - t < timeout);
	buffer[bytesRecv] = 0;
	return 0;
}

void COBDSPI::xbPurge()
{
#ifdef ESP32
	bee_flush();
#else
	char buf[16];
	setTarget(TARGET_OBD);
	sendCommand("ATCLRGSM\r", buf, sizeof(buf));
#endif
}

void COBDSPI::xbTogglePower()
{
#ifdef PIN_XBEE_PWR
	digitalWrite(PIN_XBEE_PWR, HIGH);
	sleep(50);
#ifdef XBEE_DEBUG
	Serial.println("xBee power pin set to low");
#endif
	digitalWrite(PIN_XBEE_PWR, LOW);
	sleep(2000);
#ifdef XBEE_DEBUG
	Serial.println("xBee power pin set to high");
#endif
	digitalWrite(PIN_XBEE_PWR, HIGH);
	sleep(1000);
	digitalWrite(PIN_XBEE_PWR, LOW);
#else
	setTarget(TARGET_OBD);
	char buffer[64];
	sendCommand("ATGSMPWR\r", buffer, sizeof(buffer));
#endif
}
