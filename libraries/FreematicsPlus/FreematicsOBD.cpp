/*************************************************************************
* Arduino Library for Freematics ONE/ONE+
* Distributed under BSD license
* Visit http://freematics.com/products/freematics-one for more information
* (C)2012-2017 Stanley Huang <support@freematics.com.au
*************************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include "driver/uart.h"
#include "FreematicsOBD.h"

//#define DEBUG Serial

//static SPISettings settings = SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0);

#ifdef DEBUG
void debugOutput(const char *s)
{
	DEBUG.print('[');
	DEBUG.print(millis());
	DEBUG.print(']');
	DEBUG.println(s);
}
#endif

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

int COBD::sendCommand(const char* cmd, char* buf, int bufsize, unsigned int timeout)
{
	write(cmd);
	return receive(buf, bufsize, timeout);
}

bool COBD::readPID(byte pid, int& result)
{
	char buffer[64];
	char* data = 0;
	sprintf(buffer, "%02X%02X\r", dataMode, pid);
	write(buffer);
	delay(1);
	uint32_t t = millis();
	int ret = receive(buffer, sizeof(buffer), pidWaitTime);
	pidWaitTime = millis() - t + (pidWaitTime >> 1);
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
		recover();
		return false;
	}
	result = normalizeData(pid, data);
	return true;
/*
	char buffer[64];
	sprintf(buffer, "%02X%02X\r", dataMode, pid);
	write(buffer);
	char* data = getResponse(pid, buffer, sizeof(buffer));
	if (!data) {
		recover();
		errors++;
		return false;
	}
	result = normalizeData(pid, data);
	return true;
*/
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

void COBD::clearDTC()
{
	char buffer[32];
	write("04\r");
	receive(buffer, sizeof(buffer));
}

void COBD::write(const char* s)
{
#ifdef DEBUG
	DEBUG.print("<<<");
	DEBUG.println(s);
#endif
	uart_write_bytes(OBD_UART_NUM, s, strlen(s));
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
	while (receive(buffer, bufsize) > 0) {
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
	sendCommand("ATLP\r", buf, sizeof(buf));
}

void COBD::leaveLowPowerMode()
{
	// simply send any command to wake the device up
	char buf[32];
	sendCommand("ATI\r", buf, sizeof(buf), 1000);
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
	if (sendCommand("ATRV\r", buf, sizeof(buf), 100) > 0) {
		char* p = getResultValue(buf);
		if (p) return (float)atof(p);
    }
    return 0;
}

bool COBD::getVIN(char* buffer, byte bufsize)
{
	for (byte n = 0; n < 2; n++) {
		if (sendCommand("0902\r", buffer, bufsize)) {
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

byte COBD::begin(unsigned long baudrate)
{
    uart_config_t uart_config = {
        .baud_rate = OBD_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    //Configure UART parameters
    uart_param_config(OBD_UART_NUM, &uart_config);
    //Set UART pins
    uart_set_pin(OBD_UART_NUM, PIN_OBD_UART_TX, PIN_OBD_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Install UART driver
    if (uart_driver_install(OBD_UART_NUM, OBD_UART_BUF_SIZE, 0, 0, NULL, 0) != ESP_OK)
		return 0;
	byte ver = getVersion();
	if (ver == 0) end();
	return ver;
}

byte COBD::getVersion()
{
	byte version = 0;
	for (byte n = 0; n < 3; n++) {
		char buffer[32];
		if (sendCommand("ATI\r", buffer, sizeof(buffer), 200)) {
			char *p = strchr(buffer, ' ');
			if (p) {
				p += 2;
				version = (*p - '0') * 10 + (*(p + 2) - '0');
				break;
			}
		}
	}
	return version;
}

int COBD::receive(char* buffer, int bufsize, unsigned int timeout)
{
	unsigned char n = 0;
	unsigned long startTime = millis();
	unsigned long elapsed;
	for (;;) {
		elapsed = millis() - startTime;
		if (elapsed > timeout) break;
		if (n >= bufsize - 1) break;
		int len = uart_read_bytes(OBD_UART_NUM, (uint8_t*)buffer + n, bufsize - n - 1, 1);
		if (len < 0) break;
		if (len == 0) continue;
		buffer[n + len] = 0;
		if (strchr(buffer + n, '>')) {
			n += len;
			break;
		}
		n += len;
		if (strstr(buffer, "...")) {
			buffer[0] = 0;
			n = 0;
			timeout += OBD_TIMEOUT_LONG;
		}
	}
#ifdef DEBUG
	DEBUG.print(">>>");
	DEBUG.println(buffer);
#endif
	return n;
}

void COBD::recover()
{
	sendCommand("\r", 0, 0);
}

bool COBD::init(OBD_PROTOCOLS protocol)
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

void COBD::end()
{
	m_state = OBD_DISCONNECTED;
	uart_driver_delete(OBD_UART_NUM);
}

bool COBD::setBaudRate(unsigned long baudrate)
{
	char buf[32];
	sprintf(buf, "ATBR1 %lu\r", baudrate);
	sendCommand(buf, buf, sizeof(buf));
	delay(50);
	end();
	return begin(baudrate) != 0;
}

void COBD::reset()
{
	char buf[32];
	sendCommand("ATR\r", buf, sizeof(buf));
	delay(3000);
	sendCommand("ATZ\r", buf, sizeof(buf));
}

void COBD::uninit()
{
	char buf[32];
	sendCommand("ATPC\r", buf, sizeof(buf));
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

/*************************************************************************
* OBD-II SPI bridge
*************************************************************************/

byte COBDSPI::begin(unsigned long freq)
{
	pinMode(SPI_PIN_READY, INPUT);
	pinMode(SPI_PIN_CS, OUTPUT);
	digitalWrite(SPI_PIN_CS, HIGH);
	SPI.begin();
	SPI.setFrequency(freq);
	//delay(50);
	byte ver = getVersion();
	return ver;
}

void COBDSPI::end()
{
	SPI.end();
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
#ifdef DEBUG
				debugOutput("NO READY SIGNAL");
#endif
				break;
			}
		}
		digitalWrite(SPI_PIN_CS, LOW);
		while (digitalRead(SPI_PIN_READY) == LOW && millis() - t < timeout) {
			char c = SPI.transfer(' ');
			if (c == 0 && c == 0xff) continue;
			if (!eos) eos = (c == 0x9);
			if (eos) continue;
			if (!matched) {
				// match header
				if (n == 0 && c != header[0]) continue;
				if (n == bufsize - 1) continue;
				buffer[n++] = c;
				if (n == sizeof(header)) {
					matched = memcmp(buffer, header, sizeof(header)) == 0;
					if (matched) {
						n = 0;
					} else {
						memmove(buffer, buffer + 1, --n);
					}
				}
				continue;
			}
			if (n > 3 && c == '.' && buffer[n - 1] == '.' && buffer[n - 2] == '.') {
				// SEARCHING...
				n = 0;
				timeout += OBD_TIMEOUT_LONG;
			} else {
				if (n == bufsize - 1) {
					int bytesDumped = dumpLine(buffer, n);
					n -= bytesDumped;
#ifdef DEBUG
					debugOutput("BUFFER FULL");
#endif
				}
				buffer[n++] = c;
			}
		}
		digitalWrite(SPI_PIN_CS, HIGH);
	} while (!eos && millis() - t < timeout);
#ifdef DEBUG
	if (!eos && millis() - t >= timeout) {
		// timed out
		debugOutput("RECV TIMEOUT");
	}
#endif
	buffer[n] = 0;
#ifdef DEBUG
	debugOutput(buffer);
#endif
	// wait for READY pin to restore high level so SPI bus is released
	while (digitalRead(SPI_PIN_READY) == LOW) delay(1);
	return n;
}

void COBDSPI::write(const char* s)
{
#ifdef DEBUG
	debugOutput(s);
#endif
	int len = strlen(s);
	uint8_t tail = 0x1B;
	//SPI.beginTransaction(settings);
	digitalWrite(SPI_PIN_CS, LOW);
	delay(1);
	SPI.writeBytes((uint8_t*)header, sizeof(header));
	SPI.writeBytes((uint8_t*)s, len);
	SPI.writeBytes((uint8_t*)&tail, 1);
	delay(1);
	digitalWrite(SPI_PIN_CS, HIGH);
	//SPI.endTransaction();
}

int COBDSPI::sendCommand(const char* cmd, char* buf, int bufsize, unsigned int timeout)
{
	uint32_t t = millis();
	int n;
	do {
		write(cmd);
		delay(1);
		n = receive(buf, bufsize, timeout);
		if (n == 0 || (buf[1] != 'O' && !memcmp(buf + 5, "NO DATA", 7))) {
			// data not ready
			delay(50);
		} else {
	  		break;
		}
	} while (millis() - t < timeout);
	return n;
}

COBD* createOBD()
{
    COBD* obd = new COBD;
    if (!obd->begin()) {
        delete obd;    
        obd = new COBDSPI;
        obd->begin();
    }
	return obd;
}
