/*************************************************************************
* This sketch redirects serial UART I/O to OBD chip via SPI
* to emulate a ELM327 device (with extensions) by Freematics ONE
* Distributed under GPL v2.0
* Visit http://freematics.com for more information
* Developed by Stanley Huang <stanleyhuangyc@gmail.com>
*************************************************************************/

#include <SPI.h>

#define SERIAL_BAUDRATE 115200
#define SPI_TIMEOUT 5000
#define PIN_CS 7
#define PIN_READY 6

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  // initialize SPI connection
  pinMode(PIN_READY, INPUT);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin();
  SPI.setClockDivider(2);
  Serial.println("Freematics ONE ELM327 Emulator");
}

byte receive(char* buffer)
{
	int n = 0;
	bool eof = false;
	uint32_t t = millis();
	do {
		while (digitalRead(PIN_READY) == HIGH) {
		   if (millis() - t > SPI_TIMEOUT) {
                      digitalWrite(PIN_CS, HIGH);
                      Serial.println("TIMEOUT!");
  		      return 0;
		   }
		}
		digitalWrite(PIN_CS, LOW);
		for (; !eof && digitalRead(PIN_READY) == LOW; n++) {
		  byte data = SPI.transfer(' ');
                  buffer[n] = data;
		  eof = n > 2 && buffer[n] == '\t' && buffer[n - 1] =='>' && buffer[n - 2] == '\r';
		}
		digitalWrite(PIN_CS, HIGH);
	} while (!eof &&  millis() - t < SPI_TIMEOUT);
	if (eof) n--;
	buffer[n] = 0;
	return n;
}

static char prefix[5] = "$OBD";

void sendData(char* data, unsigned int len)
{
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(1000);
  if (data[0] != '$') {
    for (int i = 0; prefix[i]; i++) {
      SPI.transfer((byte)prefix[i]);
      delayMicroseconds(5);
    }
  }
  for (int i = 0; i < len; i++) {
    SPI.transfer((byte)data[i]);
    delayMicroseconds(5);
  }
  digitalWrite(PIN_CS, HIGH);
}

void loop() {
  static char buffer[1072];
  static int bufbytes = 0;

  if (!Serial.available())
    return;

  char c = Serial.read();
  if (bufbytes < sizeof(buffer) - 1) 
  {
    buffer[bufbytes++] = c;    
  }
  if (c == '\r')
  {
    // a complete line is received
    buffer[bufbytes] = 0;
    if (buffer[0] == '$' && buffer[4] == '\r') {
      memcpy(prefix, buffer, 4);
      Serial.print("Prefix set to ");
      Serial.println(prefix);
    } else {
      sendData(buffer,bufbytes);
      if ((!memcmp(buffer, "$GSM", 4) && buffer[4]) || !memcmp(prefix, "$GSM", 4)) {
        delay(500);
        sendData("$OBDATGRD\r", 10);
      }
    }

    byte n = receive(buffer);
    if (n) {
      // strip $OBD header from response
      if (!memcmp(buffer, "$OBD", 4)) {
        Serial.write(buffer + 4, n - 4);
      } else {
        Serial.write(buffer, n);
      }
    }
    // buffer is now empty
    bufbytes = 0;
  }
}
