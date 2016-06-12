/*************************************************************************
* Firmware updating agent for Freematics ONE
* Distributed under GPL v2.0
* Visit http://freematics.com for more information
* (C)2016 Stanley Huang <support@freematics.com.au>
*************************************************************************/

#include <SPI.h>
#include <Wire.h>
#include <FreematicsONE.h>

COBDSPI one;

/*
void begin(byte pinCS, byte pinReady)
{
	m_pinCS = pinCS;
	m_pinReady = pinReady;
	pinMode(pinReady, INPUT);
	pinMode(pinCS, OUTPUT);
	digitalWrite(m_pinCS, HIGH);
	SPI.begin();
	SPI.setClockDivider(2);
}

byte receive(char* buffer, int timeout)
{
	int n = 0;
	bool eof = false;
	uint32_t t = millis();
	do {
		while (digitalRead(m_pinReady) == HIGH) {
		   if (millis() - t > 5000) {
		   	Serial.println("DATA READY TIMEOUT!");
        digitalWrite(m_pinCS, HIGH);
		    return 0;
		   }
		}
		digitalWrite(m_pinCS, LOW);
		for (; !eof && digitalRead(m_pinReady) == LOW; n++) {
		  byte data = SPI.transfer(' ');
                  buffer[n] = data;
		  eof = n > 2 && buffer[n] == '\t' && buffer[n - 1] =='>' && buffer[n - 2] == '\r';
		}
		digitalWrite(m_pinCS, HIGH);
	} while (!eof &&  millis() - t < timeout);

	buffer[n] = 0;
	return n;

}

*/

void setup() {
  one.begin();
  one.setTarget(TARGET_RAW);
  Serial.begin(115200);
}

/*
void sendData(char* Data,unsigned int SendLen)
{
  unsigned int i;
  digitalWrite(m_pinCS, LOW);
  delayMicroseconds(1000);
  for (i=0;i<SendLen;i++) {
    SPI.transfer((byte)Data[i]);
    delayMicroseconds(5);
  }
  digitalWrite(m_pinCS, HIGH);
}
*/

void doFirmwareUpdate()
{
  char buffer[1070];
  int bufbytes = 0;
  delay(1000);
  for (;;) {
    char c = Serial.read();
    if (bufbytes < sizeof(buffer) - 1) 
    {
      buffer[bufbytes++] = c;    
    }
    if (c == '\r') //
    {
    	buffer[bufbytes] = 0;
    	one.write((byte*)buffer,bufbytes);   
    	bufbytes = 0;
    	byte n = one.receive(buffer, sizeof(buffer), 5000);
    	if(n)
    	{
		Serial.write(buffer, n);
    	}
    }
    for (uint32_t t = millis(); !Serial.available() && millis() - t < 30000; t++);
    if (!Serial.available()) {
      // timeout
     return; 
    }
  }
}

char buffer[1070];
int bufbytes = 0;

void loop() {
  if (!Serial.available())
    return;

  char c = Serial.read();
  if (bufbytes < sizeof(buffer) - 1) 
  {
    buffer[bufbytes++] = c;    
  }
   if (c == '\r') //
   {
    	buffer[bufbytes] = 0;
    	one.write((byte*)buffer,bufbytes);   
    	bufbytes = 0;
    	byte n = one.receive(buffer, sizeof(buffer), 5000);
    	if(n)
    	{
    		Serial.write(buffer, n);
    	}
   }
}
