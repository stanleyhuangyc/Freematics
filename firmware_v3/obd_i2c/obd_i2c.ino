#include <Arduino.h>
#include <Wire.h>
#include <OBD.h>

#define I2C_ADDR 0x62
//#define DEBUG 1

#ifdef DEBUG
#include <SoftwareSerial.h>
SoftwareSerial debug(A2, A3);
#endif

uint32_t dataTime = 0;
uint8_t lastCmdCount = 0;
uint8_t cmdCount = 0;
uint16_t pidRequested = 0;

typedef union {
    byte data[2];
    uint16_t value;
} PID_DATA;

PID_DATA obdData[MAX_PIDS] = {0};
byte obdPid[MAX_PIDS] = {0}; //{PID_RPM, PID_SPEED, PID_THROTTLE, PID_ENGINE_LOAD};
uint32_t obdTime[MAX_PIDS] = {0};

char recvBuf[MAX_PAYLOAD_SIZE];
char sendBuf[OBD_RECV_BUF_SIZE];
byte recvBytes = 0;
uint8_t bytesToSend = 0;

void applyBuffer(char* buffer, byte bytes)
{
    if (bytesToSend) {
        uint32_t t = millis();
        while (bytesToSend && millis() - t < 1000);
        bytesToSend = 0;
    }
    memcpy(sendBuf, buffer, bytes);
    bytesToSend = bytes;
}

class COBDAgent : public COBD
{
public:
    void processCommand()
    {
        // check command
#ifdef DEBUG
        debug.print('[');
        debug.print(cmd.time);
        debug.print(']');
#endif

        switch (cmd.message) {
        case CMD_SEND_AT_COMMAND:
            {
#ifdef DEBUG
                debug.println("CMD_SEND_COMMAND");
#endif
                // send adapter command
                for (byte n = 0; n < recvBytes; n++) {
                    write(recvBuf[n]);
                }
                char dataBuf[OBD_RECV_BUF_SIZE] = {0};
                byte recvbytes = receive(dataBuf) + 1;
                applyBuffer(dataBuf, recvbytes);
            }
            break;
        case CMD_QUERY_STATUS:
            {
#ifdef DEBUG
                debug.println("CMD_QUERY_STATUS");
#endif

                if (getState() != OBD_CONNECTED || errors != 0)
                    connect();

                char dataBuf[MAX_PAYLOAD_SIZE];
                strcpy(dataBuf, "OBD ");
                switch (getState()) {
                case OBD_CONNECTED:
                    dataBuf[4] = 'Y';
                    break;
                case OBD_CONNECTING:
                    dataBuf[4] = 'C';
                    break;
                case OBD_DISCONNECTED:
                    dataBuf[4] = 'N';
                    break;
                }
                memcpy(dataBuf + 16, pidmap, 16);
                applyBuffer(dataBuf, MAX_PAYLOAD_SIZE);
            }
            break;
        }
    }
    void connect()
    {
        do {
#ifdef DEBUG
            debug.println("OBD Connecting...");
#endif
        } while (!init());
#ifdef DEBUG
        debug.println("OBD Connected!");
#endif
    }
    void processOBD(byte idx)
    {
        if (errors >= 2)
            connect();

        byte pid = obdPid[idx];
        sendQuery(pid);
        dataIdleLoop();
#ifdef DEBUG
        debug.print("PID[");
        debug.print(pid, HEX);
        debug.print("]=");
#endif
        char buffer[OBD_RECV_BUF_SIZE];
        byte n = receive(buffer);
        char* data = 0;
        char *p = buffer;
        while ((p = strstr(p, "41 "))) {
            p += 3;
            pid = hex2uint8(p);
            if (pid == obdPid[idx]) {
                errors = 0;
                p += 2;
                if (*p == ' ') {
                    data = p + 1;
                    break;
                }
            } else {
#ifdef DEBUG
                debug.print('[');
                debug.print(pid, HEX);
                debug.print(']');
#endif
                receive(buffer, 0);
                p = buffer;
            }
        }
#ifdef DEBUG
        debug.println(buffer);
#endif
        if (!data) {
            return;
        }
        obdTime[idx] = millis();
        if (data[3] < '0') {
            // A
            obdData[idx].value = hex2uint8(data);
        } else {
            // A/B
            obdData[idx].data[1] = hex2uint8(data);
            obdData[idx].data[0] = hex2uint8(data + 3);
        }
    }
    COMMAND_BLOCK cmd;
    bool obdReady;
private:
    byte parseData(char* hexdata, byte* buffer, byte bufsize)
    {
        byte n = 0;
        while(n < bufsize && *hexdata != '\r') {
            buffer[n++] = hex2uint8(hexdata);
            hexdata += 2;
            while (*hexdata == ' ') hexdata++;
        }
        return n;
    }
};

COBDAgent agent;

void I2CRecv(int numBytes)
{
    if (numBytes >= sizeof(COMMAND_BLOCK)) {
        Wire.readBytes((char*)&agent.cmd, sizeof(COMMAND_BLOCK));
        recvBytes = numBytes - sizeof(COMMAND_BLOCK);
        if (recvBytes > 0) {
            Wire.readBytes(recvBuf, recvBytes);
        }
        switch (agent.cmd.message) {
        case CMD_LOAD_OBD_DATA: {
            PID_INFO* pi = (PID_INFO*)sendBuf;
            memset(sendBuf, 0, sizeof(sendBuf));
            uint32_t t = millis();
            for (byte n = 0; n < MAX_PIDS && obdPid[n]; n++) {
                pi[n].value = obdData[n].value;
                pi[n].age = t - obdTime[n];
            }
            bytesToSend = sizeof(PID_INFO) * MAX_PIDS;
            } break;
        case CMD_APPLY_OBD_PIDS:
            memcpy(obdPid, recvBuf, min(recvBytes, sizeof(obdPid)));
            memset(obdTime, 0, sizeof(obdTime));
            memset(obdData, 0, sizeof(obdData));
            break;
        default:
            cmdCount++;
        }
    } else {
        // unload the data
        do {
            if (Wire.available()) {
                Wire.read();
                numBytes--;
            }
        } while (numBytes > 0);
    }
}

void I2CRequest()
{
    if (!bytesToSend) {
        for (byte n = MAX_PAYLOAD_SIZE; n; n--)
            Wire.write((byte)0);
        return;
    }

    Wire.write((byte*)sendBuf, MAX_PAYLOAD_SIZE);
    if (bytesToSend > MAX_PAYLOAD_SIZE) {
        memmove(sendBuf, sendBuf + MAX_PAYLOAD_SIZE, bytesToSend - MAX_PAYLOAD_SIZE);
        bytesToSend -= MAX_PAYLOAD_SIZE;
    } else {
        bytesToSend = 0;
    }
}

void setup()
{
#ifdef DEBUG
    debug.begin(9600);
#endif
    delay(500);

    agent.begin();

    Wire.onReceive(I2CRecv);
    Wire.onRequest(I2CRequest);
    Wire.begin(I2C_ADDR);
}

void loop()
{
    static byte idx = 0;
    if (agent.getState() != OBD_CONNECTED) {
        agent.connect();
    }
    if (cmdCount != lastCmdCount) {
        agent.processCommand();
        lastCmdCount = cmdCount;
    } else if (obdPid[0] && agent.getState() == OBD_CONNECTED) {
        if (obdPid[idx] == 0 || idx == MAX_PIDS)
            idx = 0;

        if (obdPid[idx]) {
            agent.processOBD(idx);
        }
        idx++;
    }
}
