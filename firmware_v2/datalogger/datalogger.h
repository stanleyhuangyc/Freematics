#define FORMAT_BIN 0
#define FORMAT_CSV 1

typedef struct {
    uint32_t time;
    uint16_t pid;
    uint8_t flags;
    uint8_t checksum;
    float value[3];
} LOG_DATA_COMM;

#define HEADER_LEN 128 /* bytes */

#define PID_GPS_LATITUDE 0xF00A
#define PID_GPS_LONGITUDE 0xF00B
#define PID_GPS_ALTITUDE 0xF00C
#define PID_GPS_SPEED 0xF00D
#define PID_GPS_HEADING 0xF00E
#define PID_GPS_SAT_COUNT 0xF00F
#define PID_GPS_TIME 0xF010
#define PID_GPS_DATE 0xF011

#define PID_ACC 0xF020
#define PID_GYRO 0xF021

#define FILE_NAME_FORMAT "/DAT%05d.CSV"

#if ENABLE_DATA_OUT

#if USE_SOFTSERIAL

#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
    SoftwareSerial SerialBLE(A8, A9); /* for BLE Shield on MEGA*/
#elif defined(__AVR_ATmega644P__)
    SoftwareSerial SerialBLE(9, 10); /* for Microduino */
#else
    SoftwareSerial SerialBLE(A2, A3); /* for BLE Shield on UNO/leonardo*/
#endif

#else

#define SerialBLE Serial

#endif

#endif

#if ENABLE_DATA_LOG
File sdfile;
#endif

static const char* idstr = "FREEMATICS\r";

class CDataLogger {
public:
    void initSender()
    {
#if ENABLE_DATA_OUT
        SerialBLE.begin(STREAM_BAUDRATE);
        SerialBLE.print(idstr);
#endif
#if ENABLE_DATA_LOG
        m_lastDataTime = 0;
#endif
    }
    void logTimeElapsed()
    {
#if ENABLE_DATA_LOG
        dataSize += sdfile.print(dataTime - m_lastDataTime);
        sdfile.write(',');
        dataSize++;
        m_lastDataTime = dataTime;
#endif
    }
    void logData(char c)
    {
#if ENABLE_DATA_OUT && STREAM_FORMAT == FORMAT_CSV
        SerialBLE.write(c);
#endif
#if ENABLE_DATA_LOG
        if (c >= ' ') {
            sdfile.write(c);
            dataSize++;
        }
#endif
    }
    void logData(uint16_t pid, int value)
    {
#if STREAM_FORMAT == FORMAT_BIN
        LOG_DATA_COMM ld = {dataTime, pid, 1, 0, value};
        ld.checksum = getChecksum((char*)&ld, 12);
#endif
#if ENABLE_DATA_OUT
#if STREAM_FORMAT == FORMAT_BIN
        SerialBLE.write((uint8_t*)&ld, 12);
#else
        SerialBLE.print(pid, HEX);
        SerialBLE.write(',');
        SerialBLE.print(value);
        SerialBLE.write('\r');
#endif
#endif
#if ENABLE_DATA_LOG
        logTimeElapsed();
        dataSize += sdfile.print(pid, HEX);
        dataSize += sdfile.write(',');
        dataSize += sdfile.print(value);
        dataSize += sdfile.write('\r');
#endif
    }
    void logData(uint16_t pid, int32_t value)
    {
#if STREAM_FORMAT == FORMAT_BIN
        LOG_DATA_COMM ld = {dataTime, pid, 1, 0, value};
        ld.checksum = getChecksum((char*)&ld, 12);
#endif
#if ENABLE_DATA_OUT
#if STREAM_FORMAT == FORMAT_BIN
        SerialBLE.write((uint8_t*)&ld, 12);
#else
        SerialBLE.print(pid, HEX);
        SerialBLE.write(',');
        SerialBLE.print(value);
        SerialBLE.write('\r');
#endif
#endif
#if ENABLE_DATA_LOG
        logTimeElapsed();
        dataSize += sdfile.print(pid, HEX);
        dataSize += sdfile.write(',');
        dataSize += sdfile.print(value);
        dataSize += sdfile.write('\r');
#endif
    }
    void logData(uint16_t pid, float value)
    {
#if STREAM_FORMAT == FORMAT_BIN
        LOG_DATA_COMM ld = {dataTime, pid, 1, 0, value};
        ld.checksum = getChecksum((char*)&ld, 12);
#endif
#if ENABLE_DATA_OUT
        SerialBLE.print(pid, HEX);
        SerialBLE.write(',');
        SerialBLE.print(value);
        SerialBLE.write('\r');
#endif
#if ENABLE_DATA_LOG
        logTimeElapsed();
        dataSize += sdfile.print(pid, HEX);
        dataSize += sdfile.write(',');
        dataSize += sdfile.print(value);
        dataSize += sdfile.write('\r');
#endif
    }
    void logData(uint16_t pid, int value1, int value2, int value3)
    {
#if STREAM_FORMAT == FORMAT_BIN
        LOG_DATA_COMM ld = {dataTime, pid, 3, 0, {value1, value2, value3}};
        ld.checksum = getChecksum((char*)&ld, 20);
#endif
#if ENABLE_DATA_OUT
#if STREAM_FORMAT == FORMAT_BIN
        SerialBLE.write((uint8_t*)&ld, 20);
#else
        SerialBLE.print(pid, HEX);
        SerialBLE.write(',');
        SerialBLE.print(value1);
        SerialBLE.write(',');
        SerialBLE.print(value2);
        SerialBLE.write(',');
        SerialBLE.print(value3);
        SerialBLE.write('\r');
#endif
#endif
#if ENABLE_DATA_LOG
        logTimeElapsed();
        dataSize += sdfile.print(pid, HEX);
        sdfile.write(',');
        dataSize += sdfile.print(value1);
        sdfile.write(',');
        dataSize += sdfile.print(value2);
        sdfile.write(',');
        dataSize += sdfile.print(value3);
        sdfile.write('\r');
        dataSize += 4;
#endif
    }
#if ENABLE_DATA_LOG
    uint16_t openFile(uint16_t logFlags = 0, uint32_t dateTime = 0)
    {
        uint16_t fileIndex;
        char filename[24] = "/FRMATICS";

        if (SD.exists(filename)) {
            for (fileIndex = 1; fileIndex; fileIndex++) {
                sprintf(filename + 9, FILE_NAME_FORMAT, fileIndex);
                if (!SD.exists(filename)) {
                    break;
                }
            }
            if (fileIndex == 0)
                return 0;
        } else {
            SD.mkdir(filename);
            fileIndex = 1;
            sprintf(filename + 9, FILE_NAME_FORMAT, 1);
        }

        sdfile = SD.open(filename, FILE_WRITE);
        if (!sdfile) {
            return 0;
        }

        dataSize = sdfile.print(idstr);
        return fileIndex;
    }
    void closeFile()
    {
        sdfile.close();
    }
    void flushFile()
    {
        sdfile.flush();
    }
#endif
    uint32_t dataTime;
    uint32_t dataSize;
private:
    byte getChecksum(char* buffer, byte len)
    {
        uint8_t checksum = 0;
        for (byte i = 0; i < len; i++) {
          checksum ^= buffer[i];
        }
        return checksum;
    }
#if ENABLE_DATA_LOG
    uint32_t m_lastDataTime;
#endif
};
