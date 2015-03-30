#define FORMAT_BIN 0
#define FORMAT_CSV 1

typedef struct {
    uint32_t time;
    uint16_t pid;
    uint8_t flags;
    uint8_t checksum;
    float value[3];
} LOG_DATA_COMM;

#define PID_GPS_LATITUDE 0xA
#define PID_GPS_LONGITUDE 0xB
#define PID_GPS_ALTITUDE 0xC
#define PID_GPS_SPEED 0xD
#define PID_GPS_HEADING 0xE
#define PID_GPS_SAT_COUNT 0xF
#define PID_GPS_TIME 0x10
#define PID_GPS_DATE 0x11

#define PID_ACC 0x20
#define PID_GYRO 0x21
#define PID_COMPASS 0x22
#define PID_MEMS_TEMP 0x23
#define PID_BATTERY_VOLTAGE 0x24

#define FILE_NAME_FORMAT "/DAT%05d.CSV"

#if ENABLE_DATA_OUT

#if USE_SOFTSERIAL
SoftwareSerial SerialRF(A2, A3);
#else
#define SerialRF Serial
#endif

#endif

#if ENABLE_DATA_LOG
static File sdfile;
#endif

static const char* idstr = "FREEMATICS_V3\r";

class CDataLogger {
public:
    void initSender()
    {
#if ENABLE_DATA_OUT
        SerialRF.begin(STREAM_BAUDRATE);
        SerialRF.print(idstr);
#endif
    }
    void recordData(const char* buf)
    {
#if ENABLE_DATA_LOG
        dataSize += sdfile.print(dataTime - m_lastDataTime);
        sdfile.write(',');
        dataSize += sdfile.print(buf);
        dataSize ++;
        m_lastDataTime = dataTime;
#endif
    }
    void logData(char c)
    {
#if ENABLE_DATA_LOG
        if (c >= ' ') {
            sdfile.write(c);
            dataSize++;
        }
#elif STREAM_FORMAT == FORMAT_CSV
        SerialRF.write(c);
#endif
    }
    void logData(uint16_t pid, int value)
    {
        char buf[16];
        sprintf(buf, "%X,%d\r", pid, value);
#if ENABLE_DATA_OUT
#if STREAM_FORMAT == FORMAT_BIN
        LOG_DATA_COMM ld = {dataTime, pid, 1, 0, value};
        ld.checksum = getChecksum((char*)&ld, 12);
        SerialRF.write((uint8_t*)&ld, 12);
        delay(10);
#else
        SerialRF.print(buf);
#endif
#endif
        recordData(buf);
    }
    void logData(uint16_t pid, int32_t value)
    {
        char buf[20];
        sprintf(buf, "%X,%ld\r", pid, value);
#if ENABLE_DATA_OUT
#if STREAM_FORMAT == FORMAT_BIN
        LOG_DATA_COMM ld = {dataTime, pid, 1, 0, value};
        ld.checksum = getChecksum((char*)&ld, 12);
        SerialRF.write((uint8_t*)&ld, 12);
        delay(10);
#else
        SerialRF.print(buf);
#endif
#endif
        recordData(buf);
    }
    void logData(uint16_t pid, uint32_t value)
    {
        char buf[20];
        sprintf(buf, "%X,%lu\r", pid, value);
#if ENABLE_DATA_OUT
#if STREAM_FORMAT == FORMAT_BIN
        LOG_DATA_COMM ld = {dataTime, pid, 1, 0, value};
        ld.checksum = getChecksum((char*)&ld, 12);
        SerialRF.write((uint8_t*)&ld, 12);
        delay(10);
#else
        SerialRF.print(buf);
#endif
#endif
        recordData(buf);
    }
    void logData(uint16_t pid, int value1, int value2, int value3)
    {
        char buf[24];
        sprintf(buf, "%X,%d,%d,%d\r", pid, value1, value2, value3);
#if ENABLE_DATA_OUT
#if STREAM_FORMAT == FORMAT_BIN
        LOG_DATA_COMM ld = {dataTime, pid, 3, 0, {value1, value2, value3}};
        ld.checksum = getChecksum((char*)&ld, 20);
        SerialRF.write((uint8_t*)&ld, 20);
        delay(10);
#else
        SerialRF.print(buf);
#endif
#endif
        recordData(buf);
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
        m_lastDataTime = dateTime;
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
