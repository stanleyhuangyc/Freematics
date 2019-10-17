class ClientSerial
{
public:
    bool begin(CFreematics* device) { return true; }
    void end() {}
    bool open(const char* host, uint16_t port) { return true; }
    void close() {}
    bool send(const char* data, unsigned int len)
    {
        Serial.write((uint8_t*)data, len);
        Serial.println();
        return true;
    }
    char* receive(int* pbytes = 0, unsigned int timeout = 5000)
    {
        Serial.setTimeout(timeout);
        int bytes = Serial.readBytes((uint8_t*)m_buffer, sizeof(m_buffer) - 1);
        if (pbytes) *pbytes = bytes;
        return bytes > 0 ? m_buffer : 0;
    }
    const char* deviceName() { return "Serial"; }
private:
    char m_buffer[128] = {0};
};

#define MESH_RECV_BUF_SIZE 256

class ClientWiFiMesh
{
public:
    ClientWiFiMesh();
    ~ClientWiFiMesh();
    bool begin(CFreematics* device);
    void end() {}
    bool open(const char* host, uint16_t port);
    void close() {}
    bool send(const char* data, unsigned int len);
    char* receive(int* pbytes = 0, unsigned int timeout = 5000);
    const char* deviceName() { return "WiFi Mesh"; }
private:
    char* m_buffer;
    bool m_inited = false;
};
