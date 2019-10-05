class WiFiMeshNode
{
public:
    bool begin(int channel, const char* id);
    bool send(char *data, size_t len);
    size_t receive(char *buffer, size_t size, int timeout = 5000);
};

class WiFiMeshRoot
{
public:
    bool begin(int channel, const char* id);
    bool send(char *data, size_t len);
    size_t receive(char *buffer, size_t size, int timeout = 5000);
private:
    uint8_t src_addr[6] = {0x0};
};
