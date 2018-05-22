/******************************************************************************
* Freematics Hub Server
* Developed by Stanley Huang <stanley@freematics.com.au>
* Distributed under GPL v3.0 license
* Visit https://freematics.com/hub for more information
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
******************************************************************************/

#ifndef MAX_CHANNELS
#define MAX_CHANNELS 4
#endif
#define MAX_CHANNEL_AGE (60* 60 * 1000 * 72)
#define MAX_PENDING_COMMANDS 4
#define MAX_COMMAND_MSG_LEN 128
#define SYNC_INTERVAL 15 /* seconds*/
#define CHANNEL_TIMEOUT 180 /* seconds */
#define FLAG_RUNNING 1

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
#define PID_TRIP_DISTANCE 0x30
#define PID_CSQ 0x81
#define PID_DEVICE_TEMP 0x82

#define NEW_TRIP_INTERVAL 300 /* secs */
#define PID_MODES 2
#define CACHE_INIT_SIZE (1024 * 1024)
#define CACHE_MAX_SIZE (10 * 1024 * 1024)
#define MAX_PID_DATA_LEN 24
#define MIN_LOGIN_INTERVAL 30000

#define EVENT_LOGIN 1
#define EVENT_LOGOUT 2
#define EVENT_SYNC 3
#define EVENT_RECONNECT 4
#define EVENT_COMMAND 5
#define EVENT_ACK 6

typedef enum {
	DEVICE_VEHICLE = 0,
	DEVICE_GPS,
	DEVICE_SWITCH
} DEVICE_TYPE;

typedef struct {
	uint32_t ts;
	char data[MAX_PID_DATA_LEN];
} PID_DATA;

typedef struct {
	uint32_t ts;
	uint16_t pid;
	uint8_t len;
	uint8_t reserved;
	char data[MAX_PID_DATA_LEN];
} CACHE_DATA;

#define CMD_FLAG_RESPONDED 1
#define CMD_FLAG_CHECKED 2

typedef struct {
	uint32_t token;
	uint64_t tick;
	char* message;
	uint16_t elapsed;
	uint8_t len;
	uint8_t flags;
} COMMAND_BLOCK;

typedef struct {
	uint32_t id; /* device ID */
	uint64_t serverTick;
	uint32_t deviceTick;
	uint32_t syncTick; /* versus device tick */
	uint32_t flags;
	// instant data
	PID_DATA mode[PID_MODES][256];
	// cache
	CACHE_DATA* cache;
	uint32_t cacheSize;
	uint32_t cacheReadPos;
	uint32_t cacheWritePos;
	// command
	COMMAND_BLOCK cmd[MAX_PENDING_COMMANDS];
	uint32_t cmdCount;
	// stats
	uint64_t startServerTick;
	uint32_t recvCount;
	uint32_t dataInterval;
	uint32_t dataReceived; /* bytes */
	uint32_t elapsedTime; /* seconds */
	uint16_t csq;
	uint8_t topSpeed; /* km/h */
	uint8_t deviceTemp;
	uint32_t distance;
	uint16_t dataRate;
	uint8_t dtcCount;
	uint8_t deviceType;
	char vin[32];
	IPADDR ip;
	// authorized UDP source address
	struct sockaddr_in udpPeer;
	// handles
	FILE* fp;
} CHANNEL_DATA;

CHANNEL_DATA* findChannelByID(uint32_t id);
CHANNEL_DATA* findEmptyChannel();
CHANNEL_DATA* findChannelByID(uint32_t id);
CHANNEL_DATA* findChannelByVin(const char* vin);
void SaveChannels();
FILE* getLogFile();
uint8_t hex2uint8(const char *p);
int hex2uint16(const char *p);
int checkVIN(const char* vin);
int processPayload(char* payload, CHANNEL_DATA* pld);
uint32_t issueCommand(HttpParam* hp, CHANNEL_DATA *pld, const char* cmd, uint32_t token);
int incomingUDPCallback(void* _hp);
