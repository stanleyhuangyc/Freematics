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
#define MAX_CHANNELS 16
#endif

#define META_REVISION 1

#define MAX_CHANNEL_AGE (60* 60 * 1000 * 72)
#define MAX_PENDING_COMMANDS 4
#define MAX_COMMAND_MSG_LEN 128
#define SYNC_INTERVAL 30 /* seconds*/
#define CHANNEL_TIMEOUT 180 /* seconds */
#define SESSION_GAP (15 * 60 * 1000)
#define MIN_DEVID_LEN 6
#define MAX_DEVID_LEN 64
#define PID_MODES 2

#define FLAG_RUNNING 0x1
#define FLAG_SLEEPING 0x2
#define FLAG_PINGED 0x4

#define CACHE_INIT_SIZE (1024 * 1024)
#define CACHE_MAX_SIZE (10 * 1024 * 1024)
#define MAX_PID_DATA_LEN 24
#define MIN_LOGIN_INTERVAL 30000
#define PROXY_MAX_TIME_BEHIND 1000

#define EVENT_LOGIN 1
#define EVENT_LOGOUT 2
#define EVENT_SYNC 3
#define EVENT_RECONNECT 4
#define EVENT_COMMAND 5
#define EVENT_ACK 6
#define EVENT_PING 7

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
	uint64_t serverDataTick;
	uint64_t serverPingTick;
	uint64_t serverSyncTick;
	uint64_t sessionStartTick;
	uint32_t deviceTick;
	uint16_t flags;
	uint16_t devflags;
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
	// proxy
	uint32_t proxyTick;
	// stats
	uint32_t recvCount;
	uint32_t txCount;
	uint32_t dataReceived; /* bytes */
	uint32_t elapsedTime; /* seconds */
	uint16_t csq;
	uint8_t unused;
	uint8_t deviceTemp;
	float sampleRate;
	char vin[20];
	char devid[32];
	IPADDR ip;
	// authorized UDP source address
	struct sockaddr_in udpPeer;
	// handles
	FILE* fp;
} CHANNEL_DATA;

CHANNEL_DATA* findEmptyChannel();
CHANNEL_DATA* findChannelByID(uint32_t id);
CHANNEL_DATA* findChannelByDeviceID(const char* devid);
void SaveChannels();
FILE* getLogFile();
uint8_t hex2uint8(const char *p);
int hex2uint16(const char *p);
int checkVIN(const char* vin);
int processPayload(char* payload, CHANNEL_DATA* pld, int store);
uint32_t issueCommand(HttpParam* hp, CHANNEL_DATA *pld, const char* cmd, uint32_t token);
int incomingUDPCallback(void* _hp);
void deviceLogin(CHANNEL_DATA* pld);
void deviceLogout(CHANNEL_DATA* pld);