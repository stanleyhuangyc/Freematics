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

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <stdint.h>
#include <ctype.h>
#include <errno.h>
#include <sys/stat.h>
#include "data2kml.h"
#include "httpd.h"
#include "teleserver.h"
#include "logdata.h"
#include "processpil.h"
#include "revision.h"

int uhPush(UrlHandlerParam* param);
int uhPull(UrlHandlerParam* param);
int uhGet(UrlHandlerParam* param);
int uhPost(UrlHandlerParam* param);
int uhChannels(UrlHandlerParam* param);
int uhChannelsXML(UrlHandlerParam* param);
int uhNotify(UrlHandlerParam* param);
int uhCommand(UrlHandlerParam* param);
int uhTest(UrlHandlerParam* param);

int uhTrip(UrlHandlerParam* param);
int uhHistory(UrlHandlerParam* param);
int uhData(UrlHandlerParam* param);
int uhQuery(UrlHandlerParam* param);
int phData(void* _hp, int op, char* buf, int len);

UrlHandler urlHandlerList[]={
	{"api/post", uhPost},
	{"api/push", uhPush},
	{"api/get", uhGet},
	{"api/pull", uhPull},
	{"api/notify", uhNotify },
	{"api/command", uhCommand },
	{"api/channels.xml", uhChannelsXML },
	{"api/channels", uhChannels},
	{"api/query", uhQuery},
	{"api/data", uhData},
	{"api/trip", uhTrip },
	{"api/history", uhHistory },
	{"api/test", uhTest},
	{NULL},
};

int loadConfig();

char username[64] = "admin";
char password[64] = { 0 };

AuthHandler authHandlerList[]={
	{ "", username, password },
	{NULL}
};

HttpParam httpParam;

char dataDir[256] = "data";
char logDir[256] = "log";
char serverKey[256] = { 0 };
int noGUI = 0;

CHANNEL_DATA ld[MAX_CHANNELS];

uint8_t hex2uint8(const char *p)
{
	uint8_t c1 = *p;
	uint8_t c2 = *(p + 1);
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

int hex2uint16(const char *p)
{
	char c = *p;
	uint16_t i = 0;
	char n;
	for (n = 0; c && n < 4; c = *(++p)) {
		if (c >= 'A' && c <= 'F') {
			c -= 7;
		}
		else if (c >= 'a' && c <= 'f') {
			c -= 39;
		}
		else if (c == ' ') {
			continue;
		}
		else if (c == '#' || c == '=' || c == ',' || c == ';' || c == ':') {
			return i;
		}
		else if (c < '0' || c > '9') {
			return -1;
		}
		i = (i << 4) | (c & 0xF);
		n++;
	}
	return i;
}

int ishex(char c)
{
	return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
}

int isnum(const char* s)
{
	do {
		if (!isdigit(*s)) return FALSE;;
	} while (*(++s));
	return TRUE;
}

int checkVIN(const char* vin)
{
	if (!vin || strlen(vin) != 17) return 0;
	int n = 0;
	for (const char *p = vin; *p; p++, n++) {
		if (!((*p >= 'A' && *p <= 'Z') || (*p >= 'a' && *p <= 'z') || (*p >= '0' && *p <= '9') || *p == '-' || *p == '_' || (*p == ' ' && *(p + 1) == ' '))) {
			return 0;
		}
	}
	return n >= 8;
}

CHANNEL_DATA* findChannelByID(uint32_t id)
{
	if (id) {
		int i;
		for (i = 0; i < MAX_CHANNELS; i++) {
			if (ld[i].id == id) {
				//printf("Channel found (ID:%u)\n", id);
				return ld + i;
			}
		}
	}
	printf("Channel not found (ID:%u)\n", id);
	return 0;
}

CHANNEL_DATA* findChannelByDeviceID(const char* devid)
{
	if (devid && *devid) {
		int i;
		for (i = 0; i < MAX_CHANNELS; i++) {
			if (ld[i].id && !strcmp(ld[i].devid, devid)) {
				return ld + i;
			}
		}
	}
	return 0;
}

void initChannel(CHANNEL_DATA* pld, int cacheSize)
{
	pld->cacheSize = min(cacheSize, CACHE_MAX_SIZE);
	pld->cache = calloc(cacheSize, sizeof(CACHE_DATA));
	pld->cacheReadPos = 0;
	pld->cacheWritePos = 0;
	pld->recvCount = 0;
	pld->txCount = 0;
	pld->dataReceived = 0;
	pld->proxyTick = 0;
	memset(pld->cmd, 0, sizeof(pld->cmd));
}

CHANNEL_DATA* findEmptyChannel()
{
	unsigned int id = 1;
	int index = -1;
	int i;
	for (i = 0; i < MAX_CHANNELS; i++) {
		if (ld[i].id >= id) {
			id = ld[i].id + 1;
		}
	}
	for (i = 0; i < MAX_CHANNELS; i++) {
		if (!ld[i].id) {
			// empty channel found
			index = i;
			break;
		}
	}
	if (index == -1) {
		return 0;
	}
	// initialize channel
	memset(&ld[index], 0, sizeof(CHANNEL_DATA));
	ld[index].id = id;
	return ld + index;
}

void removeChannel(CHANNEL_DATA* pld)
{
	if (pld->cache) free(pld->cache);
	if (pld->fp) fclose(pld->fp);
	memset(pld, 0, sizeof(CHANNEL_DATA));
}

FILE* getLogFile()
{
	static uint32_t curDate = 0;
	static FILE *fpLog = 0;
	time_t t = time(NULL);
	struct tm *btm = gmtime(&t);
	uint32_t date = (btm->tm_year + 1900) * 10000 + (btm->tm_mon + 1) * 100 + btm->tm_mday;
	if (date != curDate) {
		char path[128];
		snprintf(path, sizeof(path), "%s/%u.txt", logDir, date);
		if (fpLog && fpLog != stderr) {
			fclose(fpLog);
		}
#ifndef _DEBUG
		fpLog = fopen(path, "a+");
#endif
		if (!fpLog) fpLog = stderr;
		curDate = date;
	}
	fprintf(fpLog, "[%02u:%02u:%02u]", btm->tm_hour, btm->tm_min, btm->tm_sec);
	return fpLog;
}

FILE* createDataFile(CHANNEL_DATA* pld)
{
	if (!pld) return NULL;

	if (pld->fp) fclose(pld->fp);

	// Create data directory if it doesn't exist yet, print error message on failure
	if (!IsDir(dataDir) && mkdir(dataDir, 0755) < 0) {
		char *errstr = strerror(errno);
		fprintf(getLogFile(), "Can't create data directory '%s': %s\n", dataDir, errstr);
	}

	time_t t = time(NULL);
	struct tm *btm = gmtime(&t);
	char filename[256];
	int n;
	n = snprintf(filename, sizeof(filename), "%s/%s", dataDir, pld->devid);
	if (!IsDir(filename)) {
		fprintf(getLogFile(), "New device:%s\n", pld->devid);
		mkdir(filename, 0755);
	}
	n += snprintf(filename + n, sizeof(filename) - n, "/%04u", btm->tm_year + 1900);
	mkdir(filename, 0755);
	n += snprintf(filename + n, sizeof(filename) - n, "/%02u", btm->tm_mon + 1);
	mkdir(filename, 0755);
	n += snprintf(filename + n, sizeof(filename) - n, "/%02u", btm->tm_mday);
	mkdir(filename, 0755);
	n += snprintf(filename + n, sizeof(filename) - n, "/%04u%02u%02u-%02u%02u%02u.txt",
		btm->tm_year + 1900,
		btm->tm_mon + 1,
		btm->tm_mday,
		btm->tm_hour,
		btm->tm_min,
		btm->tm_sec);
	pld->fp = fopen(filename, "a+");
	if (!pld->fp) return 0;
	if (ftell(pld->fp) == 0) {
		// write initial data
		if (pld->data[PID_GPS_LATITUDE].ts && pld->data[PID_GPS_LONGITUDE].ts) {
			fprintf(pld->fp, "%X:%s,%X:%s,%X:%s\n",
				PID_GPS_LATITUDE, pld->data[PID_GPS_LATITUDE].value,
				PID_GPS_LONGITUDE, pld->data[PID_GPS_LONGITUDE].value,
				PID_GPS_ALTITUDE, pld->data[PID_GPS_ALTITUDE].value);
		}
	}
	return pld->fp;
}

void deviceLogin(CHANNEL_DATA* pld)
{
	pld->flags |= FLAG_RUNNING;
	pld->flags &= ~FLAG_SLEEPING;
	pld->proxyTick = 0;
	// clear stats
	pld->dataReceived = 0;
	pld->recvCount = 0;
	pld->txCount = 0;
	pld->elapsedTime = 0;
	SaveChannels();
	createDataFile(pld);
	fprintf(getLogFile(), " LOGIN:%s\n", pld->devid);
}

void deviceLogout(CHANNEL_DATA* pld)
{
	uint64_t serverTick = GetTickCount64();
	pld->flags &= ~FLAG_RUNNING;
	pld->serverPingTick = serverTick;
	if (pld->fp) {
		fclose(pld->fp);
		pld->fp = 0;
	}
	fprintf(getLogFile(), " LOGOUT:%s\n", pld->devid);
}

int processPayload(char* payload, CHANNEL_DATA* pld, uint16_t eventID)
{
	uint64_t tick = GetTickCount64();
	if (eventID == 0) {
		if (!pld->fp && (pld->flags & FLAG_RUNNING)) {
			createDataFile(pld);
		}
		// save data to log file
		if (pld->fp) {
			fprintf(pld->fp, "%s\n", payload);
		}
	}

	char *p = payload;
	uint32_t ts = 0;
	int count = 0;
	do {
		int pid = hex2uint16(p);
		if (pid == -1) {
			p = strchr(p, ',');
			if (p) *(p++) = 0;
			continue;
		}
		while (ishex(*p)) p++;
		if (*p != ':' && *p != '=') break;
		char *value = ++p;
		p = strchr(p, ',');
		if (p) *(p++) = 0;
		size_t len = strlen(value);
		if (len >= MAX_PID_DATA_LEN) len = MAX_PID_DATA_LEN - 1;
		// now we have pid and value
		if (pid == 0) {
			// special PID 0 for timestamp
			ts = atol(value);
			continue;
		}
		if (ts == 0) {
			// no valid timestamp yet
			continue;
		}
		// store in table
		int m = pid >> 8;
		if (m < PID_MODES) {
			pld->data[pid].ts = ts;
			memcpy(pld->data[pid].value, value, len + 1);
			// collect some stats
			switch (pid) {
			case PID_RSSI: /* signal strength */
				pld->rssi = atoi(value);
				break;
			case PID_DEVICE_TEMP:
				pld->deviceTemp = atoi(value);
				break;
			}
		}
		count++;
		// store in cache
		if (pld->cacheReadPos != pld->cacheWritePos && pld->cache[pld->cacheReadPos].ts > ts) {
			// clear cache as data looks staled
			pld->cacheReadPos = 0;
			pld->cacheWritePos = 0;
		}
		CACHE_DATA *d = &pld->cache[pld->cacheWritePos];
		d->ts = ts;
		d->pid = pid;
		d->len = (uint8_t)len;
		memcpy(d->data, value, len);
		d->data[len] = 0;
		// adjust cache pointers
		pld->cacheWritePos = (pld->cacheWritePos + 1) % pld->cacheSize;
		if (pld->cacheWritePos == pld->cacheReadPos) {
			// if write pos catch up with read pos (one lap ahead)
			// move forward read pos to discard just overwrited data
			pld->cacheReadPos = (pld->cacheReadPos + 1) % pld->cacheSize;
		}
	} while (p && *p);
	if (ts == 0) ts = pld->deviceTick;
	int interval = ts - pld->deviceTick;
	pld->deviceTick = ts;

	if (pld->flags & FLAG_RUNNING) {
		// normal
		if (interval > 100) pld->sampleRate = (float)count * 60000 / interval;
		pld->elapsedTime = (uint32_t)((tick - pld->sessionStartTick) / 1000);
		pld->flags &= ~FLAG_SLEEPING;
	}
	else if (eventID == 0) {
		// this happens when login packet not received
		deviceLogin(pld);
		pld->sessionStartTick = pld->serverDataTick;
		pld->elapsedTime = (uint32_t)((tick - pld->sessionStartTick) / 1000);
	}
	if (!(pld->flags & FLAG_SLEEPING)) {
		pld->serverDataTick = tick;
	}

	pld->recvCount++;

	printf("[%u] #%u %u bytes | Samples:%u | Device Tick:%u\n", pld->id, pld->recvCount, pld->dataReceived, count, pld->deviceTick);
	return count;
}

void __inline setPIDData(CHANNEL_DATA* pld, int pid, uint32_t ts, const char* value)
{
	pld->data[pid].ts = ts;
	strncpy(pld->data[pid].value, value, MAX_PID_DATA_LEN - 1);
}

void SaveChannels()
{
	char path[256];
	snprintf(path, sizeof(path), "%s/channels.dat", dataDir);
	FILE *fp = fopen(path, "wb");
	if (!fp) return;
	printf("Saving channels");
	for (int i = 0; i < MAX_CHANNELS; i++) {
		if (ld[i].id) printf(" [%d]", ld[i].id);
	}
	printf("...");
	fwrite(ld, MAX_CHANNELS, sizeof(CHANNEL_DATA), fp);
	fclose(fp);
	printf("OK\n");
}

int LoadChannels()
{
	char path[256];
	snprintf(path, sizeof(path), "%s/channels.dat", dataDir);
	FILE *fp = fopen(path, "rb");
	if (!fp) return 0;

	fseek(fp, 0, SEEK_END);
	unsigned int len = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	if (len == MAX_CHANNELS * sizeof(CHANNEL_DATA)) {
		fread(ld, MAX_CHANNELS, sizeof(CHANNEL_DATA), fp);
	}
	else {
		fprintf(stderr, "Channel data file size mismatch (expected %u, actual %u)\n", (unsigned int)(MAX_CHANNELS * sizeof(CHANNEL_DATA)), len);
	}
	fclose(fp);
	int count = 0;
	for (int i = 0; i < MAX_CHANNELS; i++) {
		int valid = 1;
		for (char* p = ld[i].devid; *p; p++) if (!isalpha(*p) && !isdigit(*p)) valid = 0;
		if (ld[i].id && valid) {
			printf("[%u] ID:%u DEVID:%s\n", i, ld[i].id, ld[i].devid);
			ld[i].fp = 0; /* file handle no longer valid*/
			initChannel(&ld[i], ld[i].cacheSize);
			count++;
		}
		else {
			memset(ld + i, 0, sizeof(CHANNEL_DATA));
		}
	}
	printf("%d channels loaded\n", count);
	return count;
}

void CheckChannels()
{
	uint64_t tick = GetTickCount64();
	for (int i = 0; i < MAX_CHANNELS; i++) {
		if (!ld[i].id) continue;
		CHANNEL_DATA* pld = ld + i;
		if (pld->flags & FLAG_RUNNING) {
			if (tick - pld->serverDataTick > CHANNEL_TIMEOUT * 1000) {
				pld->flags &= ~FLAG_RUNNING;
			}
		}
	}
}

void showLiveData(CHANNEL_DATA* pld)
{
	int i = 0;
	printf("[DEVID]%s\n", pld->devid);
	printf("[OBD]");
	for (i = 0x100; i < 0x100 * PID_MODES; i++) {
		if (pld->data[i].ts) {
			printf("%4X=%s ", i, pld->data[i].value);
		}
	}
	printf("\n");
	if (pld->data[PID_GPS_TIME].ts) {
		printf("[GPS]UTC:%s LAT:%s LNG:%s ALT:%sm Speed:%skm/h Sat:%s\n",
			pld->data[PID_GPS_TIME].value, pld->data[PID_GPS_LATITUDE].value, pld->data[PID_GPS_LONGITUDE].value,
			pld->data[PID_GPS_ALTITUDE].value, pld->data[PID_GPS_SPEED].value, pld->data[PID_GPS_SAT_COUNT].value);
	}
	printf("\n");
}

static int copyData(char* d, const char* s)
{
	BOOL isNum = TRUE;
	BOOL isArray = FALSE;
	const char *p;
	for (p = s; *p; p++) {
		if (*p == ';') {
			isArray = TRUE;
		} else if (!isdigit(*p) && *p != '-' && *p != '.') {
			isNum = FALSE;
		}
	}
	int len = (int)(p - s);
	if (!isNum) *(d++) = '\"';
	if (isNum && isArray) {
		*(d++) = '[';
		memcpy(d, s, len);
		for (int i = 0; i < len; i++) {
			if (d[i] == ';') d[i] = ',';
		}
		d += len;
		*(d++) = ']';
		len += 2;
	}
	else {
		memcpy(d, s, len);
		d += len;
	}
	if (!isNum) *(d++) = '\"';
	*d = 0;
	return isNum ? len : len + 2;
}

CHANNEL_DATA* locateChannel(UrlHandlerParam* param)
{
	const char* sid;
	if (param->pucRequest[0] == '/') {
		sid = param->pucRequest + 1;
	}
	else {
		sid = mwGetVarValue(param->pxVars, "id", "");
	}

	CHANNEL_DATA *pld = 0;
	if (*sid) {
		pld = findChannelByDeviceID(sid);
	}
	return pld;
}

int uhChannelsXML(UrlHandlerParam* param)
{
	int extend = mwGetVarValueInt(param->pxVars, "extend", 0);
	uint64_t tick = GetTickCount64();
	/*
	fprintf(getLogFile(), "%u.%u.%u.%u request channels XML\n",
		param->hs->ipAddr.caddr[3], param->hs->ipAddr.caddr[2], param->hs->ipAddr.caddr[1], param->hs->ipAddr.caddr[0]);
	*/

	char *p = param->pucBuffer;
	p += sprintf(p, "<?xml version=\"1.0\" encoding=\"utf-8\"?><channels>\n");
	for (int n = 0; n < MAX_CHANNELS; n++) {
		CHANNEL_DATA* pld = ld + n;
		if (pld->id) {
			p += sprintf(p, "<channel id=\"%u\" devid=\"%s\" recv=\"%u\" rate=\"%u\" tick=\"%u\" elapsed=\"%u\" age=\"%u\" parked=\"%u\" rssi=\"%d\" flags=\"%u\"",
				pld->id, pld->devid, pld->dataReceived, (unsigned int)pld->sampleRate, pld->deviceTick, pld->elapsedTime, 
				(int)(tick - pld->serverDataTick), (pld->flags & FLAG_RUNNING) ? 0 : 1, (int)pld->rssi, pld->devflags);

			if (extend) {
				if (*pld->vin) p += sprintf(p, "<vin>%s</vin>", pld->vin);
				p += sprintf(p, "><cache size=\"%u\" read=\"%u\" write=\"%u\"/></channel>\n",
					pld->cacheSize, pld->cacheReadPos, pld->cacheWritePos);
				if (pld->ip.laddr) {
					p += sprintf(p, "<ip>%u.%u.%u.%u</ip>", pld->ip.caddr[3], pld->ip.caddr[2], pld->ip.caddr[1], pld->ip.caddr[0]);
				}
				else {
					p += sprintf(p, "<ip>%s</ip>", inet_ntoa(pld->udpPeer.sin_addr));
				}
			}
			else {
				p += sprintf(p, "/>\n");
			}

		}
		else {
			p += sprintf(p, "<channel/>\n");
		}
	}
	p += sprintf(p, "</channels>");
	param->contentLength = (int)(p - param->pucBuffer);
	param->contentType = HTTPFILETYPE_XML;
	return FLAG_DATA_RAW;
}

int uhChannels(UrlHandlerParam* param)
{
	uint64_t tick = GetTickCount64();
	int bs = param->bufSize;
	char* buf = param->pucBuffer;
	int l = 0;
	int n = 0;
	const char *cmd = mwGetVarValue(param->pxVars, "cmd", 0);
	int data = mwGetVarValueInt(param->pxVars, "data", 0);
	int extend = mwGetVarValueInt(param->pxVars, "extend", 0);
	int id = mwGetVarValueInt(param->pxVars, "id", 0);
	unsigned int refresh = mwGetVarValueInt(param->pxVars, "refresh", MAX_CHANNEL_AGE);
	const char *devid = mwGetVarValue(param->pxVars, "devid", 0);

	const char *req = param->pucRequest;
	if (!strncmp(req, "/data", 5)) {
		data = 1;
		req += 5;
	}
	if (req[0] == '/') {
		devid = req + 1;
	}
	/*
	fprintf(getLogFile(), "%u.%u.%u.%u request channels\n",
		param->hs->ipAddr.caddr[3], param->hs->ipAddr.caddr[2], param->hs->ipAddr.caddr[1], param->hs->ipAddr.caddr[0]);
	*/

	if (cmd && !strcmp(cmd, "clear")) {
		CHANNEL_DATA *pld = findChannelByID(id);
		if (pld) {
			fprintf(getLogFile(), "%u.%u.%u.%u [%u] remove channel\n",
				param->hs->ipAddr.caddr[3], param->hs->ipAddr.caddr[2], param->hs->ipAddr.caddr[1], param->hs->ipAddr.caddr[0], id);
			removeChannel(pld);
			id = 0;
		}
	}
	if (!devid) {
		l += snprintf(buf + l, bs - l, "{\"channels\":[");
	}
	for (n = 0; n < MAX_CHANNELS; n++) {
		CHANNEL_DATA* pld = ld + n;
		if (!pld->id) continue;
		if (devid && strcmp(pld->devid, devid)) continue;
		if (id == 0 || pld->id == id) {
			unsigned int age = (unsigned int)(tick - pld->serverDataTick);
			unsigned int pingage = (unsigned int)(tick - pld->serverPingTick);
			if (refresh && (age > refresh && pingage > refresh)) {
				removeChannel(pld);
				continue;
			}
			l += snprintf(buf + l, bs - l, "\n{\"id\":\"%u\",\"devid\":\"%s\",\"recv\":%u,\"rate\":%u,\"tick\":%llu,\"devtick\":%u,\"elapsed\":%u,\"age\":{\"data\":%u,\"ping\":%u},\"rssi\":%d,\"flags\":%u,\"parked\":%u",
				pld->id, pld->devid, pld->dataReceived, (unsigned int)pld->sampleRate, pld->serverDataTick, pld->deviceTick, pld->elapsedTime,
				age, pingage, (int)pld->rssi, pld->devflags, (pld->flags & FLAG_RUNNING) ? 0 : 1);

			if (extend) {
				if (*pld->vin) {
					l += snprintf(buf + l, bs - l, ",\"vin\":\"%s\"", pld->vin);
				}
				if (pld->ip.laddr) {
					l += snprintf(buf + l, bs - l, ",\"ip\":\"%u.%u.%u.%u\"", pld->ip.caddr[3], pld->ip.caddr[2], pld->ip.caddr[1], pld->ip.caddr[0]);
				}
				else {
					l += snprintf(buf + l, bs - l, ",\"ip\":\"%s\"", inet_ntoa(pld->udpPeer.sin_addr));
				}
			}

			if (data) {
				l += snprintf(buf + l, bs - l, ",\"data\":[");
				for (unsigned int i = 0; i < 0x100 * PID_MODES; i++) {
					if (pld->data[i].ts) {
						l += snprintf(buf + l, bs - l, "[%u,", i);
						l += copyData(buf + l, pld->data[i].value);
						l += snprintf(buf + l, bs - l, ",%u],", age + (pld->deviceTick - pld->data[i].ts));
					}
				}
				if (buf[l - 1] == ',') l--;
				l += snprintf(buf + l, bs - l, "]");
			}
			l += snprintf(buf + l, bs - l, "},");
		}
	}

	if (l == 0) {
		l += snprintf(buf + l, bs - l, "{}");
	}
	else if (buf[l - 1] == ',') {
		buf[--l] = 0;
	}
	if (!devid) {
		l += snprintf(buf + l, bs - l, "]}");
	}
	param->contentLength = l;
	param->contentType = HTTPFILETYPE_JSON;
	return FLAG_DATA_RAW;
}

char* findNextToken(char* s)
{
	while (*s && (isdigit(*s) || *s == '-' || *s == '.' || *s == ',' || *s == '/' || *s == ';')) s++;
	return s + 1;
}

CHANNEL_DATA* assignChannel(const char* devid)
{
	if (!devid || strlen(devid) < 4) {
		fprintf(getLogFile(), "Invalid ID");
		return 0;
	}
	// check invalid character in devid string
	for (const char* p = devid; *p; p++) if (!isalpha(*p) && !isdigit(*p)) return 0;

	CHANNEL_DATA *pld = findChannelByDeviceID(devid);
	if (pld) {
		return pld;
	}
	pld = findEmptyChannel();
	if (!pld) {
		return 0;
	}
	strncpy(pld->devid, devid, sizeof(pld->devid) - 1);
	initChannel(pld, CACHE_INIT_SIZE);

	// clear history data cache
	pld->cacheReadPos = 0;
	pld->cacheWritePos = 0;
	// clear instance data cache
	memset(pld->data, 0, sizeof(pld->data));
	// clear stats
	pld->dataReceived = 0;
	pld->elapsedTime = 0;
	pld->serverDataTick = GetTickCount64();
	SaveChannels();
	printf("DEVID:%s ID:%u\r\n", devid, pld->id);
	return pld;
}

int uhPost(UrlHandlerParam* param)
{
	param->contentLength = 0;
	CHANNEL_DATA* pld = locateChannel(param);
	if (!pld) {
		param->hs->response.statusCode = 403;
		param->contentLength = 0;
		return FLAG_DATA_RAW;
	}

	const char* lat = mwGetVarValue(param->pxVars, "lat", 0);
	const char* lon = mwGetVarValue(param->pxVars, "lon", 0);
	uint32_t ts = mwGetVarValueInt(param->pxVars, "timestamp", 0);
	const char* alt = mwGetVarValue(param->pxVars, "altitude", 0);
	const char* speed = mwGetVarValue(param->pxVars, "speed", 0);
	const char* heading = mwGetVarValue(param->pxVars, "heading", 0);
	pld->deviceTick = ts;
	if (lat) setPIDData(pld, PID_GPS_LATITUDE, ts, lat);
	if (lon) setPIDData(pld, PID_GPS_LONGITUDE, ts, lon);
	if (speed) setPIDData(pld, PID_GPS_SPEED, ts, speed);
	if (alt) setPIDData(pld, PID_GPS_ALTITUDE, ts, alt);
	if (heading) setPIDData(pld, PID_GPS_HEADING, ts, heading);

	if (!param->payloadSize) {
		printf("GET from %u.%u.%u.%u | LAT:%s LON:%s ALT:%sm\n",
			param->hs->ipAddr.caddr[3], param->hs->ipAddr.caddr[2], param->hs->ipAddr.caddr[1], param->hs->ipAddr.caddr[0],
			lat, lon, alt);
		return FLAG_DATA_RAW;
	}

	printf("POST from %u.%u.%u.%u | ",
		param->hs->ipAddr.caddr[3], param->hs->ipAddr.caddr[2], param->hs->ipAddr.caddr[1], param->hs->ipAddr.caddr[0]);

	unsigned int count = processPayload(param->pucPayload, pld, 0);
	pld->dataReceived += param->payloadSize;
	pld->ip = param->hs->ipAddr;

	param->contentLength = sprintf(param->pucBuffer, "OK %u", count);
	param->contentType = HTTPFILETYPE_TEXT;
	return FLAG_DATA_RAW;
}

int uhGet(UrlHandlerParam* param)
{
	CHANNEL_DATA* pld = locateChannel(param);
	if (!pld) {
		param->hs->response.statusCode = 403;
		param->contentLength = 0;
		return FLAG_DATA_RAW;
	}

	uint64_t tick = GetTickCount64();
	int bs = param->bufSize;
	char* buf = param->pucBuffer;
	int l = 0;
	unsigned int age = pld->serverDataTick ? (unsigned int)(tick - pld->serverDataTick) : 0;
	unsigned int pingage = pld->serverPingTick ? (unsigned int)(tick - pld->serverPingTick) : 0;
	l += snprintf(buf + l, bs - l, "{\"stats\":{\"tick\":%llu,\"devtick\":%u,\"elapsed\":%u,\"age\":{\"data\":%u,\"ping\":%u},\"rssi\":%d,\"flags\":%u,\"parked\":%u}",
		pld->serverDataTick, pld->deviceTick, pld->elapsedTime,
		age, pingage, (int)pld->rssi, pld->devflags, (pld->flags & FLAG_RUNNING) ? 0 : 1);

	l += snprintf(buf + l, bs - l, ",\"data\":[");
	for (unsigned int i = 0; i < 0x100 * PID_MODES; i++) {
		if (pld->data[i].ts) {
			l += snprintf(buf + l, bs - l, "[%u,", i);
			l += copyData(buf + l, pld->data[i].value);
			l += snprintf(buf + l, bs - l, ",%u],",
				pld->deviceTick >= pld->data[i].ts ? (age + pld->deviceTick - pld->data[i].ts) : 0);
		}
	}
	if (buf[l - 1] == ',') l--;
	l += snprintf(buf + l, bs - l, "]}");

	param->contentLength = l;
	param->contentType = HTTPFILETYPE_JSON;
	return FLAG_DATA_RAW;
}

int uhPull(UrlHandlerParam* param)
{
	param->contentType = HTTPFILETYPE_JSON;
	param->contentLength = 0;
	CHANNEL_DATA *pld = locateChannel(param);
	if (!pld) {
		param->hs->response.statusCode = 403;
		param->contentLength = 0;
		return FLAG_DATA_RAW;
	}

	uint64_t startts = mwGetVarValueInt64(param->pxVars, "ts");
	uint64_t endts = mwGetVarValueInt64(param->pxVars, "endts");
	uint32_t rollback = mwGetVarValueInt(param->pxVars, "rollback", 0);
	int pid = mwGetVarValueInt(param->pxVars, "pid", 0);

	uint64_t tick = GetTickCount64();
	unsigned int age = (unsigned int)(tick - pld->serverDataTick);
	unsigned int pingage = (unsigned int)(tick - pld->serverPingTick);

	int bytes = 0;
	char* buf = param->pucBuffer;
	int bufsize = param->bufSize;

	bytes += sprintf(buf + bytes, "{");
	bytes += snprintf(buf + bytes, bufsize - bytes, "\"stats\":{\"recv\":%u,\"rate\":%u,\"tick\":%llu,\"devtick\":%u,\"elapsed\":%u,\"age\":{\"data\":%u,\"ping\":%u},\"parked\":%u}",
		pld->dataReceived, (unsigned int)pld->sampleRate, pld->serverDataTick, pld->deviceTick, pld->elapsedTime, age, pingage, (pld->flags & FLAG_RUNNING) ? 0 : 1);

	bytes += snprintf(buf + bytes, bufsize - bytes, ",\"live\":[");
	for (unsigned int i = 0; i < 0x100 * PID_MODES; i++) {
		if (pld->data[i].ts) {
			bytes += snprintf(buf + bytes, bufsize - bytes, "[%u,", i);
			bytes += copyData(buf + bytes, pld->data[i].value);
			bytes += snprintf(buf + bytes, bufsize - bytes, "],");
		}
	}
	if (buf[bytes - 1] == ',') bytes--;
	bytes += snprintf(buf + bytes, bufsize - bytes, "]");

	if (rollback) {
		// calculate and override ts
		uint64_t t = GetTickCount64() - pld->serverDataTick + pld->deviceTick;
		startts = t > rollback ? (t - rollback) : 0;
	}
	// start of data array
	bytes += sprintf(buf + bytes, ",\"data\":[");
	uint32_t readPos = pld->cacheReadPos;
	uint64_t begin = 0;
	int bytesMargin = bytes;
	uint32_t lastts = 0;
	for (; readPos != pld->cacheWritePos; readPos = (readPos + 1) % pld->cacheSize) {
		CACHE_DATA *d = pld->cache + readPos;
		if (d->ts < lastts) {
			// timestamp looping or device reset detected, wipe out all previous data
			bytes = bytesMargin;
		}
		lastts = d->ts;
		if (d->ts >= startts) {
			if (endts && d->ts >= endts) break;
			if (bytes + d->len + 64 > bufsize) {
				// buffer full
				break;
			}
			if (d->data[0] && (pid == 0 || pid == d->pid)) {
				bytes += sprintf(buf + bytes, "[%u,%d,", d->ts, d->pid);
				bytes += copyData(buf + bytes, d->data);
				bytes += sprintf(buf + bytes, "],");
			}
			// keep ts range
			if (begin == 0) begin = d->ts;
		}
	}
	if (buf[bytes - 1] == ',') bytes--;
	// end of data array
	buf[bytes++] = ']';
	if (readPos == pld->cacheWritePos) {
		// cache completely read
		bytes += sprintf(buf + bytes, ",\"eos\":1");
	}
	else {
		bytes += sprintf(buf + bytes, ",\"eos\":0");
	}
	buf[bytes++] = '}';
	buf[bytes] = 0;
	param->contentLength = bytes;
	return FLAG_DATA_RAW;
}

int isNum(const char* s)
{
	while (*s) {
		if (!isdigit(*s)) return 0;
		s++;
	}
	return 1;
}

int uhCommand(UrlHandlerParam* param)
{
	CHANNEL_DATA *pld = locateChannel(param);
	if (!pld) {
		param->hs->response.statusCode = 403;
		param->contentLength = 0;
		return FLAG_DATA_RAW;
	}

	char* cmd = mwGetVarValue(param->pxVars, "cmd", "");
	uint32_t token = mwGetVarValueInt(param->pxVars, "token", 0);
	param->contentType = HTTPFILETYPE_JSON;

	if (!*cmd && !token) {
		param->contentLength = snprintf(param->pucBuffer, param->bufSize, "{\"result\":\"failed\",\"error\":\"Invalid request\"}");
		return FLAG_DATA_RAW;
	}
	pld->serverDataTick = GetTickCount64();
	if (*cmd) {
		// token = 0: no token
		token = issueCommand(param->hp, pld, cmd, token);
		if (token) {
			param->contentLength = snprintf(param->pucBuffer, param->bufSize, "{\"result\":\"pending\",\"token\":%u}", token);
		}
		else {
			param->contentLength = snprintf(param->pucBuffer, param->bufSize, "{\"result\":\"failed\",\"error\":\"Command unsent\"}");
		}
	}
	else {
		COMMAND_BLOCK* cb = 0;
		int i;
		for (i = 0; i < MAX_PENDING_COMMANDS; i++) {
			if (pld->cmd[i].token == token) {
				cb = pld->cmd + i;
				break;
			}
		}
		if (!cb) {
			param->contentLength = snprintf(param->pucBuffer, param->bufSize, "{\"result\":\"failed\",\"error\":\"Invalid token\"}");
		}
		else if (!(cb->flags & CMD_FLAG_RESPONDED)) {
			param->contentLength = snprintf(param->pucBuffer, param->bufSize, "{\"result\":\"pending\",\"elapsed\":%u}", (unsigned int)(cb->tick - pld->serverDataTick));
		}
		else {
			param->contentLength = snprintf(param->pucBuffer, param->bufSize, "{\"result\":\"done\",\"idx\":%u,\"elapsed\":%u,\"data\":\"%s\"}",
				i, (unsigned int)cb->elapsed, cb->message);
			cb->flags |= CMD_FLAG_CHECKED;
		}
	}
	return FLAG_DATA_RAW;
}

int uhNotify(UrlHandlerParam* param)
{
	char* vin = mwGetVarValue(param->pxVars, "VIN", 0);
	int event = mwGetVarValueInt(param->pxVars, "EV", 0);
	unsigned int devflags = mwGetVarValueInt(param->pxVars, "DF", 0);
	int rssi = mwGetVarValueInt(param->pxVars, "SSI", 0);
	const char* sid;
	if (param->pucRequest[0] == '/') {
		sid = param->pucRequest + 1;
	}
	else {
		sid = mwGetVarValue(param->pxVars, "id", "");
	}
	if (!sid || strlen(sid) < 4) {
		param->hs->response.statusCode = 403;
		param->contentLength = snprintf(param->pucBuffer, param->bufSize, "{\"result\":\"failed\",\"error\":\"Invalid ID\"}");
		return FLAG_DATA_RAW;
	}
	param->contentType = HTTPFILETYPE_JSON;
	param->contentLength = 0;
	CHANNEL_DATA* pld = locateChannel(param);
	if (!pld && event != EVENT_LOGIN) {
		param->hs->response.statusCode = 403;
		param->contentLength = snprintf(param->pucBuffer, param->bufSize, "{\"result\":\"failed\",\"error\":\"Invalid channel\"}");
		return FLAG_DATA_RAW;
	}

	uint64_t tick = GetTickCount64();
	if (event == EVENT_LOGIN) {
		if (!pld) {
			pld = assignChannel(sid);
			if (!pld) {
				param->hs->response.statusCode = 403;
				return FLAG_DATA_RAW;
			}
		}

		if (checkVIN(vin)) {
			strncpy(pld->vin, vin, sizeof(pld->vin) - 1);
		}
		pld->devflags = devflags;
		pld->rssi = rssi;
		pld->sessionStartTick = tick;
		pld->proxyTick = 0;
		pld->serverDataTick = tick;
		pld->ip = param->hs->ipAddr;
		deviceLogin(pld);
		param->contentLength = snprintf(param->pucBuffer, param->bufSize, "{\"id\":%u,\"result\":\"done\"}", pld->id);
		return FLAG_DATA_RAW;
	} else if (event == EVENT_LOGOUT) {
		param->contentLength = snprintf(param->pucBuffer, param->bufSize, "{\"result\":\"done\"}");
		if (pld->fp) {
			fclose(pld->fp);
			pld->fp = 0;
		}
		pld->flags &= ~FLAG_RUNNING;
		deviceLogout(pld);
		SaveChannels();
		return FLAG_DATA_RAW;
	}
	else if (event == EVENT_SYNC) {
		param->contentLength = snprintf(param->pucBuffer, param->bufSize, "{\"result\":\"done\"}");
		return FLAG_DATA_RAW;
	}

	param->contentLength = snprintf(param->pucBuffer, param->bufSize, "{\"result\":\"failed\",\"error\":\"Invalid request\"}");
	param->hs->response.statusCode = 400;
	return FLAG_DATA_RAW;
}

int uhPush(UrlHandlerParam* param)
{
	//mwParseQueryString(param);
	CHANNEL_DATA *pld = locateChannel(param);
	if (!pld) {
		param->hs->response.statusCode = 403;
		param->contentLength = 0;
		return FLAG_DATA_RAW;
	}

	int n;
	char *s;
	uint64_t tick = GetTickCount64();
	pld->deviceTick = mwGetVarValueInt(param->pxVars, "ts", 0);
	int count = 0;
	for (n = 0; n < param->iVarCount; n++) {
		s = param->pxVars[n].name;
		if (isNum(s)) {
			int pid = hex2uint16(s);
			int mode = pid >> 8;
			if (mode <= PID_MODES) {
				setPIDData(pld, pid, pld->deviceTick, param->pxVars[n].value);
				count++;
			}
		}
	}
	pld->serverDataTick = tick;
	pld->elapsedTime = (uint32_t)((pld->serverDataTick - pld->sessionStartTick) / 1000);
	pld->recvCount++;
	showLiveData(pld);
	param->contentType = HTTPFILETYPE_JSON;
	param->contentLength = snprintf(param->pucBuffer, param->bufSize, "{\"result\":%u}", count);
	return FLAG_DATA_RAW;
}

int uhTest(UrlHandlerParam* param)
{
	char content[64];
	time_t t = time(NULL);
	struct tm* btm = gmtime(&t);
	int len = snprintf(content, sizeof(content), "{\"date\":%02u%02u%02u,\"time\":%u%02u%02u,\"tick\":%u}",
		btm->tm_year - 100, btm->tm_mon + 1, btm->tm_mday, btm->tm_hour, btm->tm_min, btm->tm_sec, GetTickCount());
	param->contentLength = snprintf(param->pucBuffer, param->bufSize, "HTTP/1.1 200 OK\r\nServer:TeleServer\r\nContent-Length:%d\r\n\r\n%s",
		len, content);
	param->contentType = HTTPFILETYPE_TEXT;
	return FLAG_DATA_RAW | FLAG_CUSTOM_HEADER;
}

char* GetLocalAddrString()
{
	// get local ip address
	struct sockaddr_in sock;
	char hostname[256] = { 0 };
	struct hostent * lpHost;
	gethostname(hostname, sizeof(hostname));
	lpHost = gethostbyname(hostname);
	if (!lpHost) return "127.0.0.1";
	memcpy(&(sock.sin_addr), (void*)lpHost->h_addr_list[0], lpHost->h_length);
	return inet_ntoa(sock.sin_addr);
}

int ServerQuit(int arg) {
	static int quitting = 0;
	if (quitting) return 0;
	quitting = 1;
	if (arg) printf("\nCaught signal (%d). Shutting down...\n",arg);
	mwServerShutdown(&httpParam);
	SaveChannels();
	return 0;
}

void GetFullPath(char* buffer, char* argv0, char* path)
{
	char* p = strrchr(argv0, '/');
	if (!p) p = strrchr(argv0, '\\');
	if (!p) {
		strcpy(buffer, path);
	} else {
		size_t l = p - argv0 + 1;
		memcpy(buffer, argv0, l);
		strcpy(buffer + l, path);
	}
}

int genHttpPostPayload(CHANNEL_DATA* pld);

int main(int argc,char* argv[])
{
	fprintf(stderr, "Freematics Hub Version %s (built on %s)\n(C)2016-2020 Mediatronic Pty Ltd / Developed by Stanley Huang\nThis is free software and is distributed under GPL v3.0\n\n", REVISION, __DATE__);

#ifdef WIN32
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) ServerQuit, TRUE );
#else
	signal(SIGINT, (void *) ServerQuit);
	signal(SIGTERM, (void *) ServerQuit);
	signal(SIGPIPE, SIG_IGN);
#endif

	//fill in default settings
	char path[256];
	GetFullPath(path, argv[0], "app/htdocs");
	mwInitParam(&httpParam, 0, path, FLAG_DISABLE_RANGE, 0, 0);
	httpParam.maxClients = 256;
	httpParam.maxClientsPerIP = 16;
	httpParam.httpPort = 8080;
	httpParam.udpPort = 8081;
	httpParam.pxUrlHandler = urlHandlerList;
	httpParam.hlBindIP = htonl(INADDR_ANY);
	httpParam.pfnIncomingUDP = incomingUDPCallback;
	httpParam.pfnProxyData = phData;

#ifdef WIN32
	char dir[240];
	if (GetEnvironmentVariable("APPDATA", dir, sizeof(dir))) {
		strcat(dir, "/FreematicsHub");
		if (IsDir(dir)) {
			if (!IsDir(dataDir))
				snprintf(dataDir, sizeof(dataDir), "%s/Data", dir);
			if (!IsDir(logDir))
				snprintf(logDir, sizeof(logDir), "%s/Log", dir);
		}
	}
#endif

	loadConfig();

	//parsing command line arguments
	{
		int i;
		for (i=1;i<argc;i++) {
			if (argv[i][0]=='-') {
				switch (argv[i][1]) {
				case 'h':
					fprintf(stderr, "Usage: teleserver\n"
						"	-h	: display this help screen\n"
						"	-p	: specifiy http port [default 8080]\n"
						"	-u	: specifiy udp port [default 8081]\n"
						"	-l	: specify log file directory\n"
						"	-d	: specify data file directory\n"
						"	-m	: specifiy max clients [default 256]\n"
						"	-M	: specifiy max clients per IP\n"
						"	-n	: specifiy HTTP authentication user name for remote access [default: admin]\n"
						"	-w	: specifiy HTTP authentication password for remote access\n"
						"	-g	: do not launch GUI\n\n");
					fflush(stderr);
					exit(1);
					break;
				case 'p':
					if ((++i)<argc) httpParam.httpPort=atoi(argv[i]);
					break;
				case 'm':
					if ((++i)<argc) httpParam.maxClients=atoi(argv[i]);
					break;
				case 'M':
					if ((++i)<argc) httpParam.maxClientsPerIP=atoi(argv[i]);
					break;
				case 'g':
					noGUI = 1;
					break;
				case 'l':
					if ((++i)<argc) strncpy(logDir, argv[i], sizeof(logDir) - 1);
					break;
				case 'd':
					if ((++i)<argc) strncpy(dataDir, argv[i], sizeof(dataDir) - 1);
					break;
				case 'k':
					if ((++i)<argc) strncpy(serverKey, argv[i], sizeof(serverKey) - 1);
					break;
				case 'u':
					if (++i < argc) httpParam.udpPort = atoi(argv[i]);
					break;
				case 'n':
					if (++i < argc) strncpy(username, argv[i], sizeof(username) - 1);
					break;
				case 'w':
					if (++i < argc) strncpy(password, argv[i], sizeof(password) - 1);
					break;
				}
			}
		}
	}

	if (password[0]) {
		httpParam.pxAuthHandler = authHandlerList;
	}

	printf("Server Host: %s:%u\n", GetLocalAddrString(), httpParam.httpPort);
	if (httpParam.udpPort) {
		printf("UDP Port: %u\n", httpParam.udpPort);
	}
	printf("Max Channels: %u\n", MAX_CHANNELS);
	if (password[0]) {
		printf("Authentication: ON\n");
	}
	printf("\nWeb UI:\nhttp://%s:%u\n\n", GetLocalAddrString(), httpParam.httpPort);
	printf("Data Feed Simulator:\nhttp://%s:%u/simulator.html\n\n", GetLocalAddrString(), httpParam.httpPort);

	memset(ld, 0, sizeof(ld));
	LoadChannels();

	if (mwServerStart(&httpParam)) {
		printf("Error starting HTTP server on port %u\nPress ENTER to exit\n", httpParam.httpPort);
		return -1;
	}

	int ret = -1;
	SHELL_PARAM proc = { 0 };
#ifdef WIN32
	if (!noGUI) {
		proc.flags = SF_SHOW_WINDOW;
		ret = ShellExec(&proc, "electron/electron app");
	}
#endif
	if (ret == 0) {
		do {
			mwHttpLoop(&httpParam, 500);
			CheckChannels();
		} while (!httpParam.bKillWebserver && ShellWait(&proc, 0) == 0);
	}
	else if (ret == -1) {
		do {
			mwHttpLoop(&httpParam, 1000);
			CheckChannels();
		} while (!httpParam.bKillWebserver);
	}
	else if (ret == -2) {
		// child process (execve failed)
		return 0;
	}

	mwServerExit(&httpParam);
	return 0;
}
////////////////////////////// END OF FILE //////////////////////////////
