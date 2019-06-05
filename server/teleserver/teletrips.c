/******************************************************************************
* Freematics Hub Server - Trip APIs
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
#include "cdecode.h"
#include "httpd.h"
#include "teleserver.h"
#include "logdata.h"
#include "data2kml.h"

extern CHANNEL_DATA ld[];

int loadConfig();
char* getUserByDeviceID(const char* devid);
int getUserInfo(const char* username, char** ppassword, char* pdevid[], int maxdev);

#define MAX_UPLOAD_SIZE 256 * 1024

char fileid[17];
int error = 0;
const char* errmsg[] = {"Invalid data format", "File creation error"};

FILE* fpDest;
char* xsl;
extern char dataDir[];

int genRequest(char* buf, int bufsize)
{
	int valids = 0;
	uint32_t date = 0;
	uint32_t time = 0;
	float lat = 0;
	float lng = 0;
	float alt = 0;
	float speed = 0;
	int heading = 0;
	int hdop = 0;
	uint8_t mask = 0;
	int len = 0;

	for (int i = 0; i < MAX_CHANNELS; i++) {
		if (ld[i].id) {
			uint32_t startTick = ld[i].proxyTick;
			CHANNEL_DATA* pld = ld + i;
			//if (pld->deviceTick > startTick + PROXY_MAX_TIME_BEHIND) startTick = pld->deviceTick - PROXY_MAX_TIME_BEHIND;
			uint32_t readPos = pld->cacheReadPos;
			mask = 0;
			for (; readPos != pld->cacheWritePos; readPos = (readPos + 1) % pld->cacheSize) {
				CACHE_DATA* d = pld->cache + readPos;
				if (d->ts <= startTick) continue;
				if (pld->proxyTick != d->ts) {
					pld->proxyTick = d->ts;
					// new time stamp, check validity
					if (mask == 0xff) {
						break;
					}
				}
				switch (d->pid) {
				case PID_GPS_DATE:
					date = atol(d->data);
					mask |= 0x1;
					break;
				case PID_GPS_TIME:
					time = atol(d->data);
					mask |= 0x2;
					break;
				case PID_GPS_LATITUDE:
					lat = (float)atof(d->data);
					mask |= 0x4;
					break;
				case PID_GPS_LONGITUDE:
					lng = (float)atof(d->data);
					mask |= 0x8;
					break;
				case PID_GPS_ALTITUDE:
					alt = (float)atof(d->data);
					mask |= 0x10;
					break;
				case PID_GPS_SPEED:
					speed = (float)atof(d->data);
					mask |= 0x20;
					break;
				case PID_GPS_HEADING:
					heading = atoi(d->data);
					mask |= 0x40;
					break;
				case PID_GPS_HDOP:
					hdop = atoi(d->data);
					break;
				}
				mask |= 0x80;
			}
			if (mask == 0xff) {
				// complete set of GPS data available
				char isoTime[26];
				char* p = isoTime + sprintf(isoTime, "%04u-%02u-%02uT%02u:%02u:%02u",
					(unsigned int)(date % 100) + 2000, (unsigned int)(date / 100) % 100, (unsigned int)(date / 10000),
					(unsigned int)(time / 1000000), (unsigned int)(time % 1000000) / 10000, (unsigned int)(time % 10000) / 100);
				unsigned char tenth = (time % 100) / 10;
				if (tenth) p += sprintf(p, ".%c00", '0' + tenth);
				*p = 'Z';
				*(p + 1) = 0;
				len = snprintf(buf, bufsize, "GET ?id=%s&timestamp=%s&lat=%f&lon=%f&altitude=%.1f&speed=%.2f&heading=%d&hdop=%.1f HTTP/1.1\r\nConnection: keep-alive\r\n\r\n",
					pld->devid, isoTime, lat, lng, alt, speed / 1.852f, heading, (float)hdop / 10);
				break;
			}
			else if (mask) {
				// no complete set of GPS data but still some new data
				len = snprintf(buf, bufsize, "GET ?id=%s HTTP/1.1\r\nConnection: keep-alive\r\n\r\n", pld->devid);
				break;
			}
			else if ((pld->flags & FLAG_PINGED)) {
				len = snprintf(buf, bufsize, "GET ?id=%s HTTP/1.1\r\nConnection: keep-alive\r\n\r\n", pld->devid);
				pld->flags &= ~FLAG_PINGED;
				fprintf(stderr, "Ping responded\n");
				break;
			}
		}
	}
	if (len > 0) printf("%s", buf);
	return len;
}

int phData(void* _hp, int op, char* buf, int len)
{
	HttpParam* hp = _hp;
	static int state = 0;
	if (op == PROXY_DATA_REQUESTED) {
		int bytes = genRequest(buf, len);
		return bytes;
	}
	else if (op == PROXY_DATA_RECEIVED) {
		if (strstr(buf, "HTTP/1.1 200 OK")) {
		}
		else {
			printf("%s", buf);
		}
	}
	return 0;
}

int uhQuery(UrlHandlerParam* param)
{
	param->contentType = HTTPFILETYPE_JSON;
	param->contentLength = snprintf(param->pucBuffer, param->bufSize, "{}");

	loadConfig();

	const char* userb64 = mwGetVarValue(param->pxVars, "user", 0);
	if (!userb64 || !*userb64) return FLAG_DATA_RAW;

	int len = strlen(userb64);
	char* user = malloc(len + 1);
	base64_decode_chars(userb64, len, user);
	char* devids[4] = { 0 };
	char* password = 0;
	int devcount = getUserInfo(user, &password, devids, 4);
	free(user);
	if (devcount == 0) {
		return FLAG_DATA_RAW;
	}
	char* buf = param->pucBuffer;
	int bs = param->bufSize;
	len = snprintf(buf, bs, "{\"traccar\":\"%s\",\"devid\":[", password);
	for (int i = 0; i < devcount; i++) {
		len += snprintf(buf + len, bs - len, "\"%s\",", devids[i]);
	}
	len--;
	len += snprintf(buf + len, bs - len, "]}");
	param->contentLength = len;
	return FLAG_DATA_RAW;
}

#if 0
int UploadCallback(HttpMultipart *pxMP, OCTET *poData, size_t dwDataChunkSize)
{
  // Do nothing with the data
	int fd = (int)pxMP->pxCallBackData;
	if (!poData) {
		// to cleanup
		if (fd > 0) {
			close(fd);
			pxMP->pxCallBackData = NULL;
		}
		return 0;
	}
	if (fd == 0) {
		char filename[256];
		char* dir = 0;
		time_t t;
		struct tm *btm;
		int n;

		for (n = 0; n < pxMP->pp.iNumParams; n++) {
			if (!strcmp(pxMP->pp.stParams[n].pchParamName, "user")) {
				dir = pxMP->pp.stParams[n].pchParamValue;
			}
		}
		if (!dir) {
			return -1;
		}
		t = time(0);
		btm = localtime(&t);
		n = sprintf(filename, "%02d%02d%02d%02d%02d%02d_", btm->tm_year - 100, btm->tm_mon + 1, btm->tm_mday, btm->tm_hour, btm->tm_min, btm->tm_sec);
		memmove(pxMP->pchFilename + n, pxMP->pchFilename, strlen(pxMP->pchFilename) + 1);
		memcpy(pxMP->pchFilename, filename, n);
		snprintf(filename, sizeof(filename), "data/%s/%s", dir, pxMP->pchFilename);
		fd = open(filename, O_CREAT | O_TRUNC | O_RDWR | O_BINARY, 0);
		pxMP->pxCallBackData = (void*)fd;
		if (fd < 0) {
			pxMP->pchFilename[0] = 0;
			return -1;
		}
	} else if (fd < 0) {
		return 0;
	}
	if (pxMP->bytesReceived > MAX_UPLOAD_SIZE) {
		close(fd);
		return -1;
	}
	write(fd, poData, dwDataChunkSize);
	if (pxMP->oFileuploadStatus & HTTPUPLOAD_LASTCHUNK) {
		close(fd);
		pxMP->pxCallBackData = NULL;
	}
	printf("Received %u bytes for multipart upload file %s\n", dwDataChunkSize, pxMP->pchFilename);
	return 0;
}
#endif

int ConvertToKML(KML_DATA* kd, FILE* fp, const char* kmlfile, uint32_t startpos, uint32_t endpos);
void CleanupKML(KML_DATA* kd);

void WriteGeoJSON(FILE* fpout, KML_DATA* kd, int size, int count)
{
	int pos = fprintf(fpout, "{\"meta\":{\"rev\":%u,\"size\":%u,\"samples\":%u,\"duration\":", META_REVISION, size, count);
	fprintf(fpout, "0         }");

	if (!kd->data) {
		return;
	}

	fprintf(fpout, ",\n");

	DATASET* end;
	for (end = kd->data; end->next; end = end->next);

	fprintf(fpout, "\"stats\":{\"distance\":%u,\"start\":{\"lat\":%f,\"lng\":%f,\"date\":%u,\"time\":%u,\"ts\":%u},\"end\":{\"lat\":%f,\"lng\":%f,\"date\":%u,\"time\":%u,\"ts\":%u}},\n",
		(unsigned int)kd->distance,
		kd->data->lat, kd->data->lng, kd->data->date, kd->data->time, kd->data->timestamp,
		end->lat, end->lng, end->date, end->time, end->timestamp);

	fprintf(fpout, "\"bounds\":[{\"lat\":%f,\"lng\":%f}, {\"lat\":%f,\"lng\":%f}],\n",
		kd->bounds[0].lat, kd->bounds[0].lng, kd->bounds[1].lat, kd->bounds[1].lng);

	fprintf(fpout, "\"pids\":[0");
	for (int n = 1; n < 65536; n++) {
		if (kd->pidMap[n]) fprintf(fpout, ",%u", n);
	}
	fprintf(fpout, "],\n");
	fprintf(fpout, "\"trip\":{\"type\":\"LineString\"");
	DATASET* pd = kd->data;
	fprintf(fpout, ",\"coordinates\":[[%f,%f]", pd->lng, pd->lat);
	while (pd = pd->next) fprintf(fpout, ",[%f,%f]", pd->lng, pd->lat);
	fprintf(fpout, "],\n");

	pd = kd->data;
	uint32_t t = kd->data->timestamp;
	fprintf(fpout, "\"timestamps\":[%u", pd->timestamp - t);
	while (pd = pd->next) fprintf(fpout, ",%u", pd->timestamp - t);
	fprintf(fpout, "],\n");

	pd = kd->data;
	fprintf(fpout, "\"altitudes\":[%d", (int)pd->alt);
	while (pd = pd->next) fprintf(fpout, ",%d", (int)pd->alt);
	fprintf(fpout, "],\n");

	pd = kd->data;
	fprintf(fpout, "\"accels\":[[%d,%d,%d]", pd->acc[0], pd->acc[1], pd->acc[2]);
	while (pd = pd->next) fprintf(fpout, ",[%d,%d,%d]", pd->acc[0], pd->acc[1], pd->acc[2]);
	fprintf(fpout, "],\n");

	pd = kd->data;
	fprintf(fpout, "\"battery\":[%.1f", (float)pd->battery / 100);
	while (pd = pd->next) fprintf(fpout, ",%.1f", (float)pd->battery / 100);
	fprintf(fpout, "],\n");

	pd = kd->data;
	fprintf(fpout, "\"speeds\":[%.1f", pd->speed);
	while (pd = pd->next) fprintf(fpout, ",%.1f", pd->speed);
	fprintf(fpout, "]\n");
	fprintf(fpout, "}\n");

	fprintf(fpout, "}");
	if (end->timestamp > kd->data->timestamp) {
		fseek(fpout, pos, SEEK_SET);
		fprintf(fpout, "%u", end->timestamp - kd->data->timestamp);
	}
}

int CreateDataFiles(KML_DATA* kd, const char* file)
{
	char path[256];
	FILE* fp;
	int count;
	int size;

	snprintf(path, sizeof(path), "%s/%s.txt", dataDir, file);
	fp = fopen(path, "r");
	if (!fp) {
		return -1;
	}
	snprintf(path, sizeof(path), "%s/%s.kml", dataDir, file);
	count = ConvertToKML(kd, fp, path, 0, 0);
	fseek(fp, 0, SEEK_END);
	size = ftell(fp);
	fclose(fp);

	snprintf(path, sizeof(path), "%s/%s.json", dataDir, file);
	fp = fopen(path, "w");
	WriteGeoJSON(fp, kd, size, count);
	fclose(fp);
	return count;
}

int loadMetaInfo(const char* file, uint32_t* duration, uint32_t* size)
{
	FILE* fp = fopen(file, "r");
	int rev = 0;
	if (fp) {
		char buf[256];
		int n = fread(buf, 1, sizeof(buf) - 1, fp);
		fclose(fp);
		if (n > 0) {
			buf[n] = 0;
			char* p;
			if (p = strstr(buf, "\"rev\":")) rev = atoi(p + 6);
			if (duration && (p = strstr(buf, "\"duration\":"))) * duration = atoi(p + 11);
			if (size && (p = strstr(buf, "\"size\":"))) * size = atoi(p + 7);
		}
	}
	return rev;
}

int uhData(UrlHandlerParam* param)
{
	const char* devid = mwGetVarValue(param->pxVars, "devid", 0);
	const char* tripid = mwGetVarValue(param->pxVars, "tripid", 0);
	int64_t offset = mwGetVarValueInt64(param->pxVars, "offset");
	int pidreq = mwGetVarValueInt(param->pxVars, "pid", 0);
	param->contentType = HTTPFILETYPE_TEXT;

	int devidlen = strlen(devid);
	if (devidlen < MIN_DEVID_LEN || devidlen > MAX_DEVID_LEN) {
		param->contentLength = sprintf(param->pucBuffer, "Invalid device ID");
		return FLAG_DATA_RAW;
	}
	if (!devid || !tripid || strlen(tripid) != 15) {
		param->contentLength = sprintf(param->pucBuffer, "Invalid arguments");
		return FLAG_DATA_RAW;
	}

	char buf[1024];
	char* p = buf + snprintf(buf, 66, "%s/", devid);
	memcpy(p, tripid, 4);
	p += 4;
	*(p++) = '/';
	memcpy(p, tripid + 4, 2);
	p += 2;
	*(p++) = '/';
	memcpy(p, tripid + 6, 2);
	p += 2;
	*(p++) = '/';
	strcpy(p, tripid);

	param->contentType = HTTPFILETYPE_JSON;
	snprintf(param->pucBuffer, param->bufSize, "%s/%s.txt", dataDir, buf);
	FILE* fp = fopen(param->pucBuffer, "r");
	if (!fp) {
		param->contentLength = sprintf(param->pucBuffer, "Data file not found");
		return FLAG_DATA_RAW;
	}

	uint32_t ts = 0;
	int len = 0;
	len += snprintf(param->pucBuffer + len, param->bufSize - len, "[");
	while (fscanf(fp, "%1024s\n", buf) > 0) {
		for (char* p = strtok(buf, ","); p; p = strtok(0, ",")) {
			int pid = hex2uint16(p);
			if (!(p = strchr(p, ':'))) break;
			float value[3] = { 0 };
			int n = 0;
			do {
				value[n++] = (float)atof(++p);
				if (!(p = strchr(p, ';'))) break;
			} while (n < 3);
			if (pid == 0) {
				ts = (uint32_t)value[0];
				continue;
			}
			if (pid == pidreq) {
				if (n == 1) {
					if (pid >= 0x100)
						len += snprintf(param->pucBuffer + len, param->bufSize - len, "[%lld,%d],", offset + ts, (int)value[0]);
					else
						len += snprintf(param->pucBuffer + len, param->bufSize - len, "[%lld,%.2f],", offset + ts, value[0]);
				}
				else {
					len += snprintf(param->pucBuffer + len, param->bufSize - len, "[%lld,[%d,%d,%d]],", offset + ts, (int)value[0], (int)value[1], (int)value[2]);
				}
			}
		}
	}
	fclose(fp);
	if (param->pucBuffer[len - 1] == ',') len--;
	len += snprintf(param->pucBuffer + len, param->bufSize - len, "]");
	param->contentLength = len;
	return FLAG_DATA_RAW;
}

int processTripData(const char* devid, const char* tripid, int force, char* file, uint32_t* psize, uint32_t* pduration)
{
	char* p = file + snprintf(file, 100, "%s/", devid);
	memcpy(p, tripid, 4);
	p += 4;
	*(p++) = '/';
	memcpy(p, tripid + 4, 2);
	p += 2;
	*(p++) = '/';
	memcpy(p, tripid + 6, 2);
	p += 2;
	*(p++) = '/';
	strcpy(p, tripid);

	int processed = 0;

	char path[256];
	snprintf(path, sizeof(path), "%s/%s.json", dataDir, file);
	uint32_t size = 0, duration = 0;
	int rev = loadMetaInfo(path, &duration, &size);
	if (rev == META_REVISION) {
		snprintf(path, sizeof(path), "%s/%s.txt", dataDir, file);
		FILE* fp = fopen(path, "r");
		if (fp) {
			fseek(fp, 0, SEEK_END);
			if (ftell(fp) == size) processed = 1;
		}
		fclose(fp);
		if (psize)* psize = size;
		if (pduration)* pduration = duration;
	}

	if (force || !processed) {
		KML_DATA kd = { 0 };
		int count = CreateDataFiles(&kd, file);
		CleanupKML(&kd);
		if (count <= 0) {
			return -1;
		}
		int rev = loadMetaInfo(path, &duration, &size);
		if (rev == META_REVISION) {
			if (psize)* psize = size;
			if (pduration)* pduration = duration;
		}
	}

	return 0;
}

int uhTrip(UrlHandlerParam* param)
{
	const char* devid = mwGetVarValue(param->pxVars, "devid", 0);
	const char* tripid = mwGetVarValue(param->pxVars, "tripid", 0);
	const char* redir = mwGetVarValue(param->pxVars, "redir", 0);
	int regen = mwGetVarValueInt(param->pxVars, "regen", 0);
	const char* ext = "json";
	param->contentType = HTTPFILETYPE_TEXT;

	int devidlen = strlen(devid);
	if (devidlen < MIN_DEVID_LEN || devidlen > MAX_DEVID_LEN) {
		param->contentLength = sprintf(param->pucBuffer, "Invalid device ID");
		return FLAG_DATA_RAW;
	}
	if (!devid || !tripid || strlen(tripid) != 15) {
		param->contentLength = sprintf(param->pucBuffer, "Invalid arguments");
		return FLAG_DATA_RAW;
	}

	char file[128];
	if (processTripData(devid, tripid, regen, file, 0, 0) == -1) {
		param->contentLength = sprintf(param->pucBuffer, "{\"status\":2,\"error\":\"No data\"}");
		return FLAG_DATA_RAW;
	}

	if (!strcmp(param->pucRequest, "/kml")) {
		ext = "kml";
		param->contentType = HTTPFILETYPE_XML;
	} else if (!strcmp(param->pucRequest, "/raw")) {
		ext = "txt";
		param->contentType = HTTPFILETYPE_TEXT;
	}
	else {
		param->contentType = HTTPFILETYPE_JSON;
	}

	if (redir) {
		snprintf(param->pucBuffer, param->bufSize, "%s/%s.%s", redir, file, ext);
		return FLAG_DATA_REDIRECT;
	} else {
		snprintf(param->pucBuffer, param->bufSize, "%s/%s.%s", dataDir, file, ext);
		return FLAG_DATA_FILE | FLAG_ABSOLUTE_PATH;
	}	
}

void getDateTimeInt(const char* isotime, int* dateint, int* timeint)
{
	int year = 0;
	int month = 0;
	int day = 0;
	int hour = 0;
	int minute = 0;
	int second = 0;
	do {
		const char* s = isotime;
		year = atoi(s);
		if (!(s = strchr(s, '-'))) break;
		month = atoi(++s);
		if (!(s = strchr(s, '-'))) break;
		day = atoi(++s);
		if (!(s = strchr(s, 'T'))) break;
		hour = atoi(++s);
		if (!(s = strchr(s, ':'))) break;
		minute = atoi(++s);
		if (!(s = strchr(s, ':'))) break;
		second = atoi(++s);
	} while (0);
	*dateint = year * 10000 + month * 100 + day;
	*timeint = hour * 10000 + minute * 100 + second;
}

void getDateTimeBreakdown(const char* isotime, int* year, int* month, int* day, int* hour, int* minute, int* second)
{
	*year = 0;
	*month = 0;
	*day = 0;
	*hour = 0;
	*minute = 0;
	*second = 0;
	do {
		const char* s = isotime;
		*year = atoi(s);
		if (!(s = strchr(s, '-'))) break;
		*month = atoi(++s);
		if (!(s = strchr(s, '-'))) break;
		*day = atoi(++s);
		if (!(s = strchr(s, 'T'))) break;
		*hour = atoi(++s);
		if (!(s = strchr(s, ':'))) break;
		*minute = atoi(++s);
		if (!(s = strchr(s, ':'))) break;
		*second = atoi(++s);
	} while (0);
}

int uhHistory(UrlHandlerParam* param)
{
	const char* szbegin = mwGetVarValue(param->pxVars, "begin", 0);
	const char* szend = mwGetVarValue(param->pxVars, "end", 0);
	const char* devid = mwGetVarValue(param->pxVars, "devid", 0);
	char *pb = param->pucBuffer;
	int bs = param->bufSize;

	if (!szbegin || !szend || !devid) return 0;
	char path[260];
	snprintf(path, sizeof(path), "%s/%s", dataDir, devid);
	if (!IsDir(path)) {
		return 0;
	}

	unsigned int beginDate = 0, beginTime = 0, endDate = 0, endTime = 0;
	getDateTimeInt(szbegin, &beginDate, &beginTime);
	getDateTimeInt(szend, &endDate, &endTime);
	if (beginDate == 0 || endDate == 0 || beginDate > endDate) {
		return 0;
	}

	int year, month, day, hour, minute, second;
	getDateTimeBreakdown(szbegin, &year, &month, &day, &hour, &minute, &second);

	int eod = 0;
	int n = 0;
	n += snprintf(pb + n, bs - n, "[\n");
	int count = 0;
	for (unsigned int date = beginDate; date <= endDate && count <= 365; count++) {
		char *p = path + snprintf(path, sizeof(path), "%s/%s/%04u/%02u/%02u",
			dataDir, devid, year, month, day);
		char file[260];
		if (ReadDir(path, file) == 0) {
			do {
				if (strlen(file) != 19 || file[8] != '-') continue;
				char *q = strchr(file, '.');
				if (!q || strcmp(q + 1, "txt")) continue;
				sprintf(p, "/%s", file);
				*q = 0;
				unsigned int time = atoi(file + 9);
				date = atoi(file);
				if (date < beginDate || date > endDate)
					continue;
				if ((date == beginDate && time < beginTime) || (date == endDate && endTime && time > endTime))
					continue;


				// retrieve meta data
				uint32_t duration = 0;
				uint32_t size = 0;
				char filepath[128];
				if (processTripData(devid, file, 0, filepath, &size, &duration) == -1) {
					continue;
				}

				hour = time / 10000;
				minute = (time / 100) % 100;
				second = time % 100;
				struct tm t = { second, minute, hour, day, month - 1, year - 1900 };
				time_t tm = mktime(&t);
				n += snprintf(pb + n, bs - n, "{\"id\":\"%s\",\"key\":%u,\"utc\":\"%04u-%02u-%02uT%02u:%02u:%02uZ\",\"size\":%u,\"duration\":%u},",
					file, (unsigned int)tm,
					year, month, day, hour, minute, second,
					size, duration
				);
			} while (ReadDir(0, file) == 0);
		}

		if (!eod) {
			if (month == 4 || month == 6 || month == 9 || month == 11)
				eod = 30;
			else if (month == 2)
				eod = (year % 4 == 0) ? 29 : 28;
			else
				eod = 31;
		}

		if (++day > eod) {
			day = 1;
			if (++month > 12) {
				month = 1;
				year++;
			}
			eod = 0;
		}
		date = year * 10000 + month * 100 + day;
	}
	n--;
	n += snprintf(pb + n, bs - n, "]");
	param->contentLength = n;
	param->contentType = HTTPFILETYPE_JSON;
	return FLAG_DATA_RAW;
}
