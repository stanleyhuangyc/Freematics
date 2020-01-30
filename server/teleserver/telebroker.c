/******************************************************************************
* Freematics Hub Server - Broker
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
#include "cJSON.h"
#include "cdecode.h"
#include "httpd.h"
#include "teleserver.h"
#include "logdata.h"

extern CHANNEL_DATA ld[];

char* genHttpPostPayload(CHANNEL_DATA* pld)
{
	int n = 0;
	cJSON* root = NULL;
	cJSON* data = NULL;
	cJSON* parameters = NULL;
	cJSON* obd2 = NULL;
	cJSON* obd2_data = NULL;
	cJSON* custom = NULL;
	cJSON* custom_data = NULL;
	char* out = NULL;

	if (!pld) return 0;

	root = cJSON_CreateObject();

	cJSON_AddItemToObject(root, "data", data = cJSON_CreateObject());
	cJSON_AddItemToObject(data, "device", cJSON_CreateString(pld->devid));
	cJSON_AddItemToObject(data, "parameters", parameters = cJSON_CreateObject());
	cJSON_AddItemToObject(parameters, "obd2", obd2 = cJSON_CreateObject());
	cJSON_AddItemToObject(obd2, "data", obd2_data = cJSON_CreateObject());
	cJSON_AddNumberToObject(obd2_data, "0x104", atoi(pld->data[PID_ENGINE_LOAD].value));
	cJSON_AddNumberToObject(obd2_data, "0x105", atoi(pld->data[PID_COOLANT_TEMP].value));
	cJSON_AddNumberToObject(obd2_data, "0x10A", atoi(pld->data[PID_FUEL_PRESSURE].value));
	cJSON_AddNumberToObject(obd2_data, "0x10B", atoi(pld->data[PID_INTAKE_PRESSURE].value));
	cJSON_AddNumberToObject(obd2_data, "0x10F", atoi(pld->data[PID_INTAKE_TEMP].value));
	cJSON_AddNumberToObject(obd2_data, "0x110", atoi(pld->data[PID_MAF_FLOW].value));
	cJSON_AddNumberToObject(obd2_data, "0x11F", atoi(pld->data[PID_RUNTIME].value));
	cJSON_AddNumberToObject(obd2_data, "0x12F", atoi(pld->data[PID_FUEL_LEVEL].value));
	cJSON_AddNumberToObject(obd2_data, "0x131", atoi(pld->data[PID_DISTANCE].value));
	cJSON_AddNumberToObject(obd2_data, "0x146", atoi(pld->data[PID_AMBIENT_TEMP].value));
	cJSON_AddItemToObject(parameters, "custom", custom = cJSON_CreateObject());
	cJSON_AddItemToObject(custom, "data", custom_data = cJSON_CreateObject());
	cJSON_AddItemToObject(custom_data, "0xA", cJSON_CreateString(pld->data[PID_GPS_LATITUDE].value));
	cJSON_AddItemToObject(custom_data, "0xB", cJSON_CreateString(pld->data[PID_GPS_LONGITUDE].value));
	cJSON_AddItemToObject(custom_data, "0xC", cJSON_CreateString(pld->data[PID_GPS_ALTITUDE].value));
	cJSON_AddItemToObject(custom_data, "0xD", cJSON_CreateString(pld->data[PID_GPS_SPEED].value));
	cJSON_AddItemToObject(custom_data, "0xE", cJSON_CreateString(pld->data[PID_GPS_HEADING].value));
	cJSON_AddItemToObject(custom_data, "0xF", cJSON_CreateString(pld->data[PID_GPS_SAT_COUNT].value));
	cJSON_AddItemToObject(custom_data, "0x10", cJSON_CreateString(pld->data[PID_GPS_TIME].value));
	cJSON_AddItemToObject(custom_data, "0x11", cJSON_CreateString(pld->data[PID_GPS_DATE].value));
	cJSON_AddItemToObject(custom_data, "0x12", cJSON_CreateString(pld->data[PID_GPS_HDOP].value));

	out = cJSON_Print(root);
	printf("[Response JSON]\n%s\n", out);

	cJSON_Delete(root);
	return out;
}

#define HTTP_REQUEST_TEMPLATE "POST /odb/store HTTP/1.1\r\n\
Content-Type: application/vnd.api+json\r\n\
Content-Length: %u\r\n\
Connection: keep-alive\r\n\
Accept: application/vnd.api+json\r\n\
Api-Key: [API token]\r\n\r\n%s"

int genRequest(char* buf, int bufsize)
{
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
			CHANNEL_DATA* pld = ld + i;

			if ((pld->flags & FLAG_PINGED)) {
				len = snprintf(buf, bufsize, "GET ?id=%s HTTP/1.1\r\nConnection: keep-alive\r\n\r\n", pld->devid);
				pld->flags &= ~FLAG_PINGED;
				fprintf(stderr, "Ping responded\n");
				break;
			}

			if (pld->deviceTick == pld->proxyTick) continue;

			char isoTime[26];
			char* p = isoTime + sprintf(isoTime, "%04u-%02u-%02uT%02u:%02u:%02u",
				(unsigned int)(date % 100) + 2000, (unsigned int)(date / 100) % 100, (unsigned int)(date / 10000),
				(unsigned int)(time / 1000000), (unsigned int)(time % 1000000) / 10000, (unsigned int)(time % 10000) / 100);
			unsigned char tenth = (time % 100) / 10;
			if (tenth) p += sprintf(p, ".%c00", '0' + tenth);
			*p = 'Z';
			*(p + 1) = 0;

			len = snprintf(buf, bufsize, "GET ?id=%s&timestamp=%s&lat=%s&lon=%s&altitude=%s&speed=%.2f&heading=%s&hdop=%.1f HTTP/1.1\r\nConnection: keep-alive\r\n\r\n",
				pld->devid, isoTime,
				pld->data[PID_GPS_LATITUDE].value,
				pld->data[PID_GPS_LONGITUDE].value,
				pld->data[PID_GPS_ALTITUDE].value,
				(float)atof(pld->data[PID_GPS_SPEED].value) / 1.852f,
				pld->data[PID_GPS_HEADING].value,
				(float)atoi(pld->data[PID_GPS_HDOP].value) / 10);

			/*
			char* payload = genHttpPostPayload(pld);
			len = snprintf(buf, bufsize, HTTP_REQUEST_TEMPLATE, (unsigned int)strlen(payload), payload);
			free(payload);
			*/

			pld->proxyTick = pld->deviceTick;
			break;
		}
	}
	if (len > 0) printf("%s", buf);
	return len;
}

int phData(void* _hp, int op, char* buf, int len)
{
	if (op == PROXY_DATA_REQUESTED) {
		int bytes = genRequest(buf, len);
		return bytes;
	}
	else if (op == PROXY_DATA_RECEIVED) {
		printf("%s", buf);
/*
		char* p = strstr(buf, "\r\n\r\n");
		if (!p) {
			printf("Invalid response:\n%s", buf);
			return -1;
		}
		if (strstr(buf, "HTTP/1.1 200 OK") && (p = strchr(p, '{'))) {
			cJSON* response = cJSON_Parse(p);
			cJSON* status = cJSON_GetObjectItem(response, "status");
			cJSON* code = cJSON_GetObjectItem(status, "code");
			cJSON* msg = cJSON_GetObjectItem(status, "msg");
			cJSON* rsp = cJSON_GetObjectItem(response, "response");
			if (code && msg && rsp) {
				printf("code:%u msg:%s response:%s\n",
					code->valueint,
					msg->valuestring,
					rsp->valuestring
				);
			}
			cJSON_Delete(response);
		}
*/
	}
	return 0;
}
