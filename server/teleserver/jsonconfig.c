#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include "cJSON.h"

#ifdef WIN32
#define strcasecmp _stricmp
#endif

cJSON* users = 0;

char* loadFile(const char* fn)
{
	FILE* fp = fopen(fn, "r");
	if (!fp) return 0;
	fseek(fp, 0, SEEK_END);
	int len = ftell(fp);
	char* buf = malloc(len + 1);
	fseek(fp, 0, SEEK_SET);
	int n = fread(buf, 1, len, fp);
	buf[n] = 0;
	fclose(fp);
	return buf;
}

int getUserInfo(const char* username, char** ppassword, char* pdevid[], int maxdev)
{
	if (!users) return 0;
	for (cJSON* entry = users->child; entry; entry = entry->next) {
		cJSON* id = cJSON_GetObjectItem(entry, "id");
		if (!id || !cJSON_IsString(id) || strcmp(id->valuestring, username)) {
			continue;
		}
		cJSON* devid = cJSON_GetObjectItem(entry, "devid");
		if (!devid) continue;
		cJSON* traccar = cJSON_GetObjectItem(entry, "traccar");
		if (traccar) *ppassword = traccar->valuestring;
		if (cJSON_IsArray(devid)) {
			int length = cJSON_GetArraySize(devid);
			if (length > maxdev) length = maxdev;
			for (int n = 0; n < length; n++) {
				cJSON* item = cJSON_GetArrayItem(devid, n);
				if (item) pdevid[n] = item->valuestring;
			}
			return length;
		}
		else if (cJSON_IsString(devid)) {
			pdevid[0] = devid->valuestring;
			return 1;
		}
	}
	return 0;
}

char* getUserByDeviceID(const char* devid)
{
	if (!users) return 0;
	for (cJSON *entry = users->child; entry; entry = entry->next) {
		cJSON *item = entry->child;
		if (item->next && item->next->valuestring && !strcmp(devid, item->next->valuestring)) {
			return item->valuestring;
		}
	}
	return 0;
}

int loadConfig()
{
	static time_t m_time = 0;
	struct stat st = { 0 };
	const char* fn = "config/users.json";
	if (stat(fn, &st) != 0)
		return -1;
	if (st.st_mtime == m_time)
		return 0;
	m_time = st.st_mtime;
	
	FILE* fp = fopen(fn, "r");
	if (!fp) return -1;
	int len = st.st_size;
	char* content = malloc(len + 1);
	int n = fread(content, 1, len, fp);
	content[n] = 0;
	fclose(fp);

	if (users) cJSON_Delete(users);
	users = cJSON_Parse(content);
	free(content);
	return 0;
}