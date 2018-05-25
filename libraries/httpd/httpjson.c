/////////////////////////////////////////////////////////////////////////////
//
// httpjson.c
//
// JSON parser
//
/////////////////////////////////////////////////////////////////////////////

#ifndef WINCE
#include <fcntl.h>
#include <errno.h>
#include <sys/stat.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "httpd.h"
#include "httpint.h"

void _mwFreeJSONPairs(UrlHandlerParam* up)
{
	if (up->json) {
		int i;
		for (i = 0; i < up->jsonPairCount; i++) {
			free(up->json[i].name);
			free(up->json[i].value);
		}
		free(up->json);
		up->json = 0;
		up->jsonPairCount = 0;
	}
}

NameValuePair* mwGetJSONData(UrlHandlerParam* up, const char* name)
{
	int i;
	for (i = 0; i < up->jsonPairCount; i++) {
		if (!strcmp(up->json[i].name, name)) {
			return up->json + i;
		}
	}
	return 0;
}

int mwParseJSONString(UrlHandlerParam* up)
{
	char *p = up->pucPayload;
	if (!p) return 0;

	NameValuePair pair;
	char keybase[256] = { 0 };
	int curLevel = 0;
	_mwFreeJSONPairs(up);

	while (*p) {
		switch (*p) {
		case '{':
			curLevel++;
			p++;
			break;
		case '}':
			curLevel--;
			if (*keybase) {
				char *q;
				for (q = keybase + strlen(keybase) - 2; q >= keybase && *q != '.'; q--);
				*(q + 1) = 0;
			}
			p++;
			break;
		case '\"':
			{
				char *q;
				;
				if (!(q = strchr(p + 1, '\"'))) continue;
				pair.name = p + 1;
				if (!(p = strchr(q + 1, ':'))) continue;
				while (*(++p) == ' ' || *p == '\r' || *p == '\n');
				int i = up->jsonPairCount;
				if (*p == '\"') {
					pair.type = JSON_TYPE_STRING;
					pair.value = ++p;
					if (!(p = strchr(p, '\"'))) continue;
					*q = 0;
					*p = 0;
					up->jsonPairCount++;
					up->json = realloc(up->json, sizeof(NameValuePair) * up->jsonPairCount);
					up->json[i].name = malloc(strlen(keybase) + strlen(pair.name) + 1);
					strcpy(up->json[i].name, keybase);
					strcat(up->json[i].name, pair.name);
					up->json[i].value = strdup(pair.value);
					up->json[i].type = JSON_TYPE_STRING;
					*q = '\"';
					*p = '\"';
					p++;
				}
				else if (*p == '{') {
					// object
					*q = 0;
					if (strlen(keybase) + strlen(pair.name) + 1 < sizeof(keybase)) {
						strcat(keybase, pair.name);
						strcat(keybase, ".");
					}
					*q = '\"';
				}
				else if (*p == '[') {
					// array
					p++;
				}
				else {
					pair.value = p;
					if (!strncmp(p, "true", 4)) {
						p += 4;
						if (isalpha((int)*p) || isdigit((int)*p)) continue;
						pair.type = JSON_TYPE_BOOLEAN;
					}
					else if (!strncmp(p, "false", 5)) {
						p += 5;
						if (isalpha((int)*p) || isdigit((int)*p)) continue;
						pair.type = JSON_TYPE_BOOLEAN;
					}
					else {
						pair.type = JSON_TYPE_DECIMAL;
						while (*(++p) == '.' || *p == '-' || isdigit((int)*p));
					}
					char c = *p;
					*q = 0;
					*p = 0;
					up->jsonPairCount++;
					up->json = realloc(up->json, sizeof(NameValuePair) * up->jsonPairCount);
					up->json[i].name = malloc(strlen(keybase) + strlen(pair.name) + 1);
					strcpy(up->json[i].name, keybase);
					strcat(up->json[i].name, pair.name);
					up->json[i].value = strdup(pair.value);
					up->json[i].type = JSON_TYPE_DECIMAL;
					*q = '\"';
					*p = c;
				}
			}
			break;
		default:
			p++;
			break;
		}
	}
	return up->jsonPairCount;
}