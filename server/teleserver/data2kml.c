/*************************************************************************
* Data2Kml - Converting OBD/GPS data to KML
* Distributed under GPL v3.0 license
* (c)2013 Written by Stanley Huang
*************************************************************************/

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "logdata.h"
#include "data2kml.h"

uint16_t hex2uint16(const char *p);
int ishex(char c);

void WriteKMLData(KML_DATA* kd, uint32_t timestamp, uint16_t pid, float value[])
{
	// in the case timestamp overflowed or device reset
	if (timestamp + kd->tsOffset < kd->cur.timestamp && timestamp < 120000) {
		kd->tsOffset = kd->cur.timestamp;
	}
	timestamp += kd->tsOffset;
	if (kd->cur.timestamp != timestamp && kd->cur.time != kd->last.time && (kd->cur.flags & (FLAG_HAVE_LAT | FLAG_HAVE_LNG)) == (FLAG_HAVE_LAT | FLAG_HAVE_LNG)) do {
		// look for correct position to insert data
		DATASET* lastpd = 0;
		for (DATASET* pd = kd->data; pd && pd->timestamp <= kd->cur.timestamp; pd = pd->next) {
			lastpd = pd;
		}
		// filter out duplicated data
		if (lastpd && lastpd->timestamp == kd->cur.timestamp) {
			kd->last = kd->cur;
			break;
		}
		// filter out invalid coordinates
		if (kd->last.timestamp) {
			if (kd->cur.lat - kd->last.lat > 1 || kd->cur.lat - kd->last.lat < -1 || kd->cur.lng - kd->last.lng > 1 || kd->cur.lng - kd->last.lng < -1) {
				break;
			}
			// calculate boundaries
			if (kd->cur.lat < kd->bounds[0].lat)
				kd->bounds[0].lat = kd->cur.lat;
			else if (kd->cur.lat > kd->bounds[1].lat)
				kd->bounds[1].lat = kd->cur.lat;
			if (kd->cur.lng < kd->bounds[0].lng)
				kd->bounds[0].lng = kd->cur.lng;
			else if (kd->cur.lng > kd->bounds[1].lng)
				kd->bounds[1].lng = kd->cur.lng;
		}
		else {
			// first data
			kd->bounds[1].lat = kd->bounds[0].lat = kd->cur.lat;
			kd->bounds[1].lng = kd->bounds[0].lng = kd->cur.lng;
		}
	
		DATASET* newdata = malloc(sizeof(DATASET));
		memcpy(newdata, &kd->cur, sizeof(DATASET));
		if (newdata->pidCount) {
			newdata->pidData = malloc(newdata->pidCount * sizeof(float));
			memcpy(newdata->pidData, kd->cur.pidData, newdata->pidCount * sizeof(float));
		}

		if (!lastpd) {
			if (kd->data) {
				newdata->next = kd->data;
				kd->data = newdata;
			}
			else {
				kd->data = newdata;
			}
		}
		else if (lastpd->next) {
			// inserting
			newdata->next = lastpd->next;
			lastpd->next = newdata;
		} else {
			lastpd->next = newdata;
		}
		kd->datacount++;

		// calculate distance between two points
		if (kd->last.lat || kd->last.lng) {
			float a = (kd->last.lat - kd->cur.lat) * 100000;
			float b = (kd->last.lng - kd->cur.lng) * 100000;
			float distance = sqrtf(a * a + b * b);;
			if (distance >= 10000) {
				// invalid coordinates
				return;
			}
			kd->distance += distance;
		}

		fprintf(kd->fp, "<when>");
		if (kd->cur.date) {
			fprintf(kd->fp, "%04u-%02u-%02u", 2000 + (kd->cur.date % 100), (kd->cur.date / 100) % 100, kd->cur.date / 10000);
		}
		else {
			time_t yesterday = time(0) - 86400;
			struct tm* btm = localtime(&yesterday);
			fprintf(kd->fp, "%04d-%02d-%02d", 1900 + btm->tm_year, btm->tm_mon + 1, btm->tm_mday);
		}

		if (kd->cur.time) {
			fprintf(kd->fp, "T%02u:%02u:%02u", kd->cur.time / 1000000, (kd->cur.time / 10000) % 100, (kd->cur.time / 100) % 100);
			if (kd->cur.time % 100) {
				fprintf(kd->fp, ".%02u0Z", kd->cur.time % 100);
			}
		}
		fprintf(kd->fp, "</when>");
		fprintf(kd->fp, "<gx:coord>%f %f %d</gx:coord>", kd->cur.lng, kd->cur.lat, (int)kd->cur.alt);

		// keep as last coordinates
		kd->last = kd->cur;
	} while (0);
	kd->cur.timestamp = timestamp;
	switch (pid) {
	case PID_GPS_LATITUDE:
		kd->cur.lat = value[0];
		kd->cur.flags |= FLAG_HAVE_LAT;
		break;
	case PID_GPS_LONGITUDE:
		kd->cur.lng = value[0];
		kd->cur.flags |= FLAG_HAVE_LNG;
		break;
	case PID_GPS_ALTITUDE:
		kd->cur.alt = value[0];
		break;
	case PID_GPS_SPEED:
		kd->cur.speed = value[0];
		break;
	case PID_ACC:
		kd->cur.acc[0] = (int16_t)value[0];
		kd->cur.acc[1] = (int16_t)value[1];
		kd->cur.acc[2] = (int16_t)value[2];
		break;
	case PID_GPS_DATE:
		kd->cur.date = (uint32_t)value[0];
		kd->cur.timestamp = kd->cur.timestamp;
		break;
	case PID_GPS_TIME:
		kd->cur.time = (uint32_t)value[0];
		kd->cur.timestamp = kd->cur.timestamp;
		break;
	case PID_BATTERY_VOLTAGE:
		kd->cur.battery = (uint16_t)value[0];
		break;
	default:
		if (pid >= 0x100) {
			pid -= 0x100;
			if (!kd->cur.pidData) {
				kd->cur.pidData = calloc(pid + 1, sizeof(float));
				kd->cur.pidCount = pid;
			} else if (pid > kd->cur.pidCount) {
				kd->cur.pidData = realloc(kd->cur.pidData, (pid + 1) * sizeof(float));
				kd->cur.pidCount = pid;
			}
			kd->cur.pidData[pid] = value[0];
		}
	}
}

void WriteExtData(KML_DATA* kd, int pid)
{
	if (pid < 0x100) return;
	fprintf(kd->fp, "<gx:SimpleArrayData name=\"%X\">", pid);
	pid -= 0x100;
	for (DATASET* pd = kd->data; pd; pd = pd->next) {
		if (pid < pd->pidCount) {
			fprintf(kd->fp, "<gx:value>%d</gx:value>", (int)pd->pidData[pid]);
		}
		else {
			fprintf(kd->fp, "<gx:value/>");
		}
	}
	fprintf(kd->fp, "</gx:SimpleArrayData>");
}

void WriteKMLTail(KML_DATA* kd)
{
	DATASET* pd;
	printf("Generating extended data\n");

	fprintf(kd->fp, "<ExtendedData><SchemaData schemaUrl=\"#schema\">");

	WriteExtData(kd, PID_SPEED);
	WriteExtData(kd, PID_RPM);
	WriteExtData(kd, PID_COOLANT_TEMP);
	WriteExtData(kd, PID_ENGINE_LOAD);
	WriteExtData(kd, PID_THROTTLE);

	fprintf(kd->fp, "<gx:SimpleArrayData name=\"%X\">", PID_BATTERY_VOLTAGE);
	for (pd = kd->data; pd; pd = pd->next) {
		fprintf(kd->fp, "<gx:value>%.1f</gx:value>", (float)pd->battery / 100);
	}
	fprintf(kd->fp, "</gx:SimpleArrayData>");

	fprintf(kd->fp, "<gx:SimpleArrayData name=\"%X\">", PID_GPS_ALTITUDE);
	for (pd = kd->data; pd; pd = pd->next) {
		fprintf(kd->fp, "<gx:value>%d</gx:value>", (int)pd->alt);
	}
	fprintf(kd->fp, "</gx:SimpleArrayData>");

	fprintf(kd->fp, "<gx:SimpleArrayData name=\"%X\">", PID_ACC);
	for (pd = kd->data; pd; pd = pd->next) {
		fprintf(kd->fp, "<gx:value>%d/%d/%d</gx:value>", pd->acc[0], pd->acc[1], pd->acc[2]);
	}
	fprintf(kd->fp, "</gx:SimpleArrayData>");

	fprintf(kd->fp, "</SchemaData></ExtendedData>\r\n</gx:Track></Placemark>");

#if 0
	n = 0;
	for (i = 0; i < kd->datacount - 1; i++) {
		float g = 0;
		if (kd->dataset[i].speed < 25) {
			continue;
		}
		if (kd->dataset[i].throttle > lowThrottle + 2) {
			// throttle pedal is still down
			continue;
		}
		if (kd->dataset[i + 1].speed < kd->dataset[i].speed)
			g = (((float)kd->dataset[i + 1].speed - kd->dataset[i].speed) * 1000 / (kd->dataset[i + 1].timestamp - kd->dataset[i].timestamp) / 3.6f) / 9.8f;
		else
			continue;

		if (g <= -0.2f) {
			uint32_t t = kd->dataset[i].timestamp + 1000;
			while (kd->dataset[++i].timestamp < t);
			n++;
			fprintf(kd->fp, "<Placemark><name>#%d %u:%02u</name>", n, kd->dataset[i].timestamp / 60000, (kd->dataset[i].timestamp / 1000) % 60);
			fprintf(kd->fp, "<styleUrl>#brakepoint</styleUrl><Point><coordinates>%f,%f</coordinates></Point>", kd->dataset[i].lng, kd->dataset[i].lat);
			fprintf(kd->fp, "<ExtendedData>");
			fprintf(kd->fp, "<Data name=\"Speed\"><value>%d</value></Data>", kd->dataset[i].speed);
			fprintf(kd->fp, "<Data name=\"RPM\"><value>%d</value></Data>", kd->dataset[i].rpm);
			fprintf(kd->fp, "<Data name=\"ACC\"><value>%.2fG</value></Data>", g);
			fprintf(kd->fp, "</ExtendedData>");
			fprintf(kd->fp, "</Placemark>\r\n");
		}
	}
#endif
	fprintf(kd->fp, "</Folder></Document></kml>");
	if (kd->fp) fclose(kd->fp);
}

void CleanupKML(KML_DATA* kd)
{
	for (DATASET* pd = kd->data; pd; ) {
		DATASET* next = pd->next;
		if (pd->pidData) free(pd->pidData);
		free(pd);
		pd = next;
	}
	kd->cur.next = 0;
}

int ConvertToKML(KML_DATA* kd, FILE* fp, const char* kmlfile, uint32_t startpos, uint32_t endpos)
{
	int pid;
	uint32_t ts = 0;
	char line[1024];

	if (!kd) return -1;
	kd->fp = fopen(kmlfile, "wb");
	if (!fp || !kd->fp) return -1;
	fprintf(stderr, "Opened %s for writing\n", kmlfile);


	FILE* fpHeader = fopen("config/kmlstyle.tpl", "rb");
	if (fpHeader) {
		for (;;) {
			int n = fread(line, 1, sizeof(line), fpHeader);
			if (n <= 0) break;
			fwrite(line, 1, n, kd->fp);
		}
		fclose(fpHeader);
	}

	//write UTF-8 file mark
	//fprintf(kd.fp, "%c%c%c", 0xEF, 0xBB, 0xBF);

	fprintf(kd->fp, "<gx:Track>");

	while (fscanf(fp, "%1024s\n", line) > 0) {
		for (char* p = strtok(line, ","); p; p = strtok(0, ",")) {
			if (!ishex(*p)) break;
			pid = hex2uint16(p);
			if (!(p = strchr(p, ':'))) break;
			float value[3] = { 0 };
			for (int n = 0; n < 3; n++) {
				value[n] = (float)atof(++p);
				if (!(p = strchr(p, ';'))) break;
			}
			if (pid == 0) ts = (uint32_t)value[0];
			if (ts < startpos) {
				continue;
			}
			else if (endpos && ts > endpos) {
				break;
			}
			if (pid) {
				kd->pidMap[pid] = 1;
				WriteKMLData(kd, ts, pid, value);
			}
		}
		if (endpos && ts > endpos)
			break;
	}

	WriteKMLTail(kd);
	return kd->datacount;
}
