/*************************************************************************
* Data2Kml - Converting OBD/GPS data to KML
* Distributed under GPL v3.0 license
* (c)2013 Written by Stanley Huang
*************************************************************************/

typedef struct {
	uint32_t timestamp;
	uint32_t date;
	uint32_t time;
	float lat;
	float lng;
	float alt;
	float speed;
	int16_t acc[3];
	uint16_t battery;
	float* pidData;
	int pidCount;
	uint32_t flags;
	void* next;
} DATASET;

#define FLAG_HAVE_LAT 0x1
#define FLAG_HAVE_LNG 0x2
#define FLAG_HAVE_DATE 0x4
#define FLAG_HAVE_TIME 0x8

typedef struct {
	uint32_t timestamp;
	uint32_t date;
	uint32_t time;
} TIME_DATA;

typedef struct {
	float lat;
	float lng;
} COORDS;

typedef struct {
	int state;
	FILE* fp;
	DATASET* data;
	int datacount;
	COORDS bounds[2];
	uint8_t pidMap[65536];
	float distance;
	DATASET cur;
	DATASET last;
	uint32_t tsOffset;
} KML_DATA;
