/******************************************************************************
* MiniWeb platform independent layer
* Distributed under BSD license
******************************************************************************/

#include <stdio.h>
#include <time.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <ctype.h>
#include <fcntl.h>
#include "httppil.h"

#ifndef WIN32
#include <sys/types.h>
#include <unistd.h>
#ifndef ESP8266
#include <sys/time.h>
#include <dirent.h>
#endif
#endif

char *GetTimeString()
{
	static char buf[16];
	time_t tm=time(NULL);
	memcpy(buf,ctime(&tm)+4,15);
	buf[15]=0;
	return buf;
}

#if defined(ARDUINO)

int IsDir(const char* pchName)
{
	return 0;
}

int ReadDir(const char* pchDir, char* pchFileNameBuf)
{
	return 0;
}

int IsFileExist(const char* filename)
{
	return 0;
}

#else

int IsDir(const char* pchName)
{
#ifdef WIN32
	WIN32_FIND_DATA f;
	HANDLE hFind = FindFirstFile(pchName, &f);
	FindClose(hFind);
	if (hFind != INVALID_HANDLE_VALUE && (f.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
		return 1;
	return 0;
#else
	struct stat stDirInfo;
	if (stat( pchName, &stDirInfo) < 0) return 0;
	return S_ISDIR(stDirInfo.st_mode);
#endif
}

int ReadDir(const char* pchDir, char* pchFileNameBuf)
{
#if defined(WIN32)
	static HANDLE hFind=NULL;
	WIN32_FIND_DATA finddata;

	if (!pchFileNameBuf) {
		if (hFind) {
			FindClose(hFind);
			hFind=NULL;
		}
		return 0;
	}
	if (pchDir) {
		char *p;
		int len;
		if (!IsDir(pchDir)) return -1;
		if (hFind) FindClose(hFind);
		len = strlen(pchDir);
		p = malloc(len + 5);
		snprintf(p, len + 5, "%s\\*.*", pchDir);
		hFind=FindFirstFile(p,&finddata);
		free(p);
		if (hFind==INVALID_HANDLE_VALUE) {
			hFind=NULL;
			return -1;
		}
		strcpy(pchFileNameBuf,finddata.cFileName);
		return finddata.nFileSizeLow;
	}
	if (!hFind) return -1;
	if (!FindNextFile(hFind,&finddata)) {
		FindClose(hFind);
		hFind=NULL;
		return -1;
	}
	strcpy(pchFileNameBuf,finddata.cFileName);
#else
	static DIR *stDirIn=NULL;
	struct dirent *stFiles;

	if (!pchFileNameBuf) {
		if (stDirIn) {
			closedir(stDirIn);
			stDirIn=NULL;
		}
		return 0;
	}
	if (pchDir) {
		if (!IsDir(pchDir)) return -1;
		if (stDirIn) closedir(stDirIn);
		stDirIn = opendir( pchDir);
	}
	if (!stDirIn) return -1;
	stFiles = readdir(stDirIn);
	if (!stFiles) {
		closedir(stDirIn);
		stDirIn=NULL;
		return -1;
	}
	strcpy(pchFileNameBuf, stFiles->d_name);
#endif
	return 0;
}

int IsFileExist(const char* filename)
{
#ifdef WIN32
	WIN32_FIND_DATA f;
	HANDLE hFind = FindFirstFile(filename, &f);
	if (hFind == INVALID_HANDLE_VALUE)
		return 0;
	FindClose(hFind);
	return 1;
#else
	struct stat stat_ret;
	if (stat(filename, &stat_ret) != 0) return 0;

	return S_ISREG(stat_ret.st_mode);
#endif
}

#endif

#ifndef WIN32

#ifndef ARDUINO
uint32_t GetTickCount()
{
	struct timeval ts;
	gettimeofday(&ts,0);
	return ts.tv_sec * 1000 + ts.tv_usec / 1000;
}
#endif

uint64_t GetTickCount64()
{
	struct timeval ts;
	gettimeofday(&ts, 0);
	return (uint64_t)ts.tv_sec * 1000 + ts.tv_usec / 1000;
}
#endif
