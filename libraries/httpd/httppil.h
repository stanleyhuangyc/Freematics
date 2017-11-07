/******************************************************************************
* MiniWeb platform independent layer header file
* Distributed under BSD license
******************************************************************************/

#ifndef _HTTPPIL_H_
#define _HTTPPIL_H_

#ifdef SYS_MINGW
#ifndef WIN32
#define WIN32
#endif
#endif

#if defined(WIN32)
#include <windows.h>
#include <io.h>

#elif defined(ESP32) || defined(ESP8266)

#include <lwip/err.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include "string.h"

#else

#include <stdlib.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/select.h>
#include <netdb.h>
#include <stdint.h>

#if !defined(O_BINARY)
#define O_BINARY 0
#endif
#endif

#if defined(WIN32)

#define ssize_t size_t
#define socklen_t int
#define open _open
#define read _read
#define write _write
#define close _close
#define lseek _lseek
#define strdup _strdup
#define dup2 _dup2
#define dup _dup
#define pipe _pipe
#define spawnvpe _spawnvpe
#define spawnvp _spawnvp
#define atoll _atoi64

#else

#ifndef ARDUINO
#define closesocket close
#endif

#ifndef MAX_PATH
#define MAX_PATH 256
#endif
#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

typedef int SOCKET;
typedef unsigned int DWORD;
typedef unsigned short int WORD;
typedef unsigned char BYTE;
typedef int BOOL;
#endif
typedef unsigned char OCTET;

#if defined(_WIN32_WCE) || defined(WIN32)
#define msleep(ms) (Sleep(ms))
#else
#define msleep(ms) (usleep(ms<<10))
#endif

#ifdef __cplusplus
extern "C" {
#endif

int InitSocket();
void UninitSocket();
char *GetTimeString();
int ReadDir(const char* pchDir, char* pchFileNameBuf);
int IsFileExist(const char* filename);
int IsDir(const char* pchName);

#ifndef WIN32
uint32_t GetTickCount();
uint64_t GetTickCount64();
#endif
#ifdef __cplusplus
}
#endif

#endif
