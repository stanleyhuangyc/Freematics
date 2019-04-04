/******************************************************************************
* MiniWeb - a mini and high efficiency HTTP server implementation
* Distributed under BSD license
******************************************************************************/

#ifndef _HTTPAPI_H_
#define _HTTPAPI_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "httppil.h"

#define	FD_ISSET_BOOL(n, p)	((FD_ISSET(n,p)) >> ((n) % NFDBITS))

#ifndef min
#define min(x,y) (x>y?y:x)
#endif

#ifdef HTTP_DEBUG
#define DBG printf
#else
#define DBG
#endif
#define LOG_ERR 1

#define ASSERT
#define GETDWORD(ptrData) (*(DWORD*)(ptrData))
#define SETDWORD(ptrData,data) (*(DWORD*)(ptrData)=data)
#define GETWORD(ptrData) (*(WORD*)(ptrData))
#define SETWORD(ptrData,data) (*(WORD*)(ptrData)=data)
#ifndef BIG_ENDINE
#define DEFDWORD(char1,char2,char3,char4) ((char1)+((char2)<<8)+((char3)<<16)+((char4)<<24))
#define DEFWORD(char1,char2) (char1+(char2<<8))
#else
#define DEFDWORD(char1,char2,char3,char4) ((char4)+((char3)<<8)+((char2)<<16)+((char1)<<24))
#define DEFWORD(char1,char2) (char2+(char1<<8))
#endif

///////////////////////////////////////////////////////////////////////
// Public definitions
///////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
extern "C" {
#endif

// file types
typedef enum {
  HTTPFILETYPE_UNKNOWN = 0,
  HTTPFILETYPE_HTML,
  HTTPFILETYPE_XML,
  HTTPFILETYPE_TEXT,
  HTTPFILETYPE_XUL,
  HTTPFILETYPE_CSS,
  HTTPFILETYPE_JS,
  HTTPFILETYPE_PNG,
  HTTPFILETYPE_JPEG,
  HTTPFILETYPE_GIF,
  HTTPFILETYPE_SWF,
  HTTPFILETYPE_MPA,
  HTTPFILETYPE_MPEG,
  HTTPFILETYPE_AVI,
  HTTPFILETYPE_MP4,
  HTTPFILETYPE_MOV,
  HTTPFILETYPE_264,
  HTTPFILETYPE_FLV,
  HTTPFILETYPE_TS,
  HTTPFILETYPE_3GP,
  HTTPFILETYPE_ASF,
  HTTPFILETYPE_OCTET,
  HTTPFILETYPE_STREAM,
  HTTPFILETYPE_M3U8,
  HTTPFILETYPE_SDP,
  HTTPFILETYPE_HEX,
  HTTPFILETYPE_JSON,
} HttpFileType;

/////////////////////////////////////////////////////////////////////////////
// typedefs
/////////////////////////////////////////////////////////////////////////////

#define FLAG_REQUEST_GET		0x1
#define FLAG_REQUEST_POST		0x2
#define FLAG_HEADER_SENT		0x80
#define FLAG_CONN_CLOSE			0x100
#define FLAG_ABSOLUTE_PATH		0x200
#define FLAG_AUTHENTICATION		0x400
#define FLAG_MORE_CONTENT		0x800
#define FLAG_TO_FREE			0x1000
#define FLAG_CHUNK				0x2000
#define FLAG_CLOSE_CALLBACK     0x4000

#define FLAG_DATA_FILE		0x10000
#define FLAG_DATA_RAW		0x20000
#define FLAG_DATA_REDIRECT	0x80000
#define FLAG_DATA_STREAM	0x100000
#define FLAG_CUSTOM_HEADER	0x200000
#define FLAG_MULTIPART		0x400000

#define FLAG_RECEIVING		0x40000000
#define FLAG_SENDING		0x80000000

#define SETFLAG(hs,bit) (hs->flags|=(bit));
#define CLRFLAG(hs,bit) (hs->flags&=~(bit));
#define ISFLAGSET(hs,bit) (hs->flags&(bit))

typedef union {
	unsigned long laddr;
	unsigned short saddr[2];
	unsigned char caddr[4];
} IPADDR;

typedef struct {
	int iHttpVer;
	unsigned int startByte;
	char *pucPath;
	const char *pucReferer;
	char* pucHost;
	int headerSize;
	char* pucPayload;
	unsigned int payloadSize;
	int iCSeq;
	const char* pucTransport;
	const char* pucAuthInfo;
} HttpRequest;

typedef struct {
	int statusCode;
	int headerBytes;
	int sentBytes;
	unsigned int contentLength;
	HttpFileType fileType;
} HttpResponse;

typedef struct {
	char *name;
	char *value;
} HttpVariables;

// Callback function protos
typedef int (*PFN_UDP_CALLBACK)(void* hp);

typedef struct {
	uint32_t reqCount;
	size_t totalSentBytes;
	uint32_t authFailCount;
	uint16_t clientCount;
	uint16_t clientCountMax;
	uint16_t openedFileCount;
} HttpStats;

#ifndef ARDUINO
#define HTTP_BUFFER_SIZE (64*1024 /*bytes*/)
#define MAX_POST_PAYLOAD_SIZE (128*1024 /*bytes*/)
#define HTTP_MAX_CLIENTS_DEFAULT 32
#else
#define HTTP_BUFFER_SIZE (16*1024 /*bytes*/)
#define MAX_POST_PAYLOAD_SIZE (16*1024 /*bytes*/)
#define HTTP_MAX_CLIENTS_DEFAULT 16
#endif

// per connection/socket structure
typedef struct _HttpSocket{
	SOCKET socket;
	IPADDR ipAddr;
	HttpRequest request;
	HttpResponse response;
	char *pucData;
	uint32_t bufferSize;			// the size of buffer pucData pointing to
	uint32_t contentLength;
	FILE* fp;
	uint32_t flags;
	void* handler;				// http handler function address
	void* ptr;
	time_t tmExpirationTime;
	char* mimeType;
	char* buffer;
	uint16_t reqCount;
} HttpSocket;

typedef enum {
	JSON_TYPE_STRING = 0,
	JSON_TYPE_DECIMAL,
	JSON_TYPE_BOOLEAN,
} JSONValueType;

typedef struct {
	char* name;
	char* value;
	JSONValueType type;
} NameValuePair;

typedef struct {
	void* hp;
	HttpSocket* hs;
	const char *pucRequest;
	HttpVariables* pxVars;
	int iVarCount;
	char *pucHeader;
	char *pucBuffer;
	unsigned int bufSize;
	char *pucPayload;
	unsigned int payloadSize;
	unsigned int contentLength;
	HttpFileType contentType;
	NameValuePair* json;
	int jsonPairCount;
} UrlHandlerParam;

typedef int (*PFNURLCALLBACK)(UrlHandlerParam*);

typedef struct {
	const char* pchUrlPrefix;
	PFNURLCALLBACK pfnUrlHandler;
} UrlHandler;

#define AUTH_NO_NEED (0)
#define AUTH_SUCCESSED (1)
#define AUTH_REQUIRED (2)
#define AUTH_FAILED (-1)

#define MAX_AUTH_INFO_LEN 64

typedef struct {
	const char* pchUrlPrefix;
	const char* pchUsername;
	const char* pchPassword;
	const char* pchOtherInfo;
	char pchAuthString[MAX_AUTH_INFO_LEN];
} AuthHandler;

#define FLAG_DIR_LISTING 1
#define FLAG_DISABLE_RANGE 2

typedef struct _httpParam {
	HttpSocket* hsSocketQueue;				/* socket queue*/
	uint16_t maxClients;
	uint16_t maxClientsPerIP;
	unsigned int flags;
	SOCKET listenSocket;
	SOCKET udpSocket;
	uint16_t httpPort;
	uint16_t udpPort;
	char* pchWebPath;
	UrlHandler *pxUrlHandler;		/* pointer to URL handler array */
	AuthHandler *pxAuthHandler;     /* pointer to authorization handler array */
	// incoming udp callback
	PFN_UDP_CALLBACK pfnIncomingUDP;
	// misc
	DWORD dwAuthenticatedNode;
	time_t tmAuthExpireTime;
	HttpStats stats;
	DWORD hlBindIP;
	BOOL bKillWebserver;
	BOOL bWebserverRunning;
} HttpParam;

typedef struct {
	const char* pchRootPath;
	const char* pchHttpPath;
	char cFilePath[MAX_PATH];
	char* pchExt;
	int fTailSlash;
} HttpFilePath;

///////////////////////////////////////////////////////////////////////
// Return codes
///////////////////////////////////////////////////////////////////////
// for post callback
#define WEBPOST_OK                (0)
#define WEBPOST_AUTHENTICATED     (1)
#define WEBPOST_NOTAUTHENTICATED  (2)
#define WEBPOST_AUTHENTICATIONON  (3)
#define WEBPOST_AUTHENTICATIONOFF (4)

// for multipart file uploads
#define HTTPUPLOAD_MORECHUNKS     (0)
#define HTTPUPLOAD_FIRSTCHUNK     (1)
#define HTTPUPLOAD_LASTCHUNK      (2)

///////////////////////////////////////////////////////////////////////
// Public functions
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
// mwInitParam. Init the context structure with default values
///////////////////////////////////////////////////////////////////////
void mwInitParam(HttpParam* hp, int port, const char* pchWebPath);

///////////////////////////////////////////////////////////////////////
// mwServerStart. Startup the webserver
///////////////////////////////////////////////////////////////////////
int mwServerStart(HttpParam* hp);
void mwServerExit(HttpParam* hp);

///////////////////////////////////////////////////////////////////////
// mwHttpLoop. Enter webserver loop
///////////////////////////////////////////////////////////////////////
void mwHttpLoop(HttpParam *hp, uint32_t timeout);

///////////////////////////////////////////////////////////////////////
// mwServerShutdown. Shutdown the webserver (closes connections and
// releases resources)
///////////////////////////////////////////////////////////////////////
int mwServerShutdown(HttpParam* hp);

///////////////////////////////////////////////////////////////////////
// mwSetRcvBufSize. Change the TCP windows size of acceped sockets
///////////////////////////////////////////////////////////////////////
int mwSetRcvBufSize(WORD wSize);

///////////////////////////////////////////////////////////////////////
// Default subst, post and file-upload callback processing
///////////////////////////////////////////////////////////////////////
int mwGetHttpDateTime(time_t tm, char *buf, int bufsize);
int mwGetLocalFileName(HttpFilePath* hfp);
char* mwGetVarValue(HttpVariables* vars, const char *varname, const char *defval);
int mwGetVarValueInt(HttpVariables* vars, const char *varname, int defval);
unsigned int mwGetVarValueHex(HttpVariables* vars, const char *varname, unsigned int defval);
int64_t mwGetVarValueInt64(HttpVariables* vars, const char *varname);
float mwGetVarValueFloat(HttpVariables* vars, const char *varname);
int mwParseQueryString(UrlHandlerParam* up);
int mwGetContentType(const char *pchExtname);
void mwDecodeString(char* s);
NameValuePair* mwGetJSONData(UrlHandlerParam* up, const char* name);
int mwParseJSONString(UrlHandlerParam* up);

#ifdef __cplusplus
}
#endif

#endif // _HTTPAPI_H

////////////////////////// END OF FILE ////////////////////////////////
