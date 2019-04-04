/******************************************************************************
* MiniWeb http implementation internal header file
* Distributed under BSD license
******************************************************************************/

#ifndef _HTTPINT_H_
#define _HTTPINT_H_

/////////////////////////////////////////////////////////////////////////////
// defines
/////////////////////////////////////////////////////////////////////////////

// HTTP messages/part messages
#ifndef HTTP_SERVER_NAME
#define HTTP_SERVER_NAME "MiniWeb"
#endif
#define HTTP200_HEADER "%s %d %s\r\nServer: %s\r\nCache-control: no-cache\r\nConnection: %s\r\n"
#define HTTP200_HDR_EST_SIZE ((sizeof(HTTP200_HEADER)+256)&(-4))
#define HTTP403_HEADER "HTTP/1.1 403 Forbidden\r\nContent-Length: 0\r\n\r\n"
#define HTTP404_HEADER "HTTP/1.1 404 Not Found\r\nServer: %s\r\nContent-Length: %d\r\nContent-Type: text/html\r\n\r\n"
#define HTTP404_BODY "<html><head><title>404 Not Found</title></head><body><h1>Not Found</h1><p>The requested URL has no content.</p></body></html>"
#define HTTPBODY_REDIRECT "<html><head><meta http-equiv=\"refresh\" content=\"0; URL=%s\"></head><body></body></html>"
#define HTTP301_HEADER "HTTP/1.1 301 Moved Permanently\r\nServer: %s\r\nLocation: %s\r\nContent-Length: 0\r\n\r\n"
#define HTTP401_HEADER "HTTP/1.1 401 Authorization Required\r\nWWW-Authenticate: Basic realm=\"%s\"\r\nContent-Length: %d\r\nContent-Type: text/html\r\n\r\n"
//#define HTTP401_BODY "<html><head><title>401 Authorization Required</title></head><body><h1>Authorization Required</h1><p>This server could not verify that you are authorized to access the resource requested</p></body></html>"
#define HTTP_CONTENTLENGTH "Content-Length:"
#define HTTP_MULTIPARTHEADER "multipart/form-data"
#define HTTP_MULTIPARTCONTENT "Content-Disposition: form-data; name="
#define HTTP_MULTIPARTBOUNDARY "boundary="
#define HTTP_FILENAME "filename="
#define HTTP_HEADER_END "\r\n\r\n"
#define HTTP_SUBST_PATTERN (WORD)(('$' << 8) + '$')

// Define file extensions
#define FILEEXT_HTM DEFDWORD('H','T','M',0)
#define FILEEXT_XML DEFDWORD('X','M','L',0)
#define FILEEXT_XSL DEFDWORD('X','S','L',0)
#define FILEEXT_TEXT DEFDWORD('T','X','T',0)
#define FILEEXT_XUL DEFDWORD('X','U','L',0)
#define FILEEXT_GIF DEFDWORD('G','I','F',0)
#define FILEEXT_JPG DEFDWORD('J','P','G',0)
#define FILEEXT_PNG DEFDWORD('P','N','G',0)
#define FILEEXT_CSS DEFDWORD('C','S','S',0)
#define FILEEXT_JS DEFDWORD('J','S',0,0)
#define FILEEXT_SWF DEFDWORD('S','W','F',0)
#define FILEEXT_HTML DEFDWORD('H','T','M','L')
#define FILEEXT_MPG DEFDWORD('M','P','G',0)
#define FILEEXT_MPEG DEFDWORD('M','P','E','G')
#define FILEEXT_MPA DEFDWORD('M','P','3' - 32,0)
#define FILEEXT_AVI DEFDWORD('A','V','I',0)
#define FILEEXT_MP4 DEFDWORD('M','P','4' - 32,0)
#define FILEEXT_MOV DEFDWORD('M','O','V',0)
#define FILEEXT_FLV DEFDWORD('F','L','V',0)
#define FILEEXT_3GP DEFDWORD('3' - 32, 'G','P',0)
#define FILEEXT_ASF DEFDWORD('A','S','F',0)
#define FILEEXT_264 DEFDWORD('2' - 32, '6' - 32, '4' - 32, 0)
#define FILEEXT_TS DEFDWORD('T', 'S', 0, 0)
#define FILEEXT_M3U8 DEFDWORD('M', '3' - 32, 'U', '8' - 32)
#define FILEEXT_SDP DEFDWORD('S', 'D', 'P', 0)

// Settings for http server
#ifndef ARDUINO
#define HTTP_EXPIRATION_TIME (120/*secs*/)
#define HTTP_KEEPALIVE_TIME (15/*secs*/)
#define HTTP_KEEPALIVE_MAX (1000 /*requests*/)
#define MAX_REQUEST_PATH_LEN (512/*bytes*/)
#else
#define HTTP_EXPIRATION_TIME (30/*secs*/)
#define HTTP_KEEPALIVE_TIME (15/*secs*/)
#define HTTP_KEEPALIVE_MAX (100 /*requests*/)
#define MAX_REQUEST_PATH_LEN (128/*bytes*/)
#define MAX_OPEN_FILES 16
#endif
#define MAX_RECV_RETRIES (3/*times*/)
#define HTTPAUTHTIMEOUT   (60/*secs*/)
#define HTTPSUBSTEXPANSION (0/*bytes*/)
#define HTTPHEADERSIZE (512/*bytes*/)
#define HTTPMAXRECVBUFFER HTTP_BUFFER_SIZE
#define HTTPUPLOAD_CHUNKSIZE (HTTPMAXRECVBUFFER / 2/*bytes*/)
#define MAX_REQUEST_SIZE (2*1024 /*bytes*/)

#define SLASH '/'

#define LOG_INFO stdout
#define SYSLOG fprintf

/////////////////////////////////////////////////////////////////////////////
// local helper function prototypes
/////////////////////////////////////////////////////////////////////////////
SOCKET _mwAcceptSocket(HttpParam* hp, struct sockaddr_in *sinaddr);
void _mwDenySocket(HttpParam* hp,struct sockaddr_in *sinaddr);
int _mwProcessReadSocket(HttpParam* hp, HttpSocket* phsSocket);
int _mwProcessWriteSocket(HttpParam *hp, HttpSocket* phsSocket);
void _mwCloseSocket(HttpParam* hp, HttpSocket* phsSocket);
int _mwStartSendFile(HttpParam* hp, HttpSocket* phsSocket);
int _mwSendFileChunk(HttpParam *hp, HttpSocket* phsSocket);
char* _mwStrStrNoCase(char* pchHaystack, char* pchNeedle);
void _mwRedirect(HttpSocket* phsSocket, char* pchFilename);
int _mwSendRawDataChunk(HttpParam *hp, HttpSocket* phsSocket);
int _mwStartSendRawData(HttpParam *hp, HttpSocket* phsSocket);
int _mwGetToken(char* pchBuffer,int iTokenNumber,char** ppchToken);
char _mwDecodeCharacter(char* pchEncodedChar);
int _mwLoadFileChunk(HttpParam *hp, HttpSocket* phsSocket);
BOOL _mwCheckAuthentication(HttpParam *hp, HttpSocket* phsSocket);
int _GetContentType(char *pchFilename);
int _mwCheckAccess(HttpSocket* phsSocket);
int _mwGetContentType(char *pchExtname);
int _mwSendHttpHeader(HttpSocket* phsSocket);
char* _mwStrDword(char* pchHaystack, DWORD dwSub, DWORD dwCharMask);
SOCKET _mwStartListening(HttpParam* hp);
int _mwParseHttpHeader(HttpSocket* phsSocket);
int _mwStrCopy(char *dest, const char *src);
int _mwStrHeadMatch(char** pbuf1, const char* buf2);
void _mwSetSocketOpts(SOCKET socket);
void _mwSendErrorPage(SOCKET socket, const char* header, const char* body);
void _mwCloseAllConnections(HttpParam* hp);
void _mwFreeJSONPairs(UrlHandlerParam* up);
#endif
////////////////////////// END OF FILE //////////////////////////////////////
