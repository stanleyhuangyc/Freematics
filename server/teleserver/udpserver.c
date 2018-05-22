/******************************************************************************
* Freematics Hub Server
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
#include <sys/stat.h>
#include "httpd.h"
#include "teleserver.h"

extern char serverKey[];

//////////////////////////////////////////////////////////////////////////
// callback from the web server whenever it recevies UDP data
//////////////////////////////////////////////////////////////////////////

int verifyChecksum(char* data)
{
	uint8_t sum = 0;
	char *p = strrchr(data, '*');
	if (!p) return 0;
	for (char *s = data; s < p; s++) sum += *s;
	if (hex2uint8(p + 1) == sum) {
		*p = 0; // strip checksum
		return 1;
	}
	else {
		return 0;
	}
}

int addChecksump(char* data)
{
	uint8_t sum = 0;
	char *s;
	for (s = data; *s; s++) sum += *s;
	s += sprintf(s, "*%X", sum);
	return (int)(s - data);
}

int incomingUDPCallback(void* _hp)
{
	HttpParam* hp = (HttpParam*)_hp;
	struct sockaddr_in cliaddr;
	socklen_t socklen = sizeof(cliaddr);
	char buf[4096];
	int recv;
	char* hostaddr;

	if ((recv = recvfrom(hp->udpSocket, buf, sizeof(buf) - 1, 0, (struct sockaddr *)&cliaddr, &socklen)) <= 0)
		return -1;


	/*
	Data format:
	<ID>#<timestamp>:<pid>=<data>[$<checksum>]
	*/

	buf[recv] = 0;
	hostaddr = inet_ntoa(cliaddr.sin_addr);
	fprintf(stderr, "%u bytes from %s | ", recv, hostaddr);

	// validate checksum
	if (!verifyChecksum(buf)) {
		fprintf(stderr, "UDP data checksum mismatch\n%s\n", buf);
		return -1;
	}

	int success = 0;
	CHANNEL_DATA* pld = 0;
	int id;
	char *msg;
	char *data;
	char *s;

	// validate header
	data = strchr(buf, '#');
	if (!data) {
		// invalid header
		fprintf(stderr, "Invalid data received\n");
		return -1;
	}
	data++;

	// parse feed ID
	id = hex2uint16(buf);

	uint32_t deviceTick = 0;
	uint32_t token = 0;
	int16_t eventID = 0;

	if (s = strstr(data, "EV=")) {
		eventID = atoi(s + 3);
	}
	s = strstr(data, "TS=");
	if (s) {
		deviceTick = atol(s + 3);
	}
	s = strstr(data, "TK=");
	if (s) {
		token = atol(s + 3);
	}
	msg = strstr(data, "MSG=");
	if (msg) msg += 4;

	//fprintf(stderr, "Channel ID:%u Event ID:%u\n", id, eventID);
	if (eventID == EVENT_LOGIN) {
		int dtcCount = 0;
		s = strstr(data, "DTC=");
		if (s) {
			dtcCount = atoi(s + 4);
		}
		char *vin = strstr(data, "VIN=");
		if (!vin || !checkVIN(vin + 4)) {
			fprintf(stderr, "Invalid VIN\n");
			return 0;
		}
		vin += 4;

		char *key = strstr(data, "SK=");
		if (key) {
			key += 3;
			s = strchr(key, ',');
			if (s) *s = 0;
		}
		s = strchr(vin, ',');
		if (s) *s = 0;

		pld = findChannelByVin(vin);
		if (!pld) {
			pld = findEmptyChannel();
			if (!pld) {
				fprintf(getLogFile(), "No more channel");
				return 0;
			}
			else {
				strncpy(pld->vin, vin, sizeof(pld->vin) - 1);
			}
		}

		// TODO: also check timed out device
		if ((pld->flags & FLAG_RUNNING)) {
			if (memcmp(&pld->udpPeer, &cliaddr, sizeof(cliaddr))) {
				// different peer trys to connect with same credential
				return -2;
			}
		} else if (*serverKey) {
			// match server key
			if (key && !strcmp(serverKey, key)) {
				memcpy(&pld->udpPeer, &cliaddr, sizeof(cliaddr));
			}
			else {
				memset(&pld->udpPeer, 0, sizeof(pld->udpPeer));
			}
		}
		else {
			// always accept
			memcpy(&pld->udpPeer, &cliaddr, sizeof(cliaddr));
		}

		pld->flags |= FLAG_RUNNING;
		if (deviceTick) pld->deviceTick = deviceTick;
		pld->serverTick = GetTickCount64();
		pld->dtcCount = dtcCount;
		if (id == 0) {
			// clear cache
			pld->cacheReadPos = 0;
			pld->cacheWritePos = 0;
			// clear instance data cache
			memset(pld->mode, 0, sizeof(pld->mode));
			// clear stats
			pld->dataReceived = 0;
			pld->recvCount = 0;
			pld->elapsedTime = 0;
			pld->topSpeed = 0;
			printf("DEVICE LOGIN, SERVER KEY: %s VIN:%s ID:%u\r\n", key, vin, pld->id);
			SaveChannels();
			pld->startServerTick = GetTickCount64();
		}
		else {
			// reconnect
			pld = findChannelByID(id);
			fprintf(stderr, "DEVICE RECONNECTED, SERVER KEY: %s VIN:%s ID:%u\r\n", key, vin, pld->id);
		}
	}
	else {
		pld = findChannelByID(id);
	}

	if (!pld) return -1;

	pld->dataReceived += recv;

	// check if authorized peer
	if (memcmp(&cliaddr, &pld->udpPeer, sizeof(cliaddr))) {
		// unauthorized
		fprintf(stderr, "Unauthorized peer\n");
		return -1;
	}

	if (eventID == EVENT_ACK) {
		// pending command executed
		if (msg) {
			for (int i = 0; i < MAX_PENDING_COMMANDS; i++) {
				COMMAND_BLOCK *cmd = pld->cmd + i;
				if (cmd->token && cmd->token == token) {
					cmd->flags |= CMD_FLAG_RESPONDED;
					cmd->elapsed = (uint16_t)(pld->serverTick - cmd->tick);
					// store received message
					int len = strlen(msg);
					if (cmd->message) {
						if (len > cmd->len) {
							free(cmd->message);
							cmd->message = malloc(len + 1);
						}
					}
					else {
						cmd->message = malloc(len + 1);
					}
					cmd->len = len;
					strcpy(cmd->message, msg);
					break;
				}
			}
		}
		// no response needed for ACK
		return 0;
	}
	processPayload(data, pld);

	if (eventID != EVENT_ACK) {
		if (eventID == 0) {
			if (pld->deviceTick - pld->syncTick >= SYNC_INTERVAL * 1000) {
				// send sync event
				pld->syncTick = pld->deviceTick;
				eventID = EVENT_SYNC;
			}
			else {
				// no response if no sync is required
				return 0;
			}
		}
		// generate response
		int len = sprintf(buf, "%X#EV=%u,RX=%u,TM=%lu", pld->id, eventID, pld->recvCount, (unsigned long)time(0));
		switch (eventID) {
		case EVENT_LOGOUT:
			// device is going offline
			pld->flags &= ~FLAG_RUNNING;
			break;
		}
		// send UDP response
		len = addChecksump(buf);
		if (sendto(hp->udpSocket, buf, len, 0, (struct sockaddr *)&cliaddr, socklen) == len)
			fprintf(stderr, "Reply sent:%s\n", buf);
		else
			fprintf(stderr, "Reply unsent\n");
	}
	return 0;
}

uint32_t issueCommand(HttpParam* hp, CHANNEL_DATA *pld, const char* cmd, uint32_t token)
{
	if (token == 0) token = ++pld->cmdCount;
	char buf[128];
	sprintf(buf, "%X#EV=%u,TK=%u,CMD=%s", pld->id, EVENT_COMMAND, token, cmd);
	int len = addChecksump(buf);
	socklen_t socklen = sizeof(struct sockaddr);
	pld->serverTick = GetTickCount64();
	if (sendto(hp->udpSocket, buf, len, 0, (struct sockaddr *)&pld->udpPeer, socklen) == len) {
		fprintf(stderr, "Command sent: %s (%u)\n", cmd, token);
		// find out checked pending command
		COMMAND_BLOCK *cmd = 0;
		for (int i = 0; i < MAX_PENDING_COMMANDS; i++) {
			cmd = pld->cmd + i;
			if (cmd->token == 0 || (cmd->flags & CMD_FLAG_CHECKED)) {
				break;
			}
			cmd = 0;
		}
		if (!cmd) {
			// find out oldest pending command regardless of its status
			unsigned int maxElapsed = 0;
			for (int i = 0; i < MAX_PENDING_COMMANDS; i++) {
				unsigned int elapsed = (unsigned int)(pld->serverTick - pld->cmd[i].tick);
				if (elapsed >= maxElapsed) {
					cmd = pld->cmd + i;
					maxElapsed = elapsed;
				}
			}
		}
		if (cmd) {
			// place sent command in pending command list
			cmd->token = token;
			cmd->tick = pld->serverTick;
			cmd->flags = 0;
		}
		return token;
	}
	else {
		fprintf(stderr, "Command unsent\n");
		return 0;
	}
}
