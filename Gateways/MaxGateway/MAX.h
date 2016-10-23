#ifndef _MAX_h
#define _MAX_h

#define MAX_BUFLEN 30
#define MAX_MAXDATALEN (MAX_BUFLEN-1)
#define MAX_overlength 2

extern uint8_t MAX_tracePackets;

extern volatile uint32_t MAX_lastRXTXmillis;
extern volatile uint8_t MAX_buf[MAX_BUFLEN];  // recv/xmit buf, 
extern volatile uint8_t MAX_rxfill;     // number of data bytes in MAX_buf
#define MAX_len MAX_buf[0]
extern volatile uint8_t MAX_rssi;
extern volatile uint16_t MAX_crc;

void MAX_writeReg(uint8_t addr, uint8_t value);
uint8_t MAX_readReg(uint8_t addr);

bool MAX_canSend();
bool MAX_sending();
void MAX_sleep(bool off);
void MAX_Initialize();
uint16_t MAX_recvDone();

void MAX_sendStart(bool fast, const uint8_t* header, uint8_t headerLength, const uint8_t* payload, uint8_t payloadLength);
void MAX_send(bool fast, const uint8_t* header, uint8_t headerLength, const uint8_t* payload, uint8_t payloadLength);
void MAX_send(bool fast, uint8_t msgId, uint8_t flags, uint8_t cmd, uint32_t src, uint32_t dest, uint8_t groupId, const uint8_t* payload, uint8_t payloadLength);
#endif

