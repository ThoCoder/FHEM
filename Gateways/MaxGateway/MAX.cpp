//#define TRACE_RXTX 1
#include "Arduino.h"
#include <stdint.h>
#include <RF69_avr.h>
#include "MAX.h"
#include "CRC16.h"

uint8_t MAX_tracePackets = 0;
uint8_t MAX_culMessages = 1;
uint8_t MAX_cul868Compatibility = 0;
uint32_t MAX_ownAddress = 0x123456;

#define REG_FIFO            0x00
#define REG_OPMODE          0x01
#define REG_FRFMSB          0x07
#define REG_AFCFEI          0x1E
#define REG_RSSIVALUE       0x24
#define REG_DIOMAPPING1     0x25
#define REG_IRQFLAGS1       0x27
#define REG_IRQFLAGS2       0x28
#define REG_SYNCCONFIG      0x2E
#define REG_SYNCVALUE1      0x2F
#define REG_SYNCVALUE2      0x30
#define REG_NODEADRS        0x39
#define REG_PACKETCONFIG2   0x3D
#define REG_AESKEY1         0x3E

#define MODE_SLEEP          0x00
#define MODE_STANDBY        0x04
#define MODE_RECEIVER       0x10
#define MODE_TRANSMITTER    0x0C

#define IRQ1_MODEREADY      0x80
#define IRQ1_RXREADY        0x40

#define IRQ2_FIFOFULL       0x80
#define IRQ2_FIFONOTEMPTY   0x40
#define IRQ2_FIFOOVERRUN    0x10
#define IRQ2_PACKETSENT     0x08
#define IRQ2_PAYLOADREADY   0x04

#define DMAP1_PACKETSENT    0x00
#define DMAP1_PAYLOADREADY  0x40
#define DMAP1_SYNCADDRESS   0x80

#define AFC_CLEAR           0x02

// transceiver states, these determine what to do with each interrupt
enum { TXCRC1, TXCRC2, TXTAIL, TXDONE, TXIDLE, TXRECV };

volatile uint32_t MAX_lastRXTXmillis;
volatile uint32_t MAX_RXTXmillis0;
volatile uint8_t MAX_buf[MAX_BUFLEN];  // recv/xmit buf, 
volatile uint8_t MAX_rxfill;     // number of data bytes in MAX_buf
volatile int8_t MAX_rxstate;     // current transceiver state
volatile uint8_t MAX_rssi;
volatile uint16_t MAX_crc;

static const uint8_t pn9_table[] = {
	0xff, 0xe1, 0x1d, 0x9a, 0xed, 0x85, 0x33, 0x24,
	0xea, 0x7a, 0xd2, 0x39, 0x70, 0x97, 0x57, 0x0a,
	0x54, 0x7d, 0x2d, 0xd8, 0x6d, 0x0d, 0xba, 0x8f,
	0x67, 0x59, 0xc7, 0xa2, 0xbf, 0x34, 0xca, 0x18,
	0x30, 0x53, 0x93, 0xdf, 0x92, 0xec, 0xa7, 0x15,
	0x8a, 0xdc, 0xf4, 0x86, 0x55, 0x4e, 0x18, 0x21,
};

uint8_t control(uint8_t cmd, uint8_t val)
{
	PreventInterrupt irq0;
	return spiTransfer(cmd, val);
}

void MAX_writeReg(uint8_t addr, uint8_t value)
{
	control(addr | 0x80, value);
}

uint8_t MAX_readReg(uint8_t addr)
{
	return control(addr, 0);
}

void flushFifo()
{
	while (MAX_readReg(REG_IRQFLAGS2) & (IRQ2_FIFONOTEMPTY | IRQ2_FIFOOVERRUN))
		MAX_readReg(REG_FIFO);
}

void setMode(uint8_t mode)
{
	MAX_writeReg(REG_OPMODE, (MAX_readReg(REG_OPMODE) & 0xE3) | mode);
	// while ((readReg(REG_IRQFLAGS1) & IRQ1_MODEREADY) == 0)
	//     ;
}

bool MAX_canSend()
{
	if (MAX_rxstate == TXRECV && MAX_rxfill == 0)
	{
		MAX_rxstate = TXIDLE;
		setMode(MODE_STANDBY);
		return true;
	}
	return false;
}

bool MAX_sending()
{
	return MAX_rxstate < TXIDLE;
}

void MAX_sleep(bool off)
{
	setMode(off ? MODE_SLEEP : MODE_STANDBY);
	MAX_rxstate = TXIDLE;
}

uint16_t MAX_recvDone()
{
	switch (MAX_rxstate)
	{
	case TXIDLE:
#ifdef TRACE_RXTX
		printf_P(PSTR("d\n"));
#endif

		MAX_rxstate = TXRECV;
		MAX_rxfill = MAX_len = 0;
		MAX_crc = 0xFFFF;
		flushFifo();
		MAX_writeReg(REG_DIOMAPPING1, DMAP1_SYNCADDRESS);    // Interrupt trigger
		setMode(MODE_RECEIVER);
		MAX_writeReg(REG_AFCFEI, AFC_CLEAR);
		break;

	case TXRECV:

		if (MAX_rxfill > 0)
		{
			if (MAX_rxfill >= (MAX_len + 1 + MAX_overlength) || MAX_rxfill >= MAX_BUFLEN)
			{
				MAX_rxstate = TXIDLE;
				setMode(MODE_STANDBY);

				if (MAX_rxfill != (MAX_len + 1 + MAX_overlength))
					return 0;

#ifdef TRACE_RXTX
				printf_P(PSTR("D"));
#endif

				if (MAX_tracePackets != 0)
				{
					uint32_t t = millis();
					uint32_t dt = t - MAX_lastRXTXmillis;
					MAX_lastRXTXmillis = t;
					uint32_t dt0 = t - MAX_RXTXmillis0;

					printf_P(PSTR("%10ld::%6ld CRC:%04X L:%2d No:%02X F:%02X Cmd:%02X %02X%02X%02X -> %02X%02X%02X G:%02X (-%2d)  P="),
						dt0, dt, MAX_crc, MAX_len,
						MAX_buf[1], MAX_buf[2], MAX_buf[3],
						MAX_buf[4], MAX_buf[5], MAX_buf[6],
						MAX_buf[7], MAX_buf[8], MAX_buf[9], MAX_buf[10],
						MAX_rssi >> 1);
					for (int i = 11; i < MAX_rxfill - 2; i++)
					{
						printf_P(PSTR("%02X "), MAX_buf[i]);
					}
					printf_P(PSTR("\n"));
				}

				return 1;
			}
		}
		break;
	}

	return 0; // keep going, not done yet
}

void MAX_interrupt()
{
	if (MAX_rxstate == TXRECV)
	{
#ifdef TRACE_RXTX
		printf_P(PSTR("r"));
#endif
		// The following line attempts to stop further interrupts
		MAX_writeReg(REG_DIOMAPPING1, 0x40);  // Interrupt on PayloadReady
		MAX_rssi = MAX_readReg(REG_RSSIVALUE);
		IRQ_ENABLE; // allow nested interrupts from here on
		for (;;)
		{
			// busy loop, to get each data byte as soon as it comes in
			if (MAX_readReg(REG_IRQFLAGS2) & (IRQ2_FIFONOTEMPTY | IRQ2_FIFOOVERRUN))
			{
				uint8_t in = MAX_readReg(REG_FIFO) ^ pn9_table[MAX_rxfill];
#ifdef TRACE_RXTX
				if (MAX_rxfill == 0)
					printf_P(PSTR("L%d"), in);
#endif	

				MAX_buf[MAX_rxfill++] = in;
				MAX_crc = calc_crc_step(in, MAX_crc);

				if (MAX_rxfill >= (MAX_len + 1 + MAX_overlength) || MAX_rxfill >= MAX_BUFLEN)
					break;
			}
		}

#ifdef TRACE_RXTX
		printf_P(PSTR("R%d"), MAX_rxfill);
#endif
	}
	else if (MAX_readReg(REG_IRQFLAGS2) & IRQ2_PACKETSENT)
	{
#ifdef TRACE_RXTX
		printf_P(PSTR("T"));
#endif
		// rxstate will be TXDONE at this point
		MAX_rxstate = TXIDLE;
		setMode(MODE_STANDBY);
		MAX_writeReg(REG_DIOMAPPING1, 0x80); // SyncAddress
	}
}

void MAX_Initialize()
{
#ifdef TRACE_RXTX
	printf_P(PSTR("MAX_Initialize ENTER\n"));
#endif

	delay(20); // needed to make RFM69 work properly on power-up

	attachInterrupt(0, MAX_interrupt, RISING);

	spiInit();
	do
		MAX_writeReg(REG_SYNCVALUE1, 0xAA);
	while (MAX_readReg(REG_SYNCVALUE1) != 0xAA);
	do
		MAX_writeReg(REG_SYNCVALUE1, 0x55);
	while (MAX_readReg(REG_SYNCVALUE1) != 0x55);

	// opmode
	MAX_writeReg(0x01, 0x04);
	// packet mode, FSK
	MAX_writeReg(0x02, 0x00);
	// BitRate 10k
	MAX_writeReg(0x03, 0x0C);
	MAX_writeReg(0x04, 0x80);
	// deviation 20khz
	MAX_writeReg(0x05, 0x01);
	MAX_writeReg(0x06, 0x48);
	// frequency 868.3 MHz
	MAX_writeReg(0x07, 0xD9);
	MAX_writeReg(0x08, 0x13);
	MAX_writeReg(0x09, 0x33);

	MAX_writeReg(0x0B, 0x20);	// AfcCtrl, afclowbetaon
	MAX_writeReg(0x19, 0x42);	// RxBw ...
	MAX_writeReg(0x1E, 0x2C);	// FeiStart, AfcAutoclearOn, AfcAutoOn
	MAX_writeReg(0x25, 0x80);	// DioMapping1 = SyncAddress (Rx)

	// RSSI threshold -100dB
	MAX_writeReg(0x29, 0xC8);
	// TX preamble (short=3 long=1250=1s)
	MAX_writeReg(0x2C, 0x04);
	MAX_writeReg(0x2D, 0xE2);
	// sync word 4 bytes (C6 26 C6 26)
	MAX_writeReg(0x2E, 0x98);
	MAX_writeReg(0x2F, 0xC6);
	MAX_writeReg(0x30, 0x26);
	MAX_writeReg(0x31, 0xC6);
	MAX_writeReg(0x32, 0x26);
	// packet config (fixed length, no whitening/manchester, no crc)
	MAX_writeReg(0x37, 0x00);
	// payload length (unlimited)
	MAX_writeReg(0x38, 0x00);

	MAX_writeReg(0x3C, 0x8F); // FifoTresh, not empty, level 15
	MAX_writeReg(0x3D, 0x10); // PacketConfig2, interpkt = 1, autorxrestart off
	MAX_writeReg(0x6F, 0x20); // TestDagc ...

	MAX_rxstate = TXIDLE;

#ifdef TRACE_RXTX
	printf_P(PSTR("MAX_Initialize LEAVE\n"));
#endif
}

void sendToFifo(uint8_t n, uint8_t out, bool doCrc)
{
	if (doCrc)
		MAX_crc = calc_crc_step(out, MAX_crc);

	while ((MAX_readReg(REG_IRQFLAGS2) & IRQ2_FIFOFULL) != 0);

	MAX_writeReg(REG_FIFO, out ^ pn9_table[n]);
}

void MAX_sendStart(bool fast, const uint8_t* header, uint8_t headerLength, const uint8_t* payload, uint8_t payloadLength)
{
#ifdef TRACE_RXTX
	printf_P(PSTR("s"));
#endif

	MAX_rxstate = TXCRC1;
	flushFifo();
	setMode(MODE_TRANSMITTER);

	if (fast)
	{
		MAX_writeReg(0x2C, 0x00);
		MAX_writeReg(0x2D, 0x04);
	}
	else
	{
		MAX_writeReg(0x2C, 0x04);
		MAX_writeReg(0x2D, 0xE2);
	}

	MAX_writeReg(REG_DIOMAPPING1, 0x00); // PacketSent

#ifdef TRACE_RXTX
	printf_P(PSTR("T%d"), headerLength + payloadLength + 3);
#endif

	uint8_t n = 0;
	MAX_crc = 0xFFFF;
	sendToFifo(n++, headerLength + payloadLength, true);
	for (uint8_t h = 0; h < headerLength; h++)
		sendToFifo(n++, header[h], true);
	for (uint8_t p = 0; p < payloadLength; p++)
		sendToFifo(n++, payload[p], true);
	sendToFifo(n++, MAX_crc >> 8, false);
	sendToFifo(n++, MAX_crc, false);

#ifdef TRACE_RXTX
	printf_P(PSTR("S%d"), n);
#endif

	MAX_rxstate = TXDONE;
}

void MAX_send(bool fast, const uint8_t* header, uint8_t headerLength, const uint8_t* payload, uint8_t payloadLength)
{
#ifdef TRACE_RXTX
	printf_P(PSTR("?"));
#endif
	while (!MAX_canSend());

	MAX_sendStart(fast, header, headerLength, payload, payloadLength);

	uint32_t t = millis();
	uint32_t dt = t - MAX_lastRXTXmillis;
	MAX_lastRXTXmillis = t;
	uint32_t dt0 = t - MAX_RXTXmillis0;

#ifdef TRACE_RXTX
	printf_P(PSTR("w"));
#endif
	while (MAX_sending());

#ifdef TRACE_RXTX
	printf_P(PSTR("W"));
#endif

	MAX_recvDone();

	if (MAX_tracePackets != 0)
	{
		printf_P(PSTR("%10ld::%6ld SENT %c   L:%2d No:%02X F:%02X Cmd:%02X %02X%02X%02X -> %02X%02X%02X G:%02X (-%2d)  P="),
			dt0, dt, fast ? 'F' : ' ',
			headerLength + payloadLength,
			header[0], header[1], header[2],
			header[3], header[4], header[5],
			header[6], header[7], header[8], header[9],
			0);
		for (int p = 0; p < payloadLength; p++)
		{
			printf_P(PSTR("%02X "), payload[p]);
		}
		printf_P(PSTR("\n"));
	}

	if (MAX_culMessages != 0)
	{
		printf_P(PSTR("Z%c%02X"), fast ? 'f' : 's', headerLength + payloadLength);
		for (uint8_t h = 0; h < headerLength; h++)
			printf_P(PSTR("%02X"), header[h]);
		for (uint8_t p = 0; p < payloadLength; p++)
			printf_P(PSTR("%02X"), payload[p]);
		printf_P(PSTR("00\n"));
	}
}

void MAX_send(bool fast, uint8_t msgId, uint8_t flags, uint8_t cmd, uint32_t src, uint32_t dest, uint8_t groupId, const uint8_t* payload, uint8_t payloadLength)
{
	uint8_t header[10];
	header[0] = msgId;
	header[1] = flags;
	header[2] = cmd;
	header[3] = (src >> 16);
	header[4] = (src >> 8);
	header[5] = src;
	header[6] = (dest >> 16);
	header[7] = (dest >> 8);
	header[8] = dest;
	header[9] = groupId;

	MAX_send(fast, header, 10, payload, payloadLength);
}