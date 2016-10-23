//#define TRACE_RXTX 1

#include "Arduino.h"
#include <stdint.h>
#include <RF69_avr.h>
#include "MAX.h"
#include "CRC16.h"

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
	0x40, 0xc4, /*0xc4, 0xd5, 0xc6, 0x91, 0x8
				0xe7, 0xd1, 0x4e, 0x09, 0x32, 0x17, 0xdf, 0x83,
				0xff, 0xf0, 0x0e, 0xcd, 0xf6, 0xc2, 0x19, 0x12,
				0x75, 0x3d, 0xe9, 0x1c, 0xb8, 0xcb, 0x2b, 0x05,
				0xaa, 0xbe, 0x16, 0xec, 0xb6, 0x06, 0xdd, 0xc7,
				0xb3, 0xac, 0x63, 0xd1, 0x5f, 0x1a, 0x65, 0x0c,
				0x98, 0xa9, 0xc9, 0x6f, 0x49, 0xf6, 0xd3, 0x0a,
				0x45, 0x6e, 0x7a, 0xc3, 0x2a, 0x27, 0x8c, 0x10,
				0x20, 0x62, 0xe2, 0x6a, 0xe3, 0x48, 0xc5, 0xe6,
				0xf3, 0x68, 0xa7, 0x04, 0x99, 0x8b, 0xef, 0xc1,
				0x7f, 0x78, 0x87, 0x66, 0x7b, 0xe1, 0x0c, 0x89,
				0xba, 0x9e, 0x74, 0x0e, 0xdc, 0xe5, 0x95, 0x02,
				0x55, 0x5f, 0x0b, 0x76, 0x5b, 0x83, 0xee, 0xe3,
				0x59, 0xd6, 0xb1, 0xe8, 0x2f, 0x8d, 0x32, 0x06,
				0xcc, 0xd4, 0xe4, 0xb7, 0x24, 0xfb, 0x69, 0x85,
				0x22, 0x37, 0xbd, 0x61, 0x95, 0x13, 0x46, 0x08,
				0x10, 0x31, 0x71, 0xb5, 0x71, 0xa4, 0x62, 0xf3,
				0x79, 0xb4, 0x53, 0x82, 0xcc, 0xc5, 0xf7, 0xe0,
				0x3f, 0xbc, 0x43, 0xb3, 0xbd, 0x70, 0x86, 0x44,
				0x5d, 0x4f, 0x3a, 0x07, 0xee, 0xf2, 0x4a, 0x81,
				0xaa, 0xaf, 0x05, 0xbb, 0xad, 0x41, 0xf7, 0xf1,
				0x2c, 0xeb, 0x58, 0xf4, 0x97, 0x46, 0x19, 0x03,
				0x66, 0x6a, 0xf2, 0x5b, 0x92, 0xfd, 0xb4, 0x42,
				0x91, 0x9b, 0xde, 0xb0, 0xca, 0x09, 0x23, 0x04,
				0x88, 0x98, 0xb8, 0xda, 0x38, 0x52, 0xb1, 0xf9,
				0x3c, 0xda, 0x29, 0x41, 0xe6, 0xe2, 0x7b
				*/
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
	if(doCrc)
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

void MAX_send(bool fast, uint8_t msgId, uint8_t flags, uint8_t cmd, uint32_t src, uint32_t dest, uint8_t groupId, const uint8_t* payload, uint8_t payloadLength)
{
	uint8_t hdr[10];
	hdr[0] = msgId;
	hdr[1] = flags;
	hdr[2] = cmd;
	hdr[3] = (src >> 16);
	hdr[4] = (src >> 8);
	hdr[5] = src;
	hdr[6] = (dest >> 16);
	hdr[7] = (dest >> 8);
	hdr[8] = dest;
	hdr[9] = groupId;

	uint32_t t = millis();
	uint32_t dt = t - MAX_lastRXTXmillis;
	MAX_lastRXTXmillis = t;

	printf_P(PSTR("%6ld     SEND L:%2d No:%02X F:%02X Cmd:%02X %02X%02X%02X -> %02X%02X%02X G:%02X (-%2d)  P="),
		dt, 10 + payloadLength,
		hdr[0], hdr[1], hdr[2],
		hdr[3], hdr[4], hdr[5],
		hdr[6], hdr[7], hdr[8], hdr[9],
		0);
	for (int p = 0; p < payloadLength; p++)
	{
		printf_P(PSTR("%02X "), payload[p]);
	}
	printf_P(PSTR("\n"));


#ifdef TRACE_RXTX
	printf_P(PSTR("?"));
#endif
	while (!MAX_canSend());

	MAX_sendStart(fast, hdr, 10, payload, payloadLength);

#ifdef TRACE_RXTX
	printf_P(PSTR("w"));
#endif
	while (MAX_sending());

#ifdef TRACE_RXTX
	printf_P(PSTR("W"));
#endif
}