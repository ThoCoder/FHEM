//#define USESERIAL 1
//#define USESERIAL2 1
#define RF69_COMPAT 1
#include "JeeLib.h"
#include "avr\wdt.h"
#include "MAX.h"
#include "CRC16.h"

#define VERSION "[ThoGateway::MAX! V1.1]"

#define LED_PIN 9
#define LED_ON LOW
#define LED_OFF HIGH
byte LEDenabled = 1;

#define LOOP_TIMEOUT_INTERVAL 59000
#define RECEIVEWATCHDOGLOOPS 6

byte RssiThreshold = 110;
byte nextMsgId = 1;
byte AutoAck = 0;
byte PairMode = 0;
byte EchoCommands = 0;

#define STACKSIZE 32
unsigned long value;
byte stack[STACKSIZE], top, pending;

#define CONFIG_EEPROM_ADDR ((uint8_t*)0x60)
#define CONFIG_MAGIC 0x12345678
struct _config
{
	long magic;
	byte RssiThreshold;
	byte LEDenabled;
	byte Trace;
	uint32_t OwnAddress;
	byte AutoAck;
	byte PairMode;
	byte CulMessages;
	byte Cul868Compatibility;
	byte EchoCommands;
} config;

void loadConfig()
{
	eeprom_read_block(&config, CONFIG_EEPROM_ADDR, sizeof(config));
	if (config.magic != CONFIG_MAGIC)
	{
		saveConfig();
	}
	else
	{
		RssiThreshold = config.RssiThreshold;
		LEDenabled = config.LEDenabled;
		MAX_tracePackets = config.Trace;
		MAX_ownAddress = config.OwnAddress;
		AutoAck = config.AutoAck;
		PairMode = config.PairMode;
		MAX_culMessages = config.CulMessages;
		MAX_cul868Compatibility = config.Cul868Compatibility;
		EchoCommands = config.EchoCommands;
	}
}

void saveConfig()
{
	config.magic = CONFIG_MAGIC;
	config.RssiThreshold = RssiThreshold;
	config.LEDenabled = LEDenabled;
	config.Trace = MAX_tracePackets;
	config.OwnAddress = MAX_ownAddress;
	config.AutoAck = AutoAck;
	config.PairMode = PairMode;
	config.CulMessages = MAX_culMessages;
	config.Cul868Compatibility = MAX_cul868Compatibility;
	config.EchoCommands = EchoCommands;

	eeprom_update_block(&config, CONFIG_EEPROM_ADDR, sizeof(config));
}

int printChar(char var, FILE *stream) {
	if (var == '\n') Serial.print('\r');
	Serial.print(var);
	return 0;
}

FILE out = { 0 };

void Reset()
{
	wdt_enable(WDTO_2S);
	while (true);
}

void activityLED(byte ledOnOff)
{
	if (LEDenabled == 0)
		return;

	digitalWrite(LED_PIN, ledOnOff);
}

void setRssiThreshold()
{
	MAX_writeReg(0x29, RssiThreshold << 1);
}

void printVersion()
{
	printf_P(PSTR("\n" VERSION "\n"));
}

void printConfiguration()
{
	printf_P(PSTR("CarrierMHz: 868.3\n"));
	printf_P(PSTR("RateBps: 10000\n"));
	printf_P(PSTR("RssiThresholdDB: -%d\n"), RssiThreshold);
	printf_P(PSTR("LED: %s\n"), (LEDenabled != 0) ? "on" : "off");
	printf_P(PSTR("EchoCommands: %s\n"), (EchoCommands != 0) ? "on" : "off");
	printf_P(PSTR("Trace: %s\n"), (MAX_tracePackets != 0) ? "on" : "off");
	printf_P(PSTR("OwnAddress: %06lX\n"), MAX_ownAddress);
	printf_P(PSTR("AutoAck: %s\n"), (AutoAck != 0) ? "on" : "off");
	printf_P(PSTR("PairMode: %s\n"), (PairMode != 0) ? "on" : "off");
	printf_P(PSTR("CulMessages: %s\n"), (MAX_culMessages != 0) ? "on" : "off");
	printf_P(PSTR("Cul868Compatibility: %s\n"), (MAX_cul868Compatibility != 0) ? "on" : "off");
}

const char helpText[] PROGMEM =
"\n"
"Available commands:\n\n"
"h           .. this help\n"
"v           .. print version and configuration\n"
"e           .. echo commands (0=off, 1=on)\n"
"<v>r        .. set RSSI threshold to -<v>dB\n"
"<v>l        .. activity LED (0=off, 1=on)\n"
"<v>x        .. packet tracing (0=off, 1=on)\n"
"Q           .. force reset\n"
"\n"
"ZaAAAAAA\\n .. set own address\n"
"<v>a        .. auto ack (0=off, 1=on)\n"
"<v>p        .. pair mode (0=off, 1=on)\n"
"<v>m        .. CUL messages (0=off, 1=on)\n"
"<v>c        .. CUL868 compatibility (0=off, 1=on)\n"
"\n"
"Zsllnnffccssssssddddddggpp...\\n  .. send with long preamble\n"
"Zfllnnffccssssssddddddggpp...\\n  .. send with short preamble\n"
"  ll     .. length field (can be 00, will be filled in automatically\n"
"  nn     .. message ID\n"
"  ff     .. flags\n"
"  cc     .. command\n"
"  ssssss .. source address\n"
"  dddddd .. destination address\n"
"  gg     .. group ID\n"
"  pp...  .. payload\n"
"\n\n";

char hexCmd = 0;
char hexSubCmd = 0;

void handleInput(char c)
{
	if (hexCmd != 0)
	{
		handleHexInput(c);
		return;
	}

	if (top == 0)
	{
		switch (c)
		{
		case 'Z':
		case 'z':
			pending = 0;
			hexCmd = c;
			return;
		}
	}

	if (c == '\r' || c == '\n')
	{
		value = top = pending = 0;
		printf_P(PSTR("\n"));
		return;
	}

	if (('0' <= c) && (c <= '9'))
	{
		value = (10 * value) + (c - '0');
		pending = 1;
		return;
	}

	if ((c == ',') || (c == ' '))
	{
		if (pending && (top < (sizeof(stack) - 1)))
		{
			stack[top++] = value; // truncated to 8 bits
			value = pending = 0;
		}
		return;
	}

	if (c == 'W')
	{
		if (pending && (top < (sizeof(stack) - 2)))
		{
			stack[top++] = value; // truncated to 8 bits
			value >>= 8;
			stack[top++] = value; // truncated to 8 bits

			value = pending = 0;
		}
		return;
	}

	if (c == 'D')
	{
		if (pending && (top < (sizeof(stack) - 4)))
		{
			stack[top++] = value; // truncated to 8 bits
			value >>= 8;
			stack[top++] = value; // truncated to 8 bits
			value >>= 8;
			stack[top++] = value; // truncated to 8 bits
			value >>= 8;
			stack[top++] = value; // truncated to 8 bits

			value = pending = 0;
		}
		return;
	}

	if (('a' <= c && c <= 'z') || ('A' <= c && c <= 'Z') || (c == '?'))
	{
		if (pending && (top < (sizeof(stack) - 1)))
		{
			stack[top++] = value; // truncated to 8 bits
			value = pending = 0;
		}

		if (EchoCommands == 1)
		{
			printf_P(PSTR("Command: "));
			for (byte i = 0; i < top; i++)
			{
				printf_P(PSTR("%d,"), stack[i]);
			}
			printf_P(PSTR("%c\n"), c);
		}
	}

	switch (c)
	{
	case 'v':
		printVersion();
		printConfiguration();
		break;

	case 'e':
		EchoCommands = (stack[0] != 0) ? 1 : 0;
		printf_P(PSTR("EchoCommands: %s\n"), (EchoCommands != 0) ? "on" : "off");
		saveConfig();
		break;

	case 'r':
		RssiThreshold = stack[0];
		setRssiThreshold();
		printf_P(PSTR("RssiThresholdDB: -%d\n"), RssiThreshold);
		saveConfig();
		break;

	case 'l':
		LEDenabled = (stack[0] != 0) ? 1 : 0;
		printf_P(PSTR("LED: %s\n"), (LEDenabled != 0) ? "on" : "off");
		saveConfig();
		break;

	case 'x':
		MAX_tracePackets = (stack[0] != 0) ? 1 : 0;
		printf_P(PSTR("Trace: %s\n"), (MAX_tracePackets != 0) ? "on" : "off");
		saveConfig();
		break;

	case 'a':
		AutoAck = (stack[0] != 0) ? 1 : 0;
		printf_P(PSTR("AutoAck: %s\n"), (AutoAck != 0) ? "on" : "off");
		saveConfig();
		break;

	case 'p':
		PairMode = (stack[0] != 0) ? 1 : 0;
		printf_P(PSTR("PairMode: %s\n"), (PairMode != 0) ? "on" : "off");
		saveConfig();
		break;

	case 'm':
		MAX_culMessages = (stack[0] != 0) ? 1 : 0;
		printf_P(PSTR("CulMessages: %s\n"), (MAX_culMessages != 0) ? "on" : "off");
		saveConfig();
		break;

	case 'c':
		MAX_cul868Compatibility = (stack[0] != 0) ? 1 : 0;
		printf_P(PSTR("Cul868Compatibility: %s\n"), (MAX_cul868Compatibility != 0) ? "on" : "off");
		saveConfig();
		break;

	case 'Q':
		printf_P(PSTR("\nFORCED RESET ...\n"));
		Reset();
		break;

	// fakes for FHEM CUL-module
	case '?':
		printf_P(PSTR("? (? is unknown) Use one of B b C F i A Z N k G M K U Y R T V W X e f m L l t u x\n"));
		break;
	case 'V':
		printf_P(PSTR("V 1.66 CUL868\n"));
		break;
	case 't':
		printf_P(PSTR("00000064\n"));
		break;
	case 'X':
		printf_P(PSTR("00 900\n"));
		break;

	// help is default
	case 'h':
		printf_P(helpText);
		break;

	default:
		if (MAX_cul868Compatibility == 0)
			printf_P(helpText);
		break;
	}

	value = top = pending = 0;
}

void handleHexInput(char c)
{
	if (hexSubCmd == 0)
	{
		hexSubCmd = c;
		return;
	}

	if (('0' <= c) && (c <= '9'))
	{
		value = (value << 4) + (c - '0');
	}
	else if (('a' <= c) && (c <= 'f'))
	{
		value = (value << 4) + (c - 'a' + 10);
	}
	else if (('A' <= c) && (c <= 'F'))
	{
		value = (value << 4) + (c - 'A' + 10);
	}
	else if (c == '\r' || c == '\n')
	{
		switch (hexSubCmd)
		{
		case 's':
		case 'S':
			// Zsllnnffccssssssddddddggpp...
			if (top < 11)
				break;
			MAX_send(false, stack + 1, 10, stack + 11, top - 11);
			break;

		case 'f':
		case 'F':
			// Zfllnnffccssssssddddddggpp...
			if (top < 11)
				break;
			MAX_send(true, stack + 1, 10, stack + 11, top - 11);
			break;

		case 'a':
		case 'A':
			// ZaXXXXXX (set own address)
			MAX_ownAddress = (((uint32_t)stack[0]) << 16) | (((uint32_t)stack[1]) << 8) | ((uint32_t)stack[2]);
			if (MAX_cul868Compatibility == 0)
				printf_P(PSTR("OwnAddress: %06lX\n"), MAX_ownAddress);
			saveConfig();
			break;

		case 'r':
		case 'R':
			// Zr (temporary enable CUL messages)
			MAX_culMessages = 1;
			if (MAX_cul868Compatibility == 0)
				printf_P(PSTR("CulMessages: %s\n"), (MAX_culMessages != 0) ? "on" : "off");
			break;

		case 'x':
		case 'X':
			// Zx (temporary disable CUL messages)
			MAX_culMessages = 0;
			if (MAX_cul868Compatibility == 0)
				printf_P(PSTR("CulMessages: %s\n"), (MAX_culMessages != 0) ? "on" : "off");
			break;
		}

		hexCmd = hexSubCmd = 0;
		value = top = pending = 0;
	}
	else
	{
		return;
	}

	if (pending == 0)
	{
		pending = 1;
		return;
	}

	if (top < (sizeof(stack) - 1))
	{
		stack[top++] = value; // truncated to 8 bits
		value = pending = 0;
	}
}

void setup()
{
	wdt_enable(WDTO_8S);

	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LED_ON);

	Serial.begin(57600);
	fdev_setup_stream(&out, printChar, NULL, _FDEV_SETUP_WRITE);
	stdout = &out;

	loadConfig();
	if (MAX_cul868Compatibility == 1)
		MAX_culMessages = 0;

	if (MAX_cul868Compatibility == 0)
		printVersion();

	MAX_Initialize();

	//rf12_initialize(NODEID, RF12_868MHZ, NETWORK);
	//// BitRate 10k
	//RF69::control(0x03 | 0x80, 0x0C);
	//RF69::control(0x04 | 0x80, 0x80);
	//// deviation 20khz
	//RF69::control(0x05 | 0x80, 0x01);
	//RF69::control(0x06 | 0x80, 0x48);
	//// frequency 868.3 MHz
	//RF69::control(0x07 | 0x80, 0xD9);
	//RF69::control(0x08 | 0x80, 0x13);
	//RF69::control(0x09 | 0x80, 0x33);
	//// RSSI threshold -100dB
	//RF69::control(0x29 | 0x80, RssiThreshold << 1);
	//// TX preamble (short=3 long=1250=1s)
	//RF69::control(0x2C | 0x80, 0x04);
	//RF69::control(0x2D | 0x80, 0xE2);
	//// sync word 4 bytes (C6 26 C6 26)
	//RF69::control(0x2E | 0x80, 0x98);
	//RF69::control(0x2F | 0x80, 0xC6);
	//RF69::control(0x30 | 0x80, 0x26);
	//RF69::control(0x31 | 0x80, 0xC6);
	//RF69::control(0x32 | 0x80, 0x26);
	//// packet config (fixed length, no whitening/manchester, no crc)
	//RF69::control(0x37 | 0x80, 0x00);
	//// payload length (unlimited)
	//RF69::control(0x38 | 0x80, 0x00);

	if (MAX_cul868Compatibility == 0)
		printConfiguration();

	delay(1000);

	digitalWrite(LED_PIN, LED_OFF);
}

uint32_t wakeupDelay = 0;
uint32_t shutterAddress = 0;
uint32_t lastWakeupTick = 0;
byte waitWakeupAck = 0;
#define WAKEUPTIMEOUT 0x3F
byte awake = 0;

void loop()
{
	int msgCount = 0;

	for (byte watchdogLoops = 0; watchdogLoops < RECEIVEWATCHDOGLOOPS; watchdogLoops++)
	{
		MilliTimer loopTimer;
		while (!loopTimer.poll(LOOP_TIMEOUT_INTERVAL))
		{
			wdt_reset();

			while (Serial.available())
				handleInput(Serial.read());

			// shutter contact wake up tests
			//sendWakeups();

			if (MAX_recvDone() == 0)
				continue;

			uint8_t buf[MAX_BUFLEN];
			uint8_t buf_fill = MAX_rxfill;
			for (uint8_t i = 0; i < buf_fill; i++)
				buf[i] = MAX_buf[i];
			uint8_t len = buf[0];
			uint16_t crc = MAX_crc;
			uint8_t rssi = MAX_rssi >> 1;

			if (crc != 0)
				continue;

			MAX_recvDone();

			msgCount++;
			activityLED(LED_ON);

			if (MAX_culMessages != 0)
			{
				printf_P(PSTR("Z"));
				for (int i = 0; i < buf_fill - 2; i++)
				{
					printf_P(PSTR("%02X"), buf[i]);
				}
				printf_P(PSTR("%02X\n"), MAX_rssi >> 1);
			}

			uint8_t cmd = MxP_Cmd(buf);
			uint8_t msgId = MxP_MsgId(buf);
			uint32_t src = MxP_Src(buf);
			uint32_t dst = MxP_Dst(buf);
			uint8_t payloadOk = 0;

			if (PairMode && (cmd == MxM_PairPing))
			{
				delay(20);
				MAX_send(true, msgId, 0, MxM_PairPong, MAX_ownAddress, src, 0, &payloadOk, 1);
			}
			else if (AutoAck && (dst == MAX_ownAddress) &&
				((cmd == MxM_ShutterContactState) ||
				(cmd == MxM_SetTemperature) ||
				(cmd == MxM_PushButtonState)))
			{
				delay(20);
				MAX_send(true, msgId, 2, MxM_Ack, MAX_ownAddress, src, 0, &payloadOk, 1);
			}

			// shutter contact wake up tests
			//handleShutterContact(buf);

			activityLED(LED_OFF);
		}
	}

	if (msgCount == 0)
	{
		printf_P(PSTR("\nRESET DUE TO MESSAGE TIMEOUT ...\n"));
		Reset();
	}
}

void sendWakeup(uint32_t dstAddress, uint8_t wakeTimeout)
{
	// Zf008000F112345616d23f003f\n
	delay(20);
	MAX_send(true, nextMsgId++, 0, MxM_Wakeup, MAX_ownAddress, dstAddress, 0, &wakeTimeout, 1);
	lastWakeupTick = millis();
}

void handleShutterContact(uint8_t* buf)
{
	uint8_t cmd = MxP_Cmd(buf);
	uint8_t msgId = MxP_MsgId(buf);
	uint32_t src = MxP_Src(buf);
	uint32_t dst = MxP_Dst(buf);
	uint8_t payloadOk = 0;

	switch (cmd)
	{
	case MxM_Ack:

		if (dst != MAX_ownAddress)
			return;

		if (waitWakeupAck != 0)
		{
			printf_P(PSTR("%6ld GOT WAKEUP ACK\n"), wakeupDelay);
			waitWakeupAck = 0;
		}
		return;

	case MxM_ShutterContactState:

		if ((dst >= 0x999900) && (dst <= 0x9999FF))
		{
			delay(20);
			MAX_send(true, msgId, 2, MxM_Ack, dst, src, 0, &payloadOk, 1);
		}
		else if ((dst != MAX_ownAddress) && (dst != 0))
		{
			return;
		}

		if (wakeupDelay != 0)
			return;

		wakeupDelay = 5000;
		shutterAddress = src;
		waitWakeupAck = 1;
		printf_P(PSTR("%6ld SEND WAKEUP\n"), wakeupDelay);
		sendWakeup(shutterAddress, WAKEUPTIMEOUT);
		return;
	}
}

void sendWakeups()
{
	if (wakeupDelay == 0)
		return;

	if (millis() - lastWakeupTick >= wakeupDelay)
	{
		if (waitWakeupAck != 0)
		{
			printf_P(PSTR("%6ld WAKEUP ACK TIMEOUT\n"), wakeupDelay);
			wakeupDelay = 0;
			waitWakeupAck = 0;
			return;
		}

		printf_P(PSTR("%6ld SEND WAKEUP\n"), wakeupDelay);
		//wakeupDelay *= 2;
		//wakeupDelay += 2000;
		wakeupDelay = 30000;
		waitWakeupAck = 1;
		sendWakeup(shutterAddress, WAKEUPTIMEOUT);
	}
}
