//#define USESERIAL 1
//#define USESERIAL2 1
//#define USESERIAL3 1
#define RF69_COMPAT 1
#include "JeeLib.h"
#include "avr\wdt.h"
#include "postbox.h"
#define REG_SYNCGROUP 0x33

#define VERSION "[ThoGateway::JeeLink V2.3]"

#define LED_PIN 9
#define LED_ON LOW
#define LED_OFF HIGH
byte LEDenabled = 1;

#define DEVICETYPE 1
byte GroupId = 1;
#define NODEID 31
byte AckMode = 0;
byte RssiThreshold = 110;

#define LOOPINTERVAL 60000
#define RECEIVEWATCHDOGLOOPS 15
byte msg[RF12_MAXDATA + 5];
byte msgLen;

#define STACKSIZE 64
unsigned long value;
byte stack[STACKSIZE], top, pending;
Postbox postbox;

#define CONFIG_EEPROM_ADDR ((uint8_t*)0x60)
#define CONFIG_MAGIC 0x12345678
struct _config
{
	long magic;
	byte GroupId;
	byte AckMode;
	byte RssiThreshold;
	byte LEDenabled;
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
		GroupId = config.GroupId;
		AckMode = config.AckMode;
		RssiThreshold = config.RssiThreshold;
		LEDenabled = config.LEDenabled;
	}
}

void saveConfig()
{
	config.magic = CONFIG_MAGIC;
	config.GroupId = GroupId;
	config.AckMode = AckMode;
	config.RssiThreshold = RssiThreshold;
	config.LEDenabled = LEDenabled;

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
	RF69::control(0x29 | 0x80, RssiThreshold << 1);
}

void printVersion()
{
	printf_P(PSTR("\n" VERSION "\n"));
}

void printConfiguration()
{
	printf_P(PSTR("CarrierMHz: 868.3\n"));
	printf_P(PSTR("RateBps: 49261\n"));
	printf_P(PSTR("RssiThresholdDB: -%d\n"), RssiThreshold);
	printf_P(PSTR("GroupId: %d\n"), GroupId);
	printf_P(PSTR("AckMode: %s\n"), (AckMode != 0) ? "on" : "off");
	printf_P(PSTR("LED: %s\n"), (LEDenabled != 0) ? "on" : "off");
}

const char helpText[] PROGMEM =
"\n"
"Available commands:\n\n"
"h                ... this help\n"
"v                ... print version and configuration\n"
"<v>g             ... set group <v>\n"
"<v>a             ... set ACK mode (0=off, 1=on)\n"
"<v>t             ... set RSSI threshold to -<v>dB\n"
"<n>,<v>,<v>,..m  ... store message for node <n> (<v>W for uint16, <v>D for uint32)\n"
"<n>m             ... clear message for node <n>\n"
"m                ... dump messages\n"
"<v>l             ... activity LED (0=off, 1=on)\n"
"r                ... reset\n"
"\n\n";

void handleInput(char c)
{
	if (('0' <= c) && (c <= '9'))
	{
		value = 10 * value + c - '0';
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

	if ('a' <= c && c <= 'z')
	{
		if (pending && (top < (sizeof(stack) - 1)))
		{
			stack[top++] = value; // truncated to 8 bits
			value = pending = 0;
		}

		printf_P(PSTR("Command: "));
		for (byte i = 0; i < top; i++)
		{
			printf_P(PSTR("%d,"), stack[i]);
		}
		printf_P(PSTR("%c\n"), c);
	}

	switch (c)
	{
	case 'v':
		printVersion();
		printConfiguration();
		break;

	case 'g':
		GroupId = stack[0];
		rf12_initialize(NODEID, RF12_868MHZ, GroupId);
		printf_P(PSTR("GroupId: %d\n"), GroupId);
		saveConfig();
		break;

	case 'a':
		AckMode = (stack[0] != 0) ? 1 : 0;
		printf_P(PSTR("AckMode: %s\n"), (AckMode != 0) ? "on" : "off");
		saveConfig();
		break;

	case 't':
		RssiThreshold = stack[0];
		setRssiThreshold();
		printf_P(PSTR("RssiThresholdDB: -%d\n"), RssiThreshold);
		saveConfig();
		break;

	case 'm':
		if (top == 0)
		{
			postbox.Dump();
		}
        else if (top == 1)
        {
            printf_P(PSTR("Missing nodeId\n"));
        }
        else if (top == 2)
        {
            postbox.ClearEntry(stack[0], stack[1]);
        }
        else
		{
			postbox.SetEntry(stack[0], stack[1], stack + 2, top - 2);
		}
		break;

	case 'l':
		LEDenabled = (stack[0] != 0) ? 1 : 0;
		printf_P(PSTR("LED: %s\n"), (LEDenabled != 0) ? "on" : "off");
		saveConfig();
		break;

	case 'r':
		printf_P(PSTR("\nFORCED RESET ...\n"));
		Reset();
		break;

	case 'h':
	default:
		printf_P(helpText);
	}

	value = top = pending = 0;
}

void setup()
{
	wdt_enable(WDTO_8S);

	pinMode(LED_PIN, OUTPUT);
	activityLED(LED_ON);

	Serial.begin(57600);
	fdev_setup_stream(&out, printChar, NULL, _FDEV_SETUP_WRITE);
	stdout = &out;

	printVersion();

	loadConfig();
	rf12_initialize(NODEID, RF12_868MHZ, GroupId);
	setRssiThreshold();

	printConfiguration();

	delay(1000);

	activityLED(LED_OFF);
}

void loop()
{
	int msgCount = 0;

	for (byte watchdogLoops = 0; watchdogLoops < RECEIVEWATCHDOGLOOPS; watchdogLoops++)
	{
		MilliTimer ackTimer;
		while (!ackTimer.poll(LOOPINTERVAL))
		{
			wdt_reset();

			while (Serial.available())
				handleInput(Serial.read());

			if (!rf12_recvDone())
				continue;

			if (rf12_crc != 0)
			{
#if defined(USESERIAL3)
				printf_P(PSTR("ERR\n"));
#endif
				continue;
			}

			if (rf12_len > RF12_MAXDATA)
			{
#if defined(USESERIAL2)
				printf_P(PSTR("TOO LONG len:%d > max:%d\n"), rf12_len, RF12_MAXDATA);
#endif
				continue;
			}

			msgCount++;

			byte rssi = (RF69::rssi >> 1);
			bool ctl = ((rf12_hdr & RF12_HDR_CTL) != 0);
			bool dst = ((rf12_hdr & RF12_HDR_DST) != 0);
			bool ack = ((rf12_hdr & RF12_HDR_ACK) != 0);
			byte nodeId = rf12_hdr & RF12_HDR_MASK;
            byte groupId = rf12_grp;

#if defined(USESERIAL)
			printf_P(PSTR("(-%3d) %c%c%c %2d DATA(%2d): "),
				rssi,
				ctl ? 'C' : '_', dst ? 'D' : '_', ack ? 'A' : '_', nodeId,
				rf12_len);
#endif
#if defined(USESERIAL2)
			for (int i = 0; i < min(16, rf12_len); i++)
			{
				printf_P(PSTR("%3d "), rf12_data[i]);
			}
#endif
#if defined(USESERIAL)
			printf_P(PSTR("\n"));
#endif
			if (ctl)
				continue;
			if (dst)
				continue;

			activityLED(LED_ON);

			msgLen = rf12_len;
			memcpy(msg, (byte*)rf12_data, msgLen);
			bool acked = false;
			byte ackLen = 0;

			if (ack && (AckMode != 0))
			{
				acked = true;

#if defined(USESERIAL)
				printf_P(PSTR("ACK\n"));
#endif
                if(GroupId == 0)
                    RF69::control(REG_SYNCGROUP | 0x80, groupId); // Reply to incoming group number

				PostboxEntry* entry = postbox.GetEntry(groupId, nodeId);
				if (entry != NULL)
				{
					ackLen = entry->DataLen;
					rf12_sendStart(RF12_ACK_REPLY, entry->Data, entry->DataLen);
					postbox.ClearEntry(entry);
				}
				else
				{
					rf12_sendStart(RF12_ACK_REPLY, 0, 0);
				}
			}

			printf_P(PSTR("OK %d %d %d "), (byte)DEVICETYPE, groupId, nodeId);
			for (int i = 0; i < msgLen; i++)
			{
				printf_P(PSTR("%d "), msg[i]);
			}
			printf_P(PSTR("(-%d)\n"), rssi);
			if (acked)
			{
				printf_P(PSTR("Ack: %d,%d,%d\n"), groupId, nodeId, ackLen);
			}

			activityLED(LED_OFF);
		}
	}

	if (msgCount == 0)
	{
		printf_P(PSTR("\nRESET DUE TO MESSAGE TIMEOUT ...\n"));
		Reset();
	}
}
