//#define USESERIAL 1
//#define USESERIAL2 1
#include "CRC16.h"
#define RF69_COMPAT 1
#include "JeeLib.h"
#include "avr\wdt.h"
#include "MAX.h"
#include "CRC16.h"

#define VERSION "[ThoGateway::MAX! V1.0]"

#define LED_PIN 9
#define LED_ON LOW
#define LED_OFF HIGH
byte LEDenabled = 1;

#define DEVICETYPE 10
#define NETWORK 77
#define NODEID 31

#define SWITCHRATEINTERVAL 59000
byte RssiThreshold = 110;
uint8_t msgId = 1;

#define RECEIVEWATCHDOGLOOPS 6

#define CMD_temperature 12
#define CMD_humidity 16
#define CMD_PowerSupply 252

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
	}
}

void saveConfig()
{
	config.magic = CONFIG_MAGIC;
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
}

const char helpText[] PROGMEM =
"\n"
"Available commands:\n\n"
"h                ... this help\n"
"v                ... print version and configuration\n"
"<v>t             ... set RSSI threshold to -<v>dB\n"
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

	uint8_t wakeupPayload[] = { 0x3F };

	switch (c)
	{
	case 'v':
		printVersion();
		printConfiguration();
		break;

	case 't':
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

	case 'r':
		printf_P(PSTR("\nFORCED RESET ...\n"));
		Reset();
		break;

	case 's':
		MAX_send(false, msgId++, 0, 0xF1, 0xAAFFEE, 0x0CD5B0, 0, wakeupPayload, sizeof(wakeupPayload));
		break;

	case 'f':
		MAX_send(true, msgId++, 0, 0xF1, 0xAAFFEE, 0x0CD5B0, 0, wakeupPayload, sizeof(wakeupPayload));
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
	digitalWrite(LED_PIN, LED_ON);

	Serial.begin(57600);
	fdev_setup_stream(&out, printChar, NULL, _FDEV_SETUP_WRITE);
	stdout = &out;

	printVersion();

	loadConfig();

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

	printConfiguration();

	delay(1000);

	digitalWrite(LED_PIN, LED_OFF);
}

void loop()
{
	int msgCount = 0;

	for (byte watchdogLoops = 0; watchdogLoops < RECEIVEWATCHDOGLOOPS; watchdogLoops++)
	{
		MilliTimer ackTimer;
		while (!ackTimer.poll(SWITCHRATEINTERVAL))
		{
			wdt_reset();

			while (Serial.available())
				handleInput(Serial.read());

			if (MAX_recvDone() == 0)
				continue;

			uint8_t buf[MAX_BUFLEN];
			uint8_t buf_fill = MAX_rxfill;
			for (uint8_t i = 0; i < buf_fill; i++)
				buf[i] = MAX_buf[i];
			uint8_t len = buf[0];
			uint16_t crc = MAX_crc;
			uint8_t rssi = MAX_rssi >> 1;

			MAX_recvDone();

#if defined(USESERIAL2)
			printf_P(PSTR("L:%2d "), len);
			for (int i = 0; i < buf_fill; i++)
			{
				printf_P(PSTR("%02X "), buf[i]);
			}
			printf_P(PSTR("\n"));
#endif
			uint32_t t = millis();
			uint32_t dt = t - MAX_lastRXTXmillis;
			MAX_lastRXTXmillis = t;

			printf_P(PSTR("%6ld CRC:%04X L:%2d No:%02X F:%02X Cmd:%02X %02X%02X%02X -> %02X%02X%02X G:%02X (-%2d)  P="), 
				dt, crc, len, buf[1], buf[2], buf[3],
				buf[4], buf[5], buf[6], 
				buf[7], buf[8], buf[9], buf[10],
				rssi);
			for (int i = 11; i < buf_fill - 2; i++)
			{
				printf_P(PSTR("%02X "), buf[i]);
			}
			printf_P(PSTR("\n"));

			msgCount++;
			activityLED(LED_ON);



			activityLED(LED_OFF);
		}
	}

	if (msgCount == 0)
	{
		printf_P(PSTR("\nRESET DUE TO MESSAGE TIMEOUT ...\n"));
		Reset();
	}
}
