//#define USESERIAL 1
//#define USESERIAL2 1
#define RF69_COMPAT 1
#include "JeeLib.h"
#include "avr\wdt.h"

#define VERSION "[ThoGateway::LaCrosse V2.1]"

#define LED_PIN 9
#define LED_ON LOW
#define LED_OFF HIGH
byte LEDenabled = 1;

#define DEVICETYPE 9
#define NETWORK 212
#define NODEID 31

#define SWITCHRATEINTERVAL 59000
#define RATE_1 17241
#define RATE_2 9579
uint16_t currentRate = 0;
byte RssiThreshold = 110;

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

byte CRC8(byte* data, byte dataLength)
{
	int i, j;
	byte res = 0;
	for (j = 0; j < dataLength; j++) {
		byte val = data[j];
		for (i = 0; i < 8; i++) {
			byte tmp = (byte)((res ^ val) & 0x80);
			res <<= 1;
			if (0 != tmp) {
				res ^= 0x31;
			}
			val <<= 1;
		}
	}

	return res;
}

void printLaCrosseData(byte rssi, byte nodeId, uint16_t T10, byte H, byte weak, byte low)
{
	uint16_t p = (low != 0) ? 22000 : (weak != 0) ? 24000 : 26000;

	printf_P(PSTR("OK %d %d %d %d %d %d %d %d %d %d %d (-%d)\n"),
		(byte)DEVICETYPE, (byte)NETWORK, nodeId,
		(byte)CMD_temperature, (byte)(T10 & 0xFF), (byte)((T10 >> 8) & 0xFF),
		(byte)CMD_humidity, H,
		(byte)CMD_PowerSupply, (byte)(p & 0xFF), (byte)((p >> 8) & 0xFF),
		rssi);
}

void setRate(uint16_t rate)
{
	rf12_sleep(RF12_SLEEP);

	switch (rate)
	{
	case RATE_1:
		currentRate = RATE_1;
		RF69::control(0x03 | 0x80, 0x07);
		RF69::control(0x04 | 0x80, 0x40);
		break;

	case RATE_2:
		currentRate = RATE_2;
		RF69::control(0x03 | 0x80, 0x0D);
		RF69::control(0x04 | 0x80, 0x0D);
		break;
	}

	rf12_sleep(RF12_WAKEUP);
}

void switchRate()
{
	setRate((currentRate == RATE_1) ? RATE_2 : RATE_1);
	printf_P(PSTR("RateBps: %d\n"), currentRate);
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
	printf_P(PSTR("RateBps: %d\n"), currentRate);
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
	rf12_initialize(NODEID, RF12_868MHZ, NETWORK);
	// LaCrosse frequency 868.3 MHz
	RF69::control(0x07 | 0x80, 0xD9);
	RF69::control(0x08 | 0x80, 0x13);
	RF69::control(0x09 | 0x80, 0x00);
	setRssiThreshold();
	setRate(RATE_1);

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

			if (!rf12_recvDone())
				continue;

#if defined(USESERIAL2)
			printf_P(PSTR("CRC:%02X DATA: "), rf12_crc);
			for (int i = 0; i < 16; i++)
			{
				printf_P(PSTR("%02X "), rf12_buf[i]);
			}
			printf_P(PSTR("\n"));
#endif
			if ((rf12_buf[1] & 0xF0) != 0x90)
				continue;

			byte crc8 = CRC8((byte*)rf12_buf + 1, 4);
			if (crc8 != rf12_buf[5])
				continue;

			msgCount++;
			activityLED(LED_ON);

			byte rssi = (RF69::rssi >> 1);
			byte nodeId = ((rf12_buf[1] & 0x0F) << 2) | (rf12_buf[2] >> 6);
			byte weak = ((rf12_buf[4] >> 7) & 0x01);
			byte low = ((rf12_buf[2] >> 5) & 0x01);
			int16_t T10 = (rf12_buf[2] & 0xF) * 100 + (rf12_buf[3] >> 4) * 10 + (rf12_buf[3] & 0xF);
			byte H = (rf12_buf[4] & 0x7F);

			printLaCrosseData(rssi, nodeId, T10, H, weak, low);

			activityLED(LED_OFF);

#if defined(USESERIAL)
			printf_P(PSTR("LACROSSE: ID:%d %dC %d%% weak:%d low:%d (-%ddB)\n"), nodeId, T10, H, weak, low, rssi);
#endif
		}

		switchRate();
	}

	if (msgCount == 0)
	{
		printf_P(PSTR("\nRESET DUE TO MESSAGE TIMEOUT ...\n"));
		Reset();
	}
}
