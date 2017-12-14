#define USESERIAL
//#define USESERIAL2
//#define LED_COUNTFLASHS
#define LED_SENDFLASHS

#define RF69_COMPAT 1
#include <JeeLib.h>
#include "avr\eeprom.h"

#define LED 7
#define SENSOR_PIN 1//9
#define SENSOR_POWER 10

#define myNodeID 10
#define network 99
#define freq RF12_868MHZ
#define ACK_TIME 50
#define RETRYDELAY 500
#define RETRIES 5
#define WAITLOOPS 600
#define WAITINTERVAL 500
#define DEVIDER 10 
#define MIDLEVEL 465
#define THRESHOLD 15

// set counter (RF12Demo V12.1):            195,nnnnnnD10m
// set counter (ThoGateway::JeeLink V2.1):  10,195,nnnnnnDm
// nnnnnn .. gas counter value to set in (0.1m^3 / DEVIDER) units

uint16_t gcSensorValue;
byte gcSensorPrevState = 255;
byte gcSensorState = 255;
uint16_t gcSensorValueHighThreshold = MIDLEVEL + THRESHOLD;
uint16_t gcSensorValueLowThreshold = MIDLEVEL - THRESHOLD;
byte gcSignal = 0;
byte gcFraction = DEVIDER - 1;
uint32_t gcCounter = 0;
bool triggerSend = false;

uint32_t gcTodayCounter0 = 0;
uint32_t gcYesterdayVolume = 0;

uint16_t gcHighMax = MIDLEVEL + THRESHOLD;
uint16_t gcHighAvg = MIDLEVEL + THRESHOLD;
uint16_t gcLowAvg = MIDLEVEL - THRESHOLD;
uint16_t gcLowMin = MIDLEVEL - THRESHOLD;
uint16_t gcCurMax, gcCurMin;
uint32_t gcCurAvg;
uint16_t gcCurAvgN = 1;
#define AVGNMAX (3600000 / WAITINTERVAL)

#define CMD_TotalVolume 191
#define CMD_TodayVolume 189
#define CMD_YesterdayVolume 188

#define CMD_Signal 194
#define CMD_Count 195
#define CMD_PowerSupply 252
#define CMD_RemoteRSSI 101

#define CMD_Sensor 179
#define CMD_SensorMax 180
#define CMD_SensorAvgHigh 181
#define CMD_SensorThresHigh 182
#define CMD_SensorAvg 183
#define CMD_SensorThresLow 184
#define CMD_SensorAvgLow 185
#define CMD_SensorMin 186

struct DataPacket
{
	byte signalCmd;
	byte signal;		// 0=off, 1=on
	byte countCmd;
	uint32_t count;		// +1 = 0.1m3 = 100L
	byte sensorMaxCmd;
	uint16_t sensorMax;			// 0 .. 1023
	byte sensorAvgHighCmd;
	uint16_t sensorAvgHigh;		// 0 .. 1023
	byte sensorThresHighCmd;
	uint16_t sensorThresHigh;	// 0 .. 1023
	byte sensorThresLowCmd;
	uint16_t sensorThresLow;	// 0 .. 1023
	byte sensorAvgLowCmd;
	uint16_t sensorAvgLow;		// 0 .. 1023
	byte sensorMinCmd;
	uint16_t sensorMin;			// 0 .. 1023
	byte totalVolumeCmd;
	uint32_t totalVolume;	// total volume in l
	byte todayVolumeCmd;
	uint32_t todayVolume; // day counter
	byte yesterdayVolumeCmd;
	uint32_t yesterdayVolume; // previous day counter value
	byte powerCmd;
	uint16_t power;		// mV * 10
	byte remoteRssiCmd;
	byte remoteRssi;	// dB * -1

	DataPacket()
	{
		signalCmd = CMD_Signal;
		countCmd = CMD_Count;
		sensorMaxCmd = CMD_SensorMax;
		sensorAvgHighCmd = CMD_SensorAvgHigh;
		sensorThresHighCmd = CMD_SensorThresHigh;
		sensorThresLowCmd = CMD_SensorThresLow;
		sensorAvgLowCmd = CMD_SensorAvgLow;
		sensorMinCmd = CMD_SensorMin;
		totalVolumeCmd = CMD_TotalVolume;
		todayVolumeCmd = CMD_TodayVolume;
		yesterdayVolumeCmd = CMD_YesterdayVolume;
		powerCmd = CMD_PowerSupply;
		remoteRssiCmd = CMD_RemoteRSSI;
	}
};

struct DataAckPacket
{
	uint32_t totalVolume;
	uint32_t todayVolume;
	uint32_t yesterdayVolume;
	uint16_t thresHigh;
	uint16_t thresLow;
};

DataPacket data;

#if defined(USESERIAL) || defined(USESERIAL2)

int printChar(char var, FILE *stream) {
	if (var == '\n') Serial.print('\r');
	Serial.print(var);
	return 0;
}

FILE out = { 0 };

#endif

ISR(WDT_vect) {
	Sleepy::watchdogEvent();  // interrupt handler for JeeLabs Sleepy power saving
}

#define CONFIG_EEPROM_ADDR ((uint8_t*)0x60)
#define CONFIG_MAGIC 0x12131415
struct _config
{
	long magic;
	uint16_t wcsSensorValueHighThreshold;
	uint16_t wcsSensorValueLowThreshold;
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
		gcSensorValueHighThreshold = config.wcsSensorValueHighThreshold;
		gcSensorValueLowThreshold = config.wcsSensorValueLowThreshold;
	}
}

void saveConfig()
{
	config.magic = CONFIG_MAGIC;
	config.wcsSensorValueHighThreshold = gcSensorValueHighThreshold;
	config.wcsSensorValueLowThreshold = gcSensorValueLowThreshold;

	eeprom_update_block(&config, CONFIG_EEPROM_ADDR, sizeof(config));
}

void printConfig()
{
	printf_P(PSTR("tH: %hu\n"), gcSensorValueHighThreshold);
	printf_P(PSTR("tL: %hu\n"), gcSensorValueLowThreshold);
}

void flashLED(byte interval, byte count)
{
	for (byte n = 0; n < count; n++)
	{
		if (n > 0)
			delay(interval);

		digitalWrite(LED, HIGH);
		delay(interval);
		digitalWrite(LED, LOW);
	}
}

uint32_t readVcc()
{
	bitClear(PRR, PRADC); ADCSRA |= bit(ADEN); // Enable the ADC
	uint32_t result;
	// Read 1.1V reference against Vcc
#if defined(__AVR_ATtiny84__) 
	ADMUX = _BV(MUX5) | _BV(MUX0); // For ATtiny84
#else
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);  // For ATmega328
#endif 
	delay(2); // Wait for Vref to settle
	ADCSRA |= _BV(ADSC); // Convert
	while (bit_is_set(ADCSRA, ADSC));
	result = ADCL;
	result |= ADCH << 8;
	result = 1126400L / result; // Back-calculate Vcc in mV
	ADCSRA &= ~bit(ADEN); bitSet(PRR, PRADC); // Disable the ADC to save power
	return result;
}

uint16_t readAnalogPin(byte analogPin)
{
	bitClear(PRR, PRADC); ADCSRA |= bit(ADEN); // Enable the ADC
	ADMUX = analogPin;

	ADCSRA |= _BV(ADSC); // Convert
	while (bit_is_set(ADCSRA, ADSC));

	uint16_t result = ADC;

	ADCSRA &= ~bit(ADEN); bitSet(PRR, PRADC); // Disable the ADC to save power
	return result;
}

uint16_t readHallValue()
{
	digitalWrite(SENSOR_POWER, HIGH);
	delayMicroseconds(50);

	uint16_t hallValue = readAnalogPin(SENSOR_PIN);

	digitalWrite(SENSOR_POWER, LOW);
	return hallValue;
}

bool waitForAck(byte destNodeId)
{
#if defined(USESERIAL2)
	printf_P(PSTR("wait_ack ..."));
#endif
	int loops = 0;
	MilliTimer ackTimer;
	while (!ackTimer.poll(ACK_TIME))
	{
		loops++;

		if (!rf12_recvDone())
			continue;

		byte crc = rf12_crc;
		byte hdr = rf12_hdr;
		byte len = rf12_len;
		byte rssi = (RF69::rssi >> 1);

#if defined(USESERIAL2)
		printf_P(PSTR(" r"));
#endif

		if (crc != 0)
		{
#if defined(USESERIAL2)
			printf_P(PSTR(" err\n"));
#endif
			continue;
		}

#if defined(USESERIAL2)
		printf_P(PSTR(" ok hdr=%02X"), hdr);
#endif

		if (hdr != (RF12_HDR_CTL | RF12_HDR_DST | myNodeID))
		{
#if defined(USESERIAL2)
			printf_P(PSTR(" notForMe\n"));
#endif
			continue;
		}

		data.remoteRssi = rssi;

		if (len > 0)
		{
#if defined(USESERIAL2)
			printf_P(PSTR(" len=%u "), len);
#endif

			if (len == sizeof(DataAckPacket))
			{
				triggerSend = true;
				uint32_t count, dCount;

				DataAckPacket* ackPacket = (DataAckPacket*)rf12_data;

				if (ackPacket->totalVolume != 0)
				{
					count = ackPacket->totalVolume / 10; // volume to 10l units
					dCount = count / DEVIDER - gcCounter;
					gcCounter += dCount;
					gcTodayCounter0 += dCount;
					gcFraction = count % DEVIDER; // fraction
					gcSignal = 0;
				}

				if (ackPacket->todayVolume != 0)
				{
					if (ackPacket->todayVolume == -1)
					{
						gcYesterdayVolume = (gcCounter - gcTodayCounter0) * 100;
						dCount = 0;
					}
					else
					{
						dCount = ackPacket->todayVolume / 100; // volume to 100l units
					}

					gcTodayCounter0 = gcCounter - dCount;
				}

				if (ackPacket->yesterdayVolume != 0)
				{
					gcYesterdayVolume = ackPacket->yesterdayVolume;
				}

				if (ackPacket->thresHigh != 0)
				{
					gcSensorValueHighThreshold = ackPacket->thresHigh;
					saveConfig();
				}

				if (ackPacket->thresLow != 0)
				{
					gcSensorValueLowThreshold = ackPacket->thresLow;
					saveConfig();
				}
			}
			else
			{
#if defined(USESERIAL2)
				printf_P(PSTR("aErr"));
#endif
			}
		}

#if defined(USESERIAL2)
		printf_P(PSTR(" match. loops=%d"), loops);
#endif
		return true;
	}

#if defined(USESERIAL2)
	printf_P(PSTR(" timeout. loops=%d"), loops);
#endif

#if defined(USESERIAL) || defined(USESERIAL2)
	Serial.println();
#endif
	return false;
}

bool sendTo(byte destNodeId, bool requestAck, void* data, byte datalen)
{
	bool acked = true;

	rf12_sleep(-1);

	while (!rf12_canSend())
		rf12_recvDone();

	rf12_sendStart(
		((destNodeId != 0) ? (destNodeId | RF12_HDR_DST) : 0) |
		(requestAck ? RF12_HDR_ACK : 0),
		data, datalen);

	if (requestAck)
	{
		acked = waitForAck(destNodeId);
	}
	else
	{
		rf12_sendWait(0);
	}

	rf12_sleep(0);

#if defined(USESERIAL2)
	printf_P(PSTR(" sendTo=%u"), destNodeId);
	if (requestAck)
	{
		printf_P(PSTR(" ack=%d"), acked);
	}
	printf_P(PSTR("\n"));
#endif
	return acked;
}

bool UpdateGasCounterState()
{
	bool changed = false;

	gcSensorValue = readHallValue();
	gcSensorPrevState = gcSensorState;

	// do stats
	if (gcSensorValue < gcCurMin)
		gcCurMin = gcSensorValue;
	if (gcSensorValue > gcCurMax)
		gcCurMax = gcSensorValue;
	gcCurAvg += gcSensorValue;
	gcCurAvgN++;
	if (gcCurAvgN > AVGNMAX)
	{
		gcCurAvg = gcCurAvg / gcCurAvgN;
		gcCurAvgN = 1;
	}

	if (gcSensorState == 0)
	{
		// check state transition LOW -> HIGH
		if (gcSensorValue <= gcSensorValueLowThreshold)
		{
			gcSensorState = 1;

			gcFraction++;
			if (gcFraction == DEVIDER)
			{
				gcFraction = 0;
				gcCounter++;
				gcSignal = 1;
				changed = true;
			}

			gcHighMax = gcCurMax;
			gcHighAvg = gcCurAvg / gcCurAvgN;

			// reset stats
			gcCurMax = gcCurAvg = gcCurMin = gcSensorValue;
			gcCurAvgN = 1;
		}
	}
	else if (gcSensorState == 1)
	{
		// check state transition HIGH -> LOW
		if (gcSensorValue >= gcSensorValueHighThreshold)
		{
			gcSensorState = 0;
			if (gcFraction == DEVIDER / 2)
			{
				gcSignal = 0;
				changed = true;
			}

			gcLowAvg = gcCurAvg / gcCurAvgN;
			gcLowMin = gcCurMin;

			// reset stats
			gcCurMax = gcCurAvg = gcCurMin = gcSensorValue;
			gcCurAvgN = 1;
		}
	}
	else
	{
		// init state
		gcSensorState = (gcSensorValue <= gcSensorValueLowThreshold) ? 1 : 0;
		changed = true;
#if defined(USESERIAL2)
		printf_P(PSTR("\ninit S=%u\n"), gcSensorState);
#endif

		// init stats
		gcCurMax = gcCurAvg = gcCurMin = gcSensorValue;
		gcCurAvgN = 1;
	}

#if defined(USESERIAL)
	printf_P(PSTR(" H=%hu st=%u F=%u\r"),
		gcSensorValue, gcSensorState, gcFraction);
#endif

#if defined(LED_COUNTFLASHS)
	if (gcSensorState != gcSensorPrevState)
	{
		flashLED(10, 1);
	}
#endif

	return changed;
}

void setup()
{
	pinMode(LED, OUTPUT);
	digitalWrite(LED, HIGH);

	pinMode(SENSOR_POWER, OUTPUT);
	digitalWrite(SENSOR_POWER, LOW);

#if defined(USESERIAL) || defined(USESERIAL2)
	Serial.begin(38400);
	fdev_setup_stream(&out, printChar, NULL, _FDEV_SETUP_WRITE);
	stdout = &out;
	printf_P(PSTR("setup\n"));
#endif

	loadConfig();
	printConfig();

	rf12_initialize(myNodeID, freq, network);
	rf12_sleep(0);

	delay(1000);
	digitalWrite(LED, LOW);
}

void loop()
{
	for (int w = 0; w < WAITLOOPS; w++)
	{
		if (UpdateGasCounterState())
			break; // leave on state changes

		if (triggerSend)
			break;

		Sleepy::loseSomeTime(WAITINTERVAL);
	}

	triggerSend = false;

	data.signal = gcSignal;
	data.count = gcCounter;
	data.sensorMax = gcHighMax;
	data.sensorAvgHigh = gcHighAvg;
	data.sensorThresHigh = gcSensorValueHighThreshold;
	data.sensorThresLow = gcSensorValueLowThreshold;
	data.sensorAvgLow = gcLowAvg;
	data.sensorMin = gcLowMin;
	data.totalVolume = gcCounter * 100;
	data.todayVolume = (gcCounter - gcTodayCounter0) * 100;
	data.yesterdayVolume = gcYesterdayVolume;

	data.power = readVcc() * 10;

#if defined(USESERIAL)
	printf_P(PSTR(" S=%u C=%lu V=%lu Vd=%lu Vy=%lu U=%hu RSSI(-%d)\n"),
		data.signal, data.count,
		data.totalVolume, data.todayVolume, data.yesterdayVolume,
		data.power, data.remoteRssi);
#endif
#if defined(USESERIAL)
	printf_P(PSTR(" H=%hu h=%hu l=%hu L=%hu (tH=%hu tL=%hu)\n"),
		gcHighMax, gcHighAvg, gcLowAvg, gcLowMin,
		gcSensorValueHighThreshold, gcSensorValueLowThreshold);
#endif

#if defined(USESERIAL2)
	printf_P(PSTR("SEND\n"));
#endif
	word retryDelay = RETRYDELAY;
	byte retry = 0;
	bool success = false;

	while (true)
	{
		if (sendTo(0, true, &data, sizeof(data)))
		{
			success = true;
			break;
		}

		retry++;

		if (retry > RETRIES)
			break;

#if defined(USESERIAL)
		printf_P(PSTR(" RETRY %u ms\n"), retryDelay);
#endif
		Sleepy::loseSomeTime(retryDelay);
		retryDelay <<= 1;
	}

	if (success)
	{
#if defined(LED_SENDFLASHS)
		flashLED(10, 1);
#endif

#if defined(USESERIAL)
		printf_P(PSTR(" OK. R=%u\n"), retry);
#endif
	}
	else
	{
#if defined(LED_SENDFLASHS)
		flashLED(50, 3);
#endif

#if defined(USESERIAL)
		printf_P(PSTR(" FAILED (%u)\n"), RETRIES);
#endif
	}
}
