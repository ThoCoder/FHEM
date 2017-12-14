#define USESERIAL
//#define USESERIAL2
#define LED_SENDFLASHS

#define RF69_COMPAT 1
#include <JeeLib.h>
#include"avr\eeprom.h"

#define LED 7
#define SENSOR_PIN 7//3
#define SENSOR_POWER 10

#define myNodeID 23
#define network 99
#define freq RF12_868MHZ
#define ACK_TIME 50
#define RETRYDELAY 500
#define RETRIES 0
#define WAITTIMEOUT_INIT 10000
#define WAITTIMEOUT_LONG 300000
#define WAITTIMEOUT_SHORT 60000
#define MEASURE_INTERVAL 35
#define MIDLEVEL 400
#define THRESHOLD 40

bool triggerSend = false;
uint32_t waitTimeout = WAITTIMEOUT_INIT;

// ThoGateway::JeeLink V2.1
//   reset day counter : <nodeID>,189,0Dm
//   set counter       : <nodeID>,195,nnnnnnDm
//                       nnnnnn .. counter value to set in L units

uint16_t wcsSensorValue;
byte wcsSensorPrevState = 255;
byte wcsSensorState = 255;
uint16_t wcsSensorValueHighThreshold = MIDLEVEL + THRESHOLD;
uint16_t wcsSensorValueLowThreshold = MIDLEVEL - THRESHOLD;
uint32_t wcsTotalVolume = 0;
uint32_t wcsTodayVolume = 0;
uint32_t wcsYesterdayVolume = 0;

uint16_t stHighMax = MIDLEVEL + THRESHOLD;
uint16_t stHighAvg = MIDLEVEL + THRESHOLD;
uint16_t stLowAvg = MIDLEVEL - THRESHOLD;
uint16_t stLowMin = MIDLEVEL - THRESHOLD;
uint16_t stCurMax, stCurMin;
uint32_t stCurAvg;
uint16_t stCurAvgN = 1;
#define AVGNMAX 8500

#define CMD_TotalVolume 191
#define CMD_TodayVolume 189
#define CMD_YesterdayVolume 188
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
	uint32_t totalVolume;		// total volume in ml
	byte todayVolumeCmd;
	uint32_t todayVolume;		// day counter
	byte yesterdayVolumeCmd;
	uint32_t yesterdayVolume;	// previous day counter
	byte powerCmd;
	uint16_t power;		// mV * 10
	byte remoteRssiCmd;
	byte remoteRssi;	// dB * -1

	DataPacket()
	{
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
#define CONFIG_MAGIC 0x11223344
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
		wcsSensorValueHighThreshold = config.wcsSensorValueHighThreshold;
		wcsSensorValueLowThreshold = config.wcsSensorValueLowThreshold;
	}
}

void saveConfig()
{
	config.magic = CONFIG_MAGIC;
	config.wcsSensorValueHighThreshold = wcsSensorValueHighThreshold;
	config.wcsSensorValueLowThreshold = wcsSensorValueLowThreshold;

	eeprom_update_block(&config, CONFIG_EEPROM_ADDR, sizeof(config));
}

void printConfig()
{
	printf_P(PSTR("ThresHigh: %hu\n"), wcsSensorValueHighThreshold);
	printf_P(PSTR("ThresLow: %hu\n"), wcsSensorValueLowThreshold);
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

long readVcc()
{
	bitClear(PRR, PRADC); ADCSRA |= bit(ADEN); // Enable the ADC
	long result;
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

uint16_t readSensorValue()
{
	digitalWrite(SENSOR_POWER, HIGH);
	delayMicroseconds(150);

	uint16_t sensorValue = readAnalogPin(SENSOR_PIN);

	digitalWrite(SENSOR_POWER, LOW);
	return sensorValue;
}

uint32_t tu = 0;

bool UpdateCounterState()
{
	uint32_t t = millis();
	uint32_t rtt = t - tu;
	tu = t;

	bool changed = false;

	wcsSensorValue = readSensorValue();
	wcsSensorPrevState = wcsSensorState;

	// do stats
	if (wcsSensorValue < stCurMin)
		stCurMin = wcsSensorValue;
	if (wcsSensorValue > stCurMax)
		stCurMax = wcsSensorValue;
	stCurAvg += wcsSensorValue;
	stCurAvgN++;
	if (stCurAvgN > AVGNMAX)
	{
		stCurAvg = stCurAvg / stCurAvgN;
		stCurAvgN = 1;
	}

	if (wcsSensorState == 0)
	{
		// check state transition LOW -> HIGH
		if (wcsSensorValue <= wcsSensorValueLowThreshold)
		{
			wcsSensorState = 1;

			wcsTotalVolume += 1000;
			wcsTodayVolume += 1000;
			changed = true;

#if defined(USESERIAL)
			printf_P(PSTR("\nS=1\n"));
#endif
			stHighMax = stCurMax;
			stHighAvg = stCurAvg / stCurAvgN;

			// reset stats
			stCurMax = stCurAvg = stCurMin = wcsSensorValue;
			stCurAvgN = 1;
		}
	}
	else if (wcsSensorState == 1)
	{
		// check state transition HIGH -> LOW
		if (wcsSensorValue >= wcsSensorValueHighThreshold)
		{
			wcsSensorState = 0;

#if defined(USESERIAL)
			printf_P(PSTR("\nS=0\n"));
#endif

			stLowAvg = stCurAvg / stCurAvgN;
			stLowMin = stCurMin;

			// reset stats
			stCurMax = stCurAvg = stCurMin = wcsSensorValue;
			stCurAvgN = 1;
		}
	}
	else
	{
		// init state
		wcsSensorState = (wcsSensorValue <= wcsSensorValueLowThreshold) ? 1 : 0;
		changed = true;
#if defined(USESERIAL2)
		printf_P(PSTR("\ninit S=%u\n"), wcsSensorState);
#endif

		// init stats
		stCurMax = stCurAvg = stCurMin = wcsSensorValue;
		stCurAvgN = 1;
	}

#if defined(USESERIAL)
	printf_P(PSTR(" H=%d RTT=%lu\r"), wcsSensorValue, rtt);
#endif

#if defined(LED_COUNTFLASHS)
	if (wcsSensorState != wcsSensorPrevState)
	{
		flashLED(10, 1);
	}
#endif

	return changed;
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
			printf_P(PSTR(" err"));
#endif
			continue;
		}

#if defined(USESERIAL2)
		printf_P(PSTR(" ok hdr=%02X"), hdr);
#endif

		if (hdr != (RF12_HDR_CTL | RF12_HDR_DST | myNodeID))
		{
#if defined(USESERIAL2)
			printf_P(PSTR(" notMe"));
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

				DataAckPacket* ackPacket = (DataAckPacket*)rf12_data;

				if (ackPacket->totalVolume != 0)
				{
					wcsTotalVolume = ackPacket->totalVolume;
				}

				if (ackPacket->todayVolume != 0)
				{
					if (ackPacket->todayVolume == -1)
					{
						wcsYesterdayVolume = wcsTodayVolume;
						wcsTodayVolume = 0;
					}
					else
					{
						wcsTodayVolume = ackPacket->todayVolume;
					}
				}

				if (ackPacket->yesterdayVolume != 0)
				{
					wcsYesterdayVolume = ackPacket->yesterdayVolume;
				}

				if (ackPacket->thresHigh != 0)
				{
					wcsSensorValueHighThreshold = ackPacket->thresHigh;
					saveConfig();
				}

				if (ackPacket->thresLow != 0)
				{
					wcsSensorValueLowThreshold = ackPacket->thresLow;
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

void setup()
{
	pinMode(LED, OUTPUT);
	digitalWrite(LED, HIGH);

	//	pinMode(SENSOR_PIN, INPUT);
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
	uint32_t t0 = millis();

	while (true)
	{
		if (UpdateCounterState())
			break; // send on counter change

		if (triggerSend)
			break;

		uint32_t t = millis();
		uint32_t dt = t - t0;
		if (dt >= waitTimeout)
			break; // send every WAITTIMEOUT interval

		delay(MEASURE_INTERVAL);
	}

	triggerSend = false;

	data.sensorMax = stHighMax;
	data.sensorAvgHigh = stHighAvg;
	data.sensorThresHigh = wcsSensorValueHighThreshold;
	data.sensorThresLow = wcsSensorValueLowThreshold;
	data.sensorAvgLow = stLowAvg;
	data.sensorMin = stLowMin;
	data.totalVolume = wcsTotalVolume;
	data.todayVolume = wcsTodayVolume;
	data.yesterdayVolume = wcsYesterdayVolume;

	data.power = readVcc() * 10;

#if defined(USESERIAL)
	printf_P(PSTR("\n V=%lu dayV=%lu ydayV=%lu U=%hu RSSI(-%d)\n"),
		data.totalVolume, data.todayVolume, data.yesterdayVolume,
		data.power, data.remoteRssi);
#endif
#if defined(USESERIAL)
	printf_P(PSTR(" H=%hu h=%hu l=%hu L=%hu (th=%hu tl=%hu)\n"),
		stHighMax, stHighAvg, stLowAvg, stLowMin,
		wcsSensorValueHighThreshold, wcsSensorValueLowThreshold);
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
		delay(retryDelay);
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

	waitTimeout = success ? WAITTIMEOUT_LONG : WAITTIMEOUT_SHORT;
}
