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
#define WAITTIMEOUT_INIT 10000
#define WAITTIMEOUT_LONG 300000
#define WAITTIMEOUT_SHORT 60000
#define MEASURE_INTERVAL 50
#define MIDLEVEL 250
#define THRESHOLD 50

bool triggerSend = false;
uint32_t waitTimeout = WAITTIMEOUT_INIT;

// ThoGateway::JeeLink V2.1
//   reset day counter : <nodeID>,189,0Dm
//   set counter       : <nodeID>,195,nnnnnnDm
//                       nnnnnn .. counter value to set in L units

uint16_t esSensorValue;
byte esSensorPrevState = 255;
byte esSensorState = 255;
uint16_t esSensorValueHighThreshold = MIDLEVEL + THRESHOLD;
uint16_t esSensorValueLowThreshold = MIDLEVEL - THRESHOLD;
uint32_t esCounter = 0;
uint32_t esTodayCounter0 = 0;
uint32_t esDeltaCounter0 = 0;
uint32_t esDeltaCounter0Tick = 0;
uint32_t esYesterdayEnergy = 0;

uint16_t stHighMax = MIDLEVEL + THRESHOLD;
uint16_t stHighAvg = MIDLEVEL + THRESHOLD;
uint16_t stLowAvg = MIDLEVEL - THRESHOLD;
uint16_t stLowMin = MIDLEVEL - THRESHOLD;
uint16_t stCurMax, stCurMin;
uint32_t stCurAvg;
uint16_t stCurAvgN = 1;
#define AVGNMAX 8500

#define CMD_Count 195
#define CMD_TotalEnergy 178
#define CMD_TodayEnergy 177
#define CMD_YesterdayEnergy 176
#define CMD_CurrentPower 175
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
	byte countCmd;
	uint32_t count;				// 75 = 1000Wh
	byte totalEnergyCmd;
	uint32_t totalEnergy;		// total energy in Wh
	byte todayEnergyCmd;
	uint32_t todayEnergy;		// day counter
	byte yesterdayEnergyCmd;
	uint32_t yesterdayEnergy;	// previous day counter
	byte currentPowerCmd;
	uint16_t currentPower;		// Watts
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
		countCmd = CMD_Count;
		totalEnergyCmd = CMD_TotalEnergy;
		todayEnergyCmd = CMD_TodayEnergy;
		yesterdayEnergyCmd = CMD_YesterdayEnergy;
		currentPowerCmd = CMD_CurrentPower;
		powerCmd = CMD_PowerSupply;
		remoteRssiCmd = CMD_RemoteRSSI;
	}
};

struct DataAckPacket
{
	uint32_t totalEnergy;
	uint32_t todayEnergy;
	uint32_t yesterdayEnergy;
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
	uint16_t esSensorValueHighThreshold;
	uint16_t esSensorValueLowThreshold;
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
		esSensorValueHighThreshold = config.esSensorValueHighThreshold;
		esSensorValueLowThreshold = config.esSensorValueLowThreshold;
	}
}

void saveConfig()
{
	config.magic = CONFIG_MAGIC;
	config.esSensorValueHighThreshold = esSensorValueHighThreshold;
	config.esSensorValueLowThreshold = esSensorValueLowThreshold;

	eeprom_update_block(&config, CONFIG_EEPROM_ADDR, sizeof(config));
}

void printConfig()
{
	printf_P(PSTR("tH: %hu\n"), esSensorValueHighThreshold);
	printf_P(PSTR("tL: %hu\n"), esSensorValueLowThreshold);
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
	delayMicroseconds(250);

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

	esSensorValue = readSensorValue();
	esSensorPrevState = esSensorState;

	// do stats
	if (esSensorValue < stCurMin)
		stCurMin = esSensorValue;
	if (esSensorValue > stCurMax)
		stCurMax = esSensorValue;
	stCurAvg += esSensorValue;
	stCurAvgN++;
	if (stCurAvgN > AVGNMAX)
	{
		stCurAvg = stCurAvg / stCurAvgN;
		stCurAvgN = 1;
	}

	if (esSensorState == 0)
	{
		// check state transition LOW -> HIGH
		if (esSensorValue >= esSensorValueHighThreshold)
		{
			esSensorState = 1;

			esCounter++;
			changed = true;

#if defined(USESERIAL2)
			printf_P(PSTR("\nS=1\n"));
#endif
			stHighMax = stCurMax;
			stHighAvg = stCurAvg / stCurAvgN;

			// reset stats
			stCurMax = stCurAvg = stCurMin = esSensorValue;
			stCurAvgN = 1;
		}
	}
	else if (esSensorState == 1)
	{
		// check state transition HIGH -> LOW
		if (esSensorValue <= esSensorValueLowThreshold)
		{
			esSensorState = 0;

#if defined(USESERIAL2)
			printf_P(PSTR("\nS=0\n"));
#endif

			stLowAvg = stCurAvg / stCurAvgN;
			stLowMin = stCurMin;

			// reset stats
			stCurMax = stCurAvg = stCurMin = esSensorValue;
			stCurAvgN = 1;
		}
	}
	else
	{
		// init state
		esSensorState = (esSensorValue >= esSensorValueHighThreshold) ? 1 : 0;
		changed = true;
#if defined(USESERIAL2)
		printf_P(PSTR("\ninit S=%u\n"), esSensorState);
#endif

		// init stats
		stCurMax = stCurAvg = stCurMin = esSensorValue;
		stCurAvgN = 1;
	}

#if defined(USESERIAL)
	printf_P(PSTR(" H=%d S=%u RTT=%lu\r"), esSensorValue, esSensorState, rtt);
#endif

#if defined(LED_COUNTFLASHS)
	if (esSensorState != esSensorPrevState)
	{
		flashLED(10, 1);
	}
#endif

	return changed;
}

uint32_t Count2Energy(uint32_t count)
{
	return (uint32_t)(((uint64_t)count * 1000) / 75);
	//return (uint32_t)((float)count * WHPERCOUNT);
}

uint32_t Energy2Count(uint32_t volume)
{
	return (uint32_t)(((uint64_t)volume * 75) / 1000);
	//return (uint32_t)((float)volume * COUNTSPERWH);
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
				uint32_t dCount;

				if (ackPacket->totalEnergy != 0)
				{
					dCount = Energy2Count(ackPacket->totalEnergy) - esCounter;
					esCounter += dCount;
					esTodayCounter0 += dCount;
					esDeltaCounter0 += dCount;
				}

				if (ackPacket->todayEnergy != 0)
				{
					if (ackPacket->todayEnergy == -1)
					{
						esYesterdayEnergy = Count2Energy(esCounter - esTodayCounter0);
						esTodayCounter0 = esCounter;
					}
					else
					{
						esTodayCounter0 = esCounter - Energy2Count(ackPacket->todayEnergy);
					}
				}

				if (ackPacket->yesterdayEnergy != 0)
				{
					esYesterdayEnergy = ackPacket->yesterdayEnergy;
				}

				if (ackPacket->thresHigh != 0)
				{
					esSensorValueHighThreshold = ackPacket->thresHigh;
					saveConfig();
				}

				if (ackPacket->thresLow != 0)
				{
					esSensorValueLowThreshold = ackPacket->thresLow;
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
	printf_P(PSTR("\n"));
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
	uint32_t t;

	while (true)
	{
		if (UpdateCounterState())
			break; // send on counter change

		if (triggerSend)
			break;

		t = millis();
		uint32_t dt = t - t0;
		if (dt >= waitTimeout)
			break; // send every WAITTIMEOUT interval

		delay(MEASURE_INTERVAL);
	}

	triggerSend = false;

	data.sensorMax = stHighMax;
	data.sensorAvgHigh = stHighAvg;
	data.sensorThresHigh = esSensorValueHighThreshold;
	data.sensorThresLow = esSensorValueLowThreshold;
	data.sensorAvgLow = stLowAvg;
	data.sensorMin = stLowMin;
	data.count = esCounter;
	data.totalEnergy = Count2Energy(esCounter);
	data.todayEnergy = Count2Energy(esCounter - esTodayCounter0);
	data.yesterdayEnergy = esYesterdayEnergy;
	
	data.currentPower = 0; 
	esDeltaCounter0 = esCounter; 
	esDeltaCounter0Tick = t;

	data.power = readVcc() * 10;

#if defined(USESERIAL)
	printf_P(PSTR("\n E=%lu Ed=%lu Ey=%lu U=%hu RSSI(-%d)\n"),
		data.totalEnergy, data.todayEnergy, data.yesterdayEnergy,
		data.power, data.remoteRssi);
#endif
#if defined(USESERIAL)
	printf_P(PSTR(" H=%hu h=%hu l=%hu L=%hu (th=%hu tl=%hu)\n"),
		stHighMax, stHighAvg, stLowAvg, stLowMin,
		esSensorValueHighThreshold, esSensorValueLowThreshold);
#endif

#if defined(USESERIAL2)
	printf_P(PSTR("SEND\n"));
#endif
	if (sendTo(0, true, &data, sizeof(data)))
	{
#if defined(LED_SENDFLASHS)
		flashLED(10, 1);
#endif
#if defined(USESERIAL)
		printf_P(PSTR(" OK.\n"));
#endif
		waitTimeout = WAITTIMEOUT_LONG;
	}
	else
	{
#if defined(LED_SENDFLASHS)
		flashLED(50, 3);
#endif
#if defined(USESERIAL)
		printf_P(PSTR(" ERR\n"));
#endif
		waitTimeout = WAITTIMEOUT_LONG;
	}
}
