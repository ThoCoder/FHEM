//#define USESERIAL
#define USESERIAL2
#define LED_SENDFLASHS

#define RF69_COMPAT 1
#include <JeeLib.h>

#define LED 7
#define SENSOR_PIN 9
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
#define MEASURE_INTERVAL 40

bool triggerSend = false;
uint32_t waitTimeout = WAITTIMEOUT_INIT;

// ThoGateway::JeeLink V2.1
//   reset day counter : <nodeID>,189,0Dm
//   set counter       : <nodeID>,195,nnnnnnDm
//                       nnnnnn .. counter value to set in L units

short wcsSensorValue;
byte wcsSensorPrevState = 255;
byte wcsSensorState = 255;
short wcsSensorValueHighThreshold = 400;
short wcsSensorValueLowThreshold = 300;
uint32_t wcsTotalVolume = 0;
uint32_t wcsTodayVolume = 0;
uint32_t wcsYesterdayVolume = 0;

#define CMD_Hall 193
#define CMD_TotalVolume 191
#define CMD_TodayVolume 189
#define CMD_YesterdayVolume 188
#define CMD_PowerSupply 252
#define CMD_SenderRSSI 196

struct DataPacket
{
	byte hallCmd;
	uint16_t hall;				// 0 .. 1023
	byte totalVolumeCmd;
	uint32_t totalVolume;		// total volume in ml
	byte todayVolumeCmd;
	uint32_t todayVolume;		// day counter
	byte yesterdayVolumeCmd;
	uint32_t yesterdayVolume;	// previous day counter value
	byte powerCmd;
	uint16_t power;		// mV * 10
	byte senderRssiCmd;
	byte senderRssi;	// dB * -1

	DataPacket()
	{
		hallCmd = CMD_Hall;
		totalVolumeCmd = CMD_TotalVolume;
		todayVolumeCmd = CMD_TodayVolume;
		yesterdayVolumeCmd = CMD_YesterdayVolume;
		powerCmd = CMD_PowerSupply;
		senderRssiCmd = CMD_SenderRSSI;
	}
};

struct DataAckPacket
{
	byte countCmd;		// CMD_TotalVolume to set total counter (ml)
						// CMD_TodayVolume to reset day counter
	uint32_t count;
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

short readAnalogPin(byte analogPin)
{
	bitClear(PRR, PRADC); ADCSRA |= bit(ADEN); // Enable the ADC
	ADMUX = analogPin;

	ADCSRA |= _BV(ADSC); // Convert
	while (bit_is_set(ADCSRA, ADSC));

	short result = ADC;

	ADCSRA &= ~bit(ADEN); bitSet(PRR, PRADC); // Disable the ADC to save power
	return result;
}

short readSensorValue()
{
	digitalWrite(SENSOR_POWER, HIGH);
	delayMicroseconds(50);

	short sensorValue = readAnalogPin(SENSOR_PIN);

	digitalWrite(SENSOR_POWER, LOW);
	return sensorValue;
}

bool UpdateCounterState()
{
	bool changed = false;

#if defined(USESERIAL)
	printf_P(PSTR("MEASURE SENSOR ... "));
#endif

	wcsSensorValue = readSensorValue();
	wcsSensorPrevState = wcsSensorState;

	if (wcsSensorState == 0)
	{
		// check state transition LOW -> HIGH
		if (wcsSensorValue <= wcsSensorValueLowThreshold)
		{
			wcsSensorState = 1;

			wcsTotalVolume += 1000;
			wcsTodayVolume += 1000;
			changed = true;
		}
	}
	else if (wcsSensorState == 1)
	{
		// check state transition HIGH -> LOW
		if (wcsSensorValue >= wcsSensorValueHighThreshold)
		{
			wcsSensorState = 0;
		}
	}
	else
	{
		// init state
		if (wcsSensorValue <= wcsSensorValueLowThreshold)
		{
			wcsSensorState = 1;
			changed = true;

#if defined(USESERIAL)
			printf_P(PSTR(" init S=1"));
#endif
		}
		else if (wcsSensorValue >= wcsSensorValueHighThreshold)
		{
			wcsSensorState = 0;
			changed = true;

#if defined(USESERIAL)
			printf_P(PSTR(" init S=0"));
#endif
		}
	}

#if defined(USESERIAL)
	printf_P(PSTR(" H=%d st=%d C=%lu\n"), wcsSensorValue, wcsSensorState, wcsTotalVolume);
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
			printf_P(PSTR(" notForMe"));
#endif
			continue;
		}

		data.senderRssi = rssi;

		if (len > 0)
		{
#if defined(USESERIAL)
			printf_P(PSTR(" len=%u"), len);
#endif

			if (len == sizeof(DataAckPacket))
			{
				DataAckPacket* ackPacket = (DataAckPacket*)rf12_data;
				uint32_t d;

				switch (ackPacket->countCmd)
				{
				case CMD_TotalVolume:
					wcsTotalVolume = ackPacket->count;
#if defined(USESERIAL)
					printf_P(PSTR(" total=%lu"), wcsTotalVolume);
#endif
					triggerSend = true;
					break;

					// day value reset
				case CMD_TodayVolume:
					wcsYesterdayVolume = wcsTodayVolume;
					wcsTodayVolume = 0;
#if defined(USESERIAL)
					printf_P(PSTR(" today=%lu"), wcsYesterdayVolume);
#endif
					triggerSend = true;
					break;
				}
			}
			else
			{
#if defined(USESERIAL)
				printf_P(PSTR(" ackSizeMismatch"));
#endif
			}
		}

#if defined(USESERIAL2)
		printf_P(PSTR(" match. loops=%d\n"), loops);
#endif
		return true;
	}

#if defined(USESERIAL2)
	printf_P(PSTR(" timeout. loops=%d\n"), loops);
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

#if defined(USESERIAL)
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

	pinMode(SENSOR_POWER, OUTPUT);
	digitalWrite(SENSOR_POWER, LOW);

#if defined(USESERIAL) || defined(USESERIAL2)
	Serial.begin(38400);
	fdev_setup_stream(&out, printChar, NULL, _FDEV_SETUP_WRITE);
	stdout = &out;
	printf_P(PSTR("setup\n"));
#endif

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

	data.hall = wcsSensorValue;
	data.totalVolume = wcsTotalVolume;
	data.todayVolume = wcsTodayVolume;
	data.yesterdayVolume = wcsYesterdayVolume;

#if defined(USESERIAL)
	printf_P(PSTR("MEASURE VCC ...\n"));
#endif
	data.power = readVcc() * 10;

#if defined(USESERIAL)
	printf_P(PSTR(" V=%lu dayV=%lu ydayV=%lu U=%hu RSSI(-%d)\n"),
		data.totalVolume, data.todayVolume, data.yesterdayVolume,
		data.power, data.senderRssi);
#endif

#if defined(USESERIAL)
	printf_P(PSTR("SEND ...\n"));
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
		printf_P(PSTR(" SEND FAILED. RETRY IN %u ms\n"), retryDelay);
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
		printf_P(PSTR(" SENT OK. R=%u\n"), retry);
#endif
	}
	else
	{
#if defined(LED_SENDFLASHS)
		flashLED(50, 3);
#endif

#if defined(USESERIAL)
		printf_P(PSTR(" FAILED TO SEND AFTER %u RETRIES.\n"), RETRIES);
#endif
	}

	waitTimeout = success ? WAITTIMEOUT_LONG : WAITTIMEOUT_SHORT;
}
