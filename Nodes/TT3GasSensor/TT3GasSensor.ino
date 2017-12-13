#define USESERIAL
//#define USESERIAL2
//#define LED_COUNTFLASHS
#define LED_SENDFLASHS

//#define RF69_COMPAT 1
#include <JeeLib.h>

#define LED 7
#define SENSOR_PIN 1//9
#define SENSOR_POWER 10

#define myNodeID 10
#define network 212
#define freq RF12_868MHZ
#define ACK_TIME 50
#define RETRYDELAY 500
#define RETRIES 5
#define WAITLOOPS 600
#define WAITINTERVAL 500
#define DEVIDER 10 

// set counter (RF12Demo V12.1):            195,nnnnnnD10m
// set counter (ThoGateway::JeeLink V2.1):  10,195,nnnnnnDm
// nnnnnn .. gas counter value to set in (0.1m^3 / DEVIDER) units

uint16_t gcHallValue;
byte gcHallPrevState = 255;
byte gcHallState = 255;
uint16_t gcHallValueHighThreshold = 480;
uint16_t gcHallValueLowThreshold = 450;
byte gcSignal = 0;
byte gcFraction = DEVIDER - 1;
uint32_t gcCounter = 0;
bool triggerSend = false;

uint32_t gcTotalVolume = 0;
uint32_t gcTodayCounter0 = 0;
uint32_t gcTodayVolume = 0;
uint32_t gcYesterdayVolume = 0;


#define CMD_TotalVolume 191
#define CMD_TodayVolume 189
#define CMD_YesterdayVolume 188

#define CMD_Hall 193
#define CMD_Signal 194
#define CMD_Count 195
#define CMD_PowerSupply 252
#define CMD_SenderRSSI 196

struct DataPacket
{
	byte hallCmd;
	uint16_t hall;		// 0 .. 1023
	byte signalCmd;
	byte signal;		// 0=off, 1=on
	byte countCmd;
	uint32_t count;		// +1 = 0.1m3 = 100L
	byte totalVolumeCmd;
	uint32_t totalVolume;	// total volume in l
	byte todayVolumeCmd;
	uint32_t todayVolume; // day counter
	byte yesterdayVolumeCmd;
	uint32_t yesterdayVolume; // previous day counter value
	byte powerCmd;
	uint16_t power;		// mV * 10
	byte senderRssiCmd;
	byte senderRssi;	// dB * -1

	DataPacket()
	{
		hallCmd = CMD_Hall;
		signalCmd = CMD_Signal;
		countCmd = CMD_Count;
		totalVolumeCmd = CMD_TotalVolume;
		todayVolumeCmd = CMD_TodayVolume;
		yesterdayVolumeCmd = CMD_YesterdayVolume;
		powerCmd = CMD_PowerSupply;
		senderRssiCmd = CMD_SenderRSSI;
	}
};

struct DataAckPacket
{
	byte countCmd;
	uint32_t count;

	DataAckPacket()
	{
		countCmd = CMD_Count;
	}
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

		data.senderRssi = rssi;

		if (len > 0)
		{
#if defined(USESERIAL2)
			printf_P(PSTR(" len=%u"), len);
#endif

			if (len == sizeof(DataAckPacket))
			{
				DataAckPacket* ackPacket = (DataAckPacket*)rf12_data;
				uint32_t d;
				uint32_t count;

				switch (ackPacket->countCmd)
				{
				case CMD_Count: // count in 10l units (0.01m3)
					gcCounter = ackPacket->count / DEVIDER; // 100l units
					gcFraction = ackPacket->count % DEVIDER; // fraction
					gcSignal = 0;
#if defined(USESERIAL)
					printf_P(PSTR(" cnt=%lu"), gcCounter);
#endif
					triggerSend = true;
					break;

				case CMD_TotalVolume: // volume in l
					count = ackPacket->count / 10; // volume to 10l units
					d = count / DEVIDER - gcCounter;
					gcCounter += d;
					gcFraction = count % DEVIDER; // fraction
					gcSignal = 0;
					gcTotalVolume = Count2Volume(gcCounter);
					gcTodayCounter0 += d;
					gcTodayVolume = Count2Volume(gcCounter - gcTodayCounter0);
#if defined(USESERIAL)
					printf_P(PSTR(" cnt=%lu V=%lu Vd=%lu"), gcCounter, gcTotalVolume, gcTodayVolume);
#endif
					triggerSend = true;
					break;

					// day value reset
				case CMD_TodayVolume: // volume in l
					d = Volume2Count(ackPacket->count);

					if (d == 0)
						gcYesterdayVolume = gcTodayVolume;

					gcTodayCounter0 = gcCounter - d;
					gcTodayVolume = Count2Volume(gcCounter - gcTodayCounter0);

#if defined(USESERIAL)
					printf_P(PSTR(" Vd=%lu Vy=%lu"), gcTodayVolume, gcYesterdayVolume);
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

#if defined(USESERIAL)
		printf_P(PSTR(" match. loops=%d\n"), loops);
#endif
		return true;
	}

#if defined(USESERIAL)
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

bool UpdateGasCounterState()
{
	bool changed = false;

#if defined(USESERIAL)
	printf_P(PSTR("MEASURE HALL ... "));
#endif

	gcHallValue = readHallValue();
	gcHallPrevState = gcHallState;

	if (gcHallState == 0)
	{
		// check state transition LOW -> HIGH
		if (gcHallValue <= gcHallValueLowThreshold)
		{
			gcHallState = 1;

			gcFraction++;
			if (gcFraction == DEVIDER)
			{
				gcFraction = 0;
				gcCounter++;
				gcSignal = 1;
				changed = true;
			}
		}
	}
	else if (gcHallState == 1)
	{
		// check state transition HIGH -> LOW
		if (gcHallValue >= gcHallValueHighThreshold)
		{
			gcHallState = 0;
			if (gcFraction == DEVIDER / 2)
			{
				gcSignal = 0;
				changed = true;
			}
		}
	}
	else
	{
		// init state
		if (gcHallValue <= gcHallValueLowThreshold)
		{
			gcHallState = 1;
			changed = true;

#if defined(USESERIAL)
			printf_P(PSTR(" init S=1\n"));
#endif
		}
		else if (gcHallValue >= gcHallValueHighThreshold)
		{
			gcHallState = 0;
			changed = true;

#if defined(USESERIAL)
			printf_P(PSTR(" init S=0\n"));
#endif
		}
	}

#if defined(USESERIAL)
	printf_P(PSTR(" H=%hu st=%u S=%u C=%lu,%u\n"), 
		gcHallValue, gcHallState, gcSignal, gcCounter, gcFraction);
#endif

#if defined(LED_COUNTFLASHS)
	if (gcHallState != gcHallPrevState)
	{
		flashLED(10, 1);
	}
#endif

	return changed;
}

uint32_t Count2Volume(uint32_t count)
{
	return count * 100;
}

uint32_t Volume2Count(uint32_t volume)
{
	return volume / 100;
}

void CalculateStatistics()
{
	gcTotalVolume = Count2Volume(gcCounter);
	gcTodayVolume = Count2Volume(gcCounter - gcTodayCounter0);
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
	for (int w = 0; w < WAITLOOPS; w++)
	{
		if (UpdateGasCounterState())
			break; // leave on state changes

		if (triggerSend)
			break;

		Sleepy::loseSomeTime(WAITINTERVAL);
	}

	CalculateStatistics();
	triggerSend = false;

	data.hall = gcHallValue;
	data.signal = gcSignal;
	data.count = gcCounter;
	data.totalVolume = gcTotalVolume;
	data.todayVolume = gcTodayVolume;
	data.yesterdayVolume = gcYesterdayVolume;

#if defined(USESERIAL2)
	printf_P(PSTR("MEASURE VCC ...\n"));
#endif
	data.power = readVcc() * 10;

#if defined(USESERIAL)
	printf_P(PSTR(" H=%hu S=%u C=%lu V=%lu Vd=%lu Vy=%lu U=%hu RSSI(-%d)\n"),
		data.hall, data.signal, data.count,
		data.totalVolume, data.todayVolume, data.yesterdayVolume,
		data.power, data.senderRssi);
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
