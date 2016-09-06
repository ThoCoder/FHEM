//#define USESERIAL
//#define USESERIAL2
#define LEDFLASHS

#define RF69_COMPAT 1
#include <JeeLib.h>

#define LED 7
#define SENSOR_PIN 1//9
#define SENSOR_POWER 10

#define myNodeID 10
#define network 99
#define freq RF12_868MHZ
#define ACK_TIME 50
#define RETRYDELAY 500
#define RETRIES 5
#define WAITLOOPS 60
#define WAITINTERVAL 5000

// set counter (RF12Demo V12.1)
// 195,nnnnnnD10m

short GasCounter_HallValue;
byte GasCounter_HallPrevState = 255;
byte GasCounter_HallState = 255;
short GasCounter_HallValueHighThreshold = 470;
short GasCounter_HallValueLowThreshold = 430;
long GasCounter_Value = 0;
bool GasCounter_ValueReset = false;

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
	long count;		// +1 = 0.1m3 = 100L
	byte powerCmd;
	uint16_t power;		// mV * 10
	byte senderRssiCmd;
	byte senderRssi;	// dB * -1

	DataPacket()
	{
		hallCmd = CMD_Hall;
		signalCmd = CMD_Signal;
		countCmd = CMD_Count;
		powerCmd = CMD_PowerSupply;
		senderRssiCmd = CMD_SenderRSSI;
	}
};

struct DataAckPacket
{
	byte countCmd;
	long count;		// +1 = 0.1m3 = 100L

	DataAckPacket()
	{
		countCmd = CMD_Count;
	}
};

DataPacket data;

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

short readHallValue()
{
	digitalWrite(SENSOR_POWER, HIGH);
	delayMicroseconds(50);

	short hallValue = readAnalogPin(SENSOR_PIN);

	digitalWrite(SENSOR_POWER, LOW);
	return hallValue;
}

bool UpdateGasCounterState()
{
	bool changed = false;

#if defined(USESERIAL)
	Serial.print("MEASURE HALL ... ");
#endif

	GasCounter_HallValue = readHallValue();
	GasCounter_HallPrevState = GasCounter_HallState;

#if defined(USESERIAL)
	Serial.print(" H=");
	Serial.print(GasCounter_HallValue, DEC);
	Serial.print(" S=");
	Serial.print(GasCounter_HallState, DEC);
#endif

	if (GasCounter_HallState == 0)
	{
		// check state transition LOW -> HIGH
		if (GasCounter_HallValue <= GasCounter_HallValueLowThreshold)
		{
			GasCounter_HallState = 1;
			changed = true;

#if defined(USESERIAL)
			Serial.print(" -> S=1");
#endif
		}
	}
	else if (GasCounter_HallState == 1)
	{
		// check state transition HIGH -> LOW
		if (GasCounter_HallValue >= GasCounter_HallValueHighThreshold)
		{
			GasCounter_HallState = 0;
			GasCounter_Value++;
			changed = true;

#if defined(USESERIAL)
			Serial.print(" -> S=0 C=");
			Serial.print(GasCounter_Value, DEC);
#endif
		}
	}
	else
	{
		// init state
		if (GasCounter_HallValue <= GasCounter_HallValueLowThreshold)
		{
			GasCounter_HallState = 1;
			changed = true;

#if defined(USESERIAL)
			Serial.print(" init S=1");
#endif
		}
		else if (GasCounter_HallValue >= GasCounter_HallValueHighThreshold)
		{
			GasCounter_HallState = 0;
			changed = true;

#if defined(USESERIAL)
			Serial.print(" init S=0");
#endif
		}
	}

#if defined(USESERIAL)
	Serial.println();
#endif

	if (GasCounter_HallState != GasCounter_HallPrevState)
	{
		flashLED(20, 1);
	}

	return changed;
}

bool waitForAck(byte destNodeId)
{
#if defined(USESERIAL2)
	Serial.print("wait_ack ...");
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
		Serial.print(" r");
#endif

		if (crc != 0)
		{
#if defined(USESERIAL2)
			Serial.println(" err");
#endif
			continue;
		}

#if defined(USESERIAL2)
		Serial.print(" ok hdr=");
		Serial.print(hdr, DEC);
#endif

		if (hdr != (RF12_HDR_CTL | RF12_HDR_DST | myNodeID))
		{
#if defined(USESERIAL2)
			Serial.println(" notForMe");
#endif
			continue;
		}

		data.senderRssi = rssi;

		if (len > 0)
		{
#if defined(USESERIAL)
			Serial.print(" len=");
			Serial.print(len, DEC);
#endif

			if (len == sizeof(DataAckPacket))
			{
				DataAckPacket* ackPacket = (DataAckPacket*)rf12_data;
				//if (ackPacket->countCmd == CMD_Count)
				//{
					GasCounter_Value = ackPacket->count;
					GasCounter_ValueReset = true;
#if defined(USESERIAL)
					Serial.print(" cnt=");
					Serial.print(GasCounter_Value, DEC);
#endif
//				}
//				else
//				{
//#if defined(USESERIAL2)
//					Serial.print(" wrongAckCmd");
//#endif
//				}
			}
			else
			{
#if defined(USESERIAL)
				Serial.print(" ackSizeMismatch");
#endif
			}
		}

#if defined(USESERIAL2)
		Serial.print(" match. loops=");
		Serial.println(loops, DEC);
#endif
		return true;
	}

#if defined(USESERIAL2)
	Serial.print(" timeout. loops=");
	Serial.println(loops, DEC);
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
	Serial.print(" sendTo=");
	Serial.print(destNodeId, DEC);
	if (requestAck)
	{
		Serial.print(" ack=");
		Serial.print(acked, DEC);
	}
	Serial.println();
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
	Serial.println("setup");
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

		if (GasCounter_ValueReset)
		{
			GasCounter_ValueReset = false;
			break;
		}

		Sleepy::loseSomeTime(WAITINTERVAL);
	}

	data.hall = GasCounter_HallValue;
	data.signal = GasCounter_HallState;
	data.count = GasCounter_Value;

#if defined(USESERIAL)
	Serial.println("MEASURE VCC ...");
#endif
	data.power = readVcc() * 10;

#if defined(USESERIAL)
	Serial.print(" H=");
	Serial.print(data.hall, DEC);
	Serial.print(" S=");
	Serial.print(data.signal, DEC);
	Serial.print(" C=");
	Serial.print(data.count, DEC);
	Serial.print(" V=");
	Serial.print(data.power, DEC);
	Serial.print(" RSSI(-");
	Serial.print(data.senderRssi, DEC);
	Serial.print(")");
	Serial.println();
#endif

#if defined(USESERIAL)
	Serial.println("SEND ...");
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
		Serial.print(" SEND FAILED. RETRY IN ");
		Serial.print(retryDelay, DEC);
		Serial.print("ms");
		Serial.println();
#endif
		Sleepy::loseSomeTime(retryDelay);
		retryDelay <<= 1;
	}

	if (success)
	{
#if defined(LEDFLASHS)
		flashLED(20, 1);
#endif

#if defined(USESERIAL)
		Serial.print(" SENT SUCCESSFULLY. R=");
		Serial.print(retry, DEC);
		Serial.println();
#endif
	}
	else
	{
#if defined(LEDFLASHS)
		flashLED(100, 3);
#endif

#if defined(USESERIAL)
		Serial.print(" FAILED TO SEND AFTER ");
		Serial.print(RETRIES, DEC);
		Serial.print(" RETRIES.");
		Serial.println();
#endif
	}
}
