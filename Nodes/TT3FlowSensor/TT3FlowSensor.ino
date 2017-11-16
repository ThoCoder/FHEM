//#define USESERIAL
//#define USESERIAL2
#define LED_SENDFLASHS

#define RF69_COMPAT 1
#include <JeeLib.h>

#define LED 7
#define SENSOR_PIN 9
#define SENSOR_POWER 10

#define myNodeID 22
#define network 99
#define freq RF12_868MHZ
#define ACK_TIME 50
#define RETRYDELAY 500
#define RETRIES 5
#define WAITTIMEOUT 300000

#define COUNTSPERLITRE 10300
#define PULSETIMEOUT 1000

// set counter (RF12Demo V12.1):            195,nnnnnnD22m
// set counter (ThoGateway::JeeLink V2.1):  22,195,nnnnnnDm
// nnnnnn .. flow counter value to set in (10300/L) units
//
// reset day counter (ThoGateway::JeeLink V2.1): 22,189,0Dm
//
byte FlowCounter_State = 0;
uint32_t FlowCounter_Value = 0;
uint32_t FlowCounter_ValueTick = 0;
uint32_t FlowCounter_Running = 0;
uint32_t FlowCounter_Q = 0; // millilitre (1000 * FlowCounter_Value / COUNTSPERLITRE)
uint32_t FlowCounter_dValue0 = 0;
uint32_t FlowCounter_dQ = 0;
uint32_t FlowCounter_dayValue0 = 0;
uint32_t FlowCounter_dayQ = 0;
uint32_t FlowCounter_yesterdayQ = 0;

#define CMD_Count 195
#define CMD_TotalVolumeML 191
#define CMD_DeltaVolumeML 190
#define CMD_TodayVolumeML 189
#define CMD_YesterdayVolumeML 188
#define CMD_PowerSupply 252
#define CMD_SenderRSSI 196

struct DataPacket
{
	byte countCmd;
	uint32_t count;
	byte totalVolumeCmd;
	uint32_t totalVolumeML;	// total volume in ml (count * 1000 / COUNTSPERLITRE)
	byte deltaVolumeCmd;
	uint32_t deltaVolumeML;	// delta volume in ml since last pump operation
	byte todayVolumeCmd;
	uint32_t todayVolumeML; // day counter
	byte yesterdayVolumeCmd;
	uint32_t yesterdayVolumeML; // previous day counter value
	byte powerCmd;
	uint16_t power;		// mV * 10
	byte senderRssiCmd;
	byte senderRssi;	// dB * -1

	DataPacket()
	{
		countCmd = CMD_Count;
		totalVolumeCmd = CMD_TotalVolumeML;
		deltaVolumeCmd = CMD_DeltaVolumeML;
		todayVolumeCmd = CMD_TodayVolumeML;
		yesterdayVolumeCmd = CMD_YesterdayVolumeML;
		powerCmd = CMD_PowerSupply;
		senderRssiCmd = CMD_SenderRSSI;
	}
};

struct DataAckPacket
{
	byte countCmd;		// CMD_Count to set counter (10300 = 1L)
						// CMD_TodayVolumeML to reset day counter
	uint32_t count;
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
				uint32_t dv;

				switch (ackPacket->countCmd)
				{
				case CMD_Count:
					dv = ackPacket->count - FlowCounter_Value;
					FlowCounter_Value += dv;
					FlowCounter_dValue0 += dv;
					FlowCounter_dayValue0 += dv;
#if defined(USESERIAL)
					Serial.print(" cnt=");
					Serial.print(FlowCounter_Value, DEC);
#endif
					break;

					// day value reset
				case CMD_TodayVolumeML:
					FlowCounter_yesterdayQ = FlowCounter_dayQ;
					FlowCounter_dayQ = 0;
					FlowCounter_dayValue0 = FlowCounter_Value;
#if defined(USESERIAL)
					Serial.print(" today=");
					Serial.print(FlowCounter_yesterdayQ, DEC);
#endif
					break;
				}
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

bool UpdateFlowCounterState()
{
	bool pumpStopped = false;

	uint32_t t = millis();
	byte state = digitalRead(SENSOR_PIN);

	if (state != FlowCounter_State)
	{
		FlowCounter_State = state;
		FlowCounter_Value++;
		FlowCounter_ValueTick = t;

		if (FlowCounter_Running == 0)
		{
#if defined(USESERIAL)
			Serial.println("PUMP ON");
#endif
			// start counting
			FlowCounter_Running = 1;
	}
}

	if (FlowCounter_Running == 1)
	{
		uint32_t dt = t - FlowCounter_ValueTick;

		if (dt >= PULSETIMEOUT)
		{
#if defined(USESERIAL)
			Serial.println("PUMP OFF");
#endif
			// stop counting
			FlowCounter_Running = 0;
			pumpStopped = true;

			FlowCounter_Q = FlowCounter_Value * 1000 / COUNTSPERLITRE;
			FlowCounter_dQ = (FlowCounter_Value - FlowCounter_dValue0) * 1000 / COUNTSPERLITRE;
			FlowCounter_dayQ = (FlowCounter_Value - FlowCounter_dayValue0) * 1000 / COUNTSPERLITRE;

			FlowCounter_dValue0 = FlowCounter_Value;
	}
	}

	return pumpStopped;
}

void setup()
{
	pinMode(LED, OUTPUT);
	digitalWrite(LED, HIGH);

	pinMode(SENSOR_POWER, OUTPUT);
	digitalWrite(SENSOR_POWER, HIGH);

#if defined(USESERIAL) || defined(USESERIAL2)
	Serial.begin(38400);
	Serial.println("setup");
#endif

	rf12_initialize(myNodeID, freq, network);
	rf12_sleep(0);

	delay(1000);
	digitalWrite(LED, LOW);

	FlowCounter_State = digitalRead(SENSOR_PIN);
}

void loop()
{
	uint32_t t0 = millis();

	while (true)
	{
		if (UpdateFlowCounterState())
			break; // leave when pump operation stopped

		uint32_t t = millis();
		uint32_t dt = t - t0;
		if (dt >= WAITTIMEOUT)
			break; // leave every WAITTIMEOUT interval
	}

	data.count = FlowCounter_Value;
	data.totalVolumeML = FlowCounter_Q;
	data.deltaVolumeML = FlowCounter_dQ;
	data.todayVolumeML = FlowCounter_dayQ;
	data.yesterdayVolumeML = FlowCounter_yesterdayQ;

#if defined(USESERIAL)
	Serial.println("MEASURE VCC ...");
#endif
	data.power = readVcc() * 10;

#if defined(USESERIAL)
	Serial.print(" C="); Serial.print(data.count, DEC);
	Serial.print(" Q="); Serial.print(data.totalVolumeML, DEC);
	Serial.print(" dQ="); Serial.print(data.deltaVolumeML, DEC);
	Serial.print(" dayQ="); Serial.print(data.todayVolumeML, DEC);
	Serial.print(" yesterdayQ="); Serial.print(data.yesterdayVolumeML, DEC);
	Serial.print(" V="); Serial.print(data.power, DEC);
	Serial.print(" RSSI(-"); Serial.print(data.senderRssi, DEC); Serial.print(")");
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
		delay(retryDelay);
		retryDelay <<= 1;
	}

	if (success)
	{
#if defined(LED_SENDFLASHS)
		flashLED(10, 1);
#endif

#if defined(USESERIAL)
		Serial.print(" SENT SUCCESSFULLY. R=");
		Serial.print(retry, DEC);
		Serial.println();
#endif
	}
	else
	{
#if defined(LED_SENDFLASHS)
		flashLED(50, 3);
#endif

#if defined(USESERIAL)
		Serial.print(" FAILED TO SEND AFTER ");
		Serial.print(RETRIES, DEC);
		Serial.print(" RETRIES.");
		Serial.println();
#endif
	}
}
