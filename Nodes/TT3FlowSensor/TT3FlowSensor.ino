#define USESERIAL
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
#define WAITTIMEOUT_INIT 10000
#define WAITTIMEOUT_LONG 300000
#define WAITTIMEOUT_SHORT 60000

#define PULSETIMEOUT 1000
const float COUNTSPERMILLILITRE = 10.300;
const float MILLILITREPERCOUNT = 1.0 / COUNTSPERMILLILITRE;

bool triggerSend = false;
uint32_t waitTimeout = WAITTIMEOUT_INIT;

// ThoGateway::JeeLink V2.1
//   reset day counter : <nodeID>,189,0Dm
//   set counter       : <nodeID>,195,nnnnnnDm
//                       nnnnnn .. counter value to set in (10300/L) units

// all volume values are in ml calculated from counter values 
//   volume = 1000 * counter / COUNTSPERLITRE)

// Q values in ml/min (volumetric flow rate)

byte fsState = 0;
uint32_t fsCounter = 0;
uint32_t fsCounterTick = 0;
uint32_t fsCounterRunning = 0;
uint32_t fsCounterStartTick = 0;
uint32_t fsDeltaCounter0 = 0;
uint32_t fsQ = 0;

uint32_t fsTodayCounter0 = 0;
uint32_t fsYesterdayVolume = 0;

#define CMD_Count 195
#define CMD_TotalVolume 191
#define CMD_DeltaVolume 190
#define CMD_TodayVolume 189
#define CMD_YesterdayVolume 188
#define CMD_VolumePerMin 187
#define CMD_PowerSupply 252
#define CMD_RemoteRSSI 101

struct DataPacket
{
	byte countCmd;
	uint32_t count;
	byte totalVolumeCmd;
	uint32_t totalVolume;	// total volume in ml (count * 1000 / COUNTSPERLITRE)
	byte deltaVolumeCmd;
	uint32_t deltaVolume;	// delta volume in ml
	byte todayVolumeCmd;
	uint32_t todayVolume; // day counter
	byte yesterdayVolumeCmd;
	uint32_t yesterdayVolume; // previous day counter value
	byte volumePerMinCmd;
	uint32_t volumePerMin; // flow rate
	byte powerCmd;
	uint16_t power;		// mV * 10
	byte remoteRssiCmd;
	byte remoteRssi;	// dB * -1

	DataPacket()
	{
		countCmd = CMD_Count;
		totalVolumeCmd = CMD_TotalVolume;
		deltaVolumeCmd = CMD_DeltaVolume;
		todayVolumeCmd = CMD_TodayVolume;
		yesterdayVolumeCmd = CMD_YesterdayVolume;
		volumePerMinCmd = CMD_VolumePerMin;
		powerCmd = CMD_PowerSupply;
		remoteRssiCmd = CMD_RemoteRSSI;
	}
};

struct DataAckPacket
{
	uint32_t totalVolume;
	uint32_t todayVolume;
	uint32_t yesterdayVolume;
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
			printf_P(PSTR(" notForMe\n"));
#endif
			continue;
		}

		data.remoteRssi = rssi;

		if (len > 0)
		{
#if defined(USESERIAL2)
			printf_P(PSTR(" len=%u"), len);
#endif

			if (len == sizeof(DataAckPacket))
			{
				triggerSend = true;

				DataAckPacket* ackPacket = (DataAckPacket*)rf12_data;
				uint32_t dCount;
				
				if (ackPacket->totalVolume != 0)
				{
					dCount = Volume2Count(ackPacket->totalVolume) - fsCounter;
					fsCounter += dCount;
					fsDeltaCounter0 += dCount;
					fsTodayCounter0 += dCount;
				}

				if (ackPacket->todayVolume != 0)
				{
					if (ackPacket->todayVolume == -1)
					{
						fsYesterdayVolume = Count2Volume(fsCounter - fsTodayCounter0);
						dCount = 0;
					}
					else
					{
						dCount = Volume2Count(ackPacket->todayVolume);
					}

					fsTodayCounter0 = fsCounter - dCount;
				}

				if (ackPacket->yesterdayVolume != 0)
				{
					fsYesterdayVolume = ackPacket->yesterdayVolume;
				}
			}
			else
			{
#if defined(USESERIAL2)
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

bool UpdateCounterState()
{
	bool stateChanged = false;

	uint32_t t = millis();
	byte state = digitalRead(SENSOR_PIN);

	if (state != fsState)
	{
		fsState = state;
		fsCounter++;
		fsCounterTick = t;

		if (fsCounterRunning == 0)
		{
#if defined(USESERIAL)
			printf_P(PSTR("CTR ON\n"));
#endif
			// start counting
			fsCounterRunning = 1;
			fsCounterStartTick = fsCounterTick;

			//stateChanged = true;
		}
	}

	if (fsCounterRunning == 1)
	{
		uint32_t dt = t - fsCounterTick;

		if (dt >= PULSETIMEOUT)
		{
#if defined(USESERIAL)
			printf_P(PSTR("CTR OFF\n"));
#endif
			// stop counting
			fsCounterRunning = 0;
			stateChanged = true;
		}
	}

	return stateChanged;
}

uint32_t Count2Volume(uint32_t count)
{
	return (uint32_t)((float)count * MILLILITREPERCOUNT);
}

uint32_t Volume2Count(uint32_t volume)
{
	return (uint32_t)((float)volume * COUNTSPERMILLILITRE);
}

void setup()
{
	pinMode(LED, OUTPUT);
	digitalWrite(LED, HIGH);

	pinMode(SENSOR_POWER, OUTPUT);
	digitalWrite(SENSOR_POWER, HIGH);

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

	fsState = digitalRead(SENSOR_PIN);
}

void loop()
{
	uint32_t t0 = millis();

	while (true)
	{
		if (UpdateCounterState())
			break; // send when pump operation stopped

		if (fsCounterRunning == 1)
			continue; // do not send while counting

		if (triggerSend)
			break;

		uint32_t t = millis();
		uint32_t dt = t - t0;
		if (dt >= waitTimeout)
			break; // send every WAITTIMEOUT interval
	}

	triggerSend = false;

	data.count = fsCounter;
	data.totalVolume = Count2Volume(fsCounter);
	data.deltaVolume = Count2Volume(fsCounter - fsDeltaCounter0); 
	fsDeltaCounter0 = fsCounter;
	data.todayVolume = Count2Volume(fsCounter - fsTodayCounter0);
	data.yesterdayVolume = fsYesterdayVolume;
	uint32_t dt = fsCounterTick - fsCounterStartTick;
	data.volumePerMin = ((dt > 0) && (data.deltaVolume > 0)) ? 60000 * data.deltaVolume / dt : 0;

#if defined(USESERIAL2)
	printf_P(PSTR("MEASURE VCC ...\n"));
#endif
	data.power = readVcc() * 10;

#if defined(USESERIAL)
	printf_P(PSTR(" C=%lu V=%lu dV=%lu Vd=%lu Vy=%lu Q=%lu U=%hu RSSI(-%d)\n"),
		data.count,
		data.totalVolume, data.deltaVolume, data.todayVolume, data.yesterdayVolume,
		data.volumePerMin,
		data.power, data.remoteRssi);
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

	waitTimeout = (data.deltaVolume > 0) ? WAITTIMEOUT_SHORT : WAITTIMEOUT_LONG;
}
