//#define USESERIAL
//#define USESERIAL2
//#define LEDFLASHS

#define RF69_COMPAT 1
#include <JeeLib.h>

#define DHT22_NO_FLOAT
#include <DHT22.h>

#define LED 7
#define DHT22_PIN 9
#define DHT22_POWER 10

#define myNodeID 14
#define network 99
#define freq RF12_868MHZ
#define ACK_TIME 50
#define RETRYDELAY 500
#define RETRIES 5
#define WAITLOOPS 5
#define WAITINTERVAL 59000

#define CMD_temperature 11
#define CMD_humidity 16
#define CMD_PowerSupply 252
#define CMD_SenderRSSI 196

DHT22 dht22(DHT22_PIN);

struct DataPacket
{
	byte tempCmd;
	int16_t temp;		// T * 10
	byte humidityCmd;
	byte humidity;		// H * 1
	byte powerCmd;
	uint16_t power;		// mV * 10
	byte senderRssiCmd;
	byte senderRssi;	// dB * -1

	DataPacket()
	{
		tempCmd = CMD_temperature;
		humidityCmd = CMD_humidity;
		powerCmd = CMD_PowerSupply;
		senderRssiCmd = CMD_SenderRSSI;
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

		if (len > 0)
		{
#if defined(USESERIAL2)
			Serial.println(" sizeMismatch");
#endif
			continue;
		}

		data.senderRssi = rssi;

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
	
	rf12_sleep(RF12_WAKEUP);

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

	rf12_sleep(RF12_SLEEP);

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

	pinMode(DHT22_POWER, OUTPUT);
	digitalWrite(DHT22_POWER, LOW);

#if defined(USESERIAL)
	Serial.begin(38400);
	Serial.println("setup");
#endif

	rf12_initialize(myNodeID, freq, network);
	rf12_sleep(RF12_SLEEP);

	delay(1000);
	digitalWrite(LED, LOW);
}

void loop() 
{
#if defined(USESERIAL)
	Serial.println("WAKE UP ...");
#endif
	digitalWrite(DHT22_POWER, HIGH);
	Sleepy::loseSomeTime(2000); // DHT22 requires minimum 2s warm-up after power-on.

#if defined(USESERIAL)
	Serial.println("MEASURE ...");
#endif
	DHT22_ERROR_t errorCode = dht22.readData();
	digitalWrite(DHT22_POWER, LOW);

	if (errorCode == DHT_ERROR_NONE)
	{
		data.temp = dht22.getTemperatureCInt();
		data.humidity = (dht22.getHumidityInt() / 10);
	}
	else
	{
		data.temp = -1000;
		data.humidity = 100 + errorCode;
	}

	data.power = readVcc() * 10;

#if defined(USESERIAL)
	Serial.print(" T=");
	Serial.print(data.temp, DEC);
	Serial.print(" H=");
	Serial.print(data.humidity, DEC);
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

#if defined(USESERIAL)
	Serial.println("SLEEP ...");
#endif
	for (int w = 0; w < WAITLOOPS; w++)
		Sleepy::loseSomeTime(WAITINTERVAL);
}
