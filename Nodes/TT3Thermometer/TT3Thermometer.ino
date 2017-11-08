//#define USESERIAL
//#define USESERIAL2
//#define LEDFLASHS

#define DHT22present 1
//#define BMP180present 1
#define RF69_COMPAT 1
#include <JeeLib.h>

#define LED 7

#if defined(DHT22present) && defined(BMP180present)
#define DHT22_POWER 3
#define DHT22_PIN 8
#define BMP180_SCL 9
#define BMP180_SDA 10
#elif defined(DHT22present)
#define DHT22_PIN 9
#define DHT22_POWER 10
#elif defined(BMP180present)
#define BMP180_SCL 9
#define BMP180_SDA 10
#endif

#define myNodeID 23
#define network 99
#define freq RF12_868MHZ
#define ACK_TIME 50
#define RETRYDELAY 500
#define RETRIES 5
#define WAITLOOPS 5
#define WAITINTERVAL 59000

#define CMD_temperature 11
#define CMD_temperature2 13
#define CMD_pressure 14
#define CMD_humidity 16
#define CMD_PowerSupply 252
#define CMD_SenderRSSI 196

#if defined(DHT22present)
#define DHT22_NO_FLOAT
#include <DHT22.h>
DHT22 dht22(DHT22_PIN);
#endif

#if defined(BMP180present)
#include <PortsBMP085.h>
PortI2C i2c(1); // BMP085 SDA to D10 and SCL to D9
BMP085 bmp180(i2c, 3); // ultra high resolution
#endif

struct DataPacket
{
	byte powerCmd;
	uint16_t power;		// mV * 10
	byte senderRssiCmd;
	byte senderRssi;	// dB * -1
#if defined(DHT22present)
	byte tempCmd;
	int16_t temp;		// °C * 10
	byte humidityCmd;
	byte humidity;		// % * 1
#endif
#if defined(BMP180present)
	byte temp2Cmd;
	int16_t temp2;		// °C * 10
	byte pressureCmd;
	int16_t pressure;	// hPa * 10
#endif

	DataPacket()
	{
		powerCmd = CMD_PowerSupply;
		senderRssiCmd = CMD_SenderRSSI;
#if defined(DHT22present)
		tempCmd = CMD_temperature;
		humidityCmd = CMD_humidity;
#endif
#if defined(BMP180present)
		temp2Cmd = CMD_temperature2;
		pressureCmd = CMD_pressure;
#endif
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

#if defined(DHT22present)
	pinMode(DHT22_POWER, OUTPUT);
	digitalWrite(DHT22_POWER, LOW);
#endif

#if defined(USESERIAL)
	Serial.begin(38400);
	Serial.println("setup");
#endif

#if defined(BMP180present)
	Sleepy::loseSomeTime(50);
	bmp180.getCalibData();
#endif

	rf12_initialize(myNodeID, freq, network);
	rf12_sleep(RF12_SLEEP);

	Sleepy::loseSomeTime(1000);
	digitalWrite(LED, LOW);
}

void loop() 
{
	// MEASURE
#if defined(DHT22present)
#if defined(USESERIAL)
	Serial.println("DHT22");
#endif
	digitalWrite(DHT22_POWER, HIGH);
	Sleepy::loseSomeTime(2000); // DHT22 requires minimum 2s warm-up after power-on.
	DHT22_ERROR_t errorCode = dht22.readData();
	digitalWrite(DHT22_POWER, LOW);
#endif

#if defined(BMP180present)
#if defined(USESERIAL)
	Serial.println("BMP180");
#endif
	Sleepy::loseSomeTime(bmp180.startMeas(BMP085::TEMP));
	bmp180.getResult(BMP085::TEMP);

	Sleepy::loseSomeTime(bmp180.startMeas(BMP085::PRES));
	bmp180.getResult(BMP085::PRES);
#endif

	// CALCULATE
#if defined(DHT22present)
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
#endif

#if defined(BMP180present)
	int16_t temp2;
	int32_t pressure;
	bmp180.calculate(temp2, pressure);
	data.temp2 = temp2;
	data.pressure = pressure / 10;
#endif

	// MEASURE VCC
	data.power = readVcc() * 10;

	// TRACE
#if defined(USESERIAL)
#if defined(DHT22present)
	Serial.print(" T=");
	Serial.print(data.temp, DEC);
	Serial.print(" H=");
	Serial.print(data.humidity, DEC);
#endif
#if defined(BMP180present)
	Serial.print(" T2=");
	Serial.print(data.temp2, DEC);
	Serial.print(" P=");
	Serial.print(data.pressure, DEC);
#endif
	Serial.print(" V=");
	Serial.print(data.power, DEC);
	Serial.print(" RSSI(-");
	Serial.print(data.senderRssi, DEC);
	Serial.print(")");
	Serial.println();
#endif

	// SEND
#if defined(USESERIAL)
	Serial.println("SEND");
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
		Serial.print(" SENT OK. R=");
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
		Serial.print(" SEND ERR AFTER ");
		Serial.print(RETRIES, DEC);
		Serial.print(" RETRIES.");
		Serial.println();
#endif
	}

	// SLEEP
#if defined(USESERIAL)
	Serial.println("SLEEP");
#endif
	for (int w = 0; w < WAITLOOPS; w++)
		Sleepy::loseSomeTime(WAITINTERVAL);
}
