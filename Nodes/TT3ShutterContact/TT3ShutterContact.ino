#define USESERIAL
//#define USESERIAL2
//#define LEDFLASHS
#define LEDFLASHONCHANGE

#define RF69_COMPAT 1
#include <JeeLib.h>

#define LED 7
#if defined(__AVR_ATtiny84__) 
#define SENSOR 10
#else
#define SENSOR 8
#endif

#define myNodeID 25
#define network 99
#define freq RF12_868MHZ
#define ACK_TIME 50
#define RETRYDELAY (500 + myNodeID*5)
#define RETRIES 5
#define WAITLOOPS 15
#define WAITINTERVAL 60000

#define CMD_ContactState 20
#define CMD_PowerSupply 252
#define CMD_SenderRSSI 196

struct DataPacket
{
    byte powerCmd;
    uint16_t power;		// mV * 10
    byte senderRssiCmd;
    byte senderRssi;	// dB * -1
    byte contactStateCmd;
    byte contactState;

    DataPacket()
    {
        powerCmd = CMD_PowerSupply;
        senderRssiCmd = CMD_SenderRSSI;
        contactStateCmd = CMD_ContactState;
    }
};

DataPacket data;

ISR(WDT_vect)
{
    Sleepy::watchdogEvent();  // interrupt handler for JeeLabs Sleepy power saving
}

volatile int counter = 0;
ISR(PCINT0_vect)
{
    counter++;
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

#if defined(USESERIAL)
    Serial.begin(38400);
    Serial.println("setup");
#endif

    rf12_initialize(myNodeID, freq, network);
    rf12_sleep(RF12_SLEEP);

    pinMode(SENSOR, INPUT/*_PULLUP*/);
    PCMSK0 |= (1 << PCINT0);
#if defined(__AVR_ATtiny84__) 
    GIMSK |= (1 << PCIE0);
#else
    PCICR |= (1 << PCIE0);
#endif

    Sleepy::loseSomeTime(1000);
    digitalWrite(LED, LOW);
}

void loop()
{
    // MEASURE
#if defined(USESERIAL)
    Serial.println("MEASURE");
#endif
    int s = digitalRead(SENSOR);
    bool stateChanged = (s != data.contactState);
    data.contactState = s;

    // MEASURE VCC
    data.power = readVcc() * 10;

    // TRACE
#if defined(USESERIAL)
    Serial.print(" cnt=");
    Serial.print(counter, DEC);
    Serial.print(" State=");
    Serial.print(data.contactState, DEC);
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
        flashLED(10, 1);
#elif defined(LEDFLASHONCHANGE)
        if (stateChanged)
            flashLED(5, 1);
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
        flashLED(50, 3);
#elif defined(LEDFLASHONCHANGE)
        if (stateChanged)
            flashLED(50, 3);
#endif

#if defined(USESERIAL)
        Serial.print(" SEND ERR AFTER ");
        Serial.print(RETRIES, DEC);
        Serial.print(" RETRIES.");
        Serial.println();
#endif
    }

#if defined(USESERIAL)
    Serial.println("SLEEP");
#endif
    // SLEEP
    for (int w = 0; w < WAITLOOPS; w++)
    {
        s = digitalRead(SENSOR);
        if (s != data.contactState)
        {
#if defined(USESERIAL2)
            Serial.println("CONTACT STATE CHANGED.");
#endif
            break;
        }

        Sleepy::loseSomeTime(WAITINTERVAL);
    }
}
