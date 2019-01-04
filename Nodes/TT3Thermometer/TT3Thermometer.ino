#define USESERIAL
//#define USESERIAL2
//#define LEDFLASHS

//#define DHT22present 1
//#define BMP180present 1
#define BME280present 1
#define BME280_5V 1
//#define BME280raw 1
#define RF69_COMPAT 1
//#define isHCW
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
#elif defined(BME280present)
#define BME280_I2Caddress 0x76
#if defined(__AVR_ATtiny84__)
#define BME280_POWER 3
#define BME280_SCL 9
#define BME280_SDA 10
#else
#define BME280_POWER 3
#define BME280_SCL PIN_A5
#define BME280_SDA PIN_A4
#endif
#endif

#define myNodeID 3
#define network 101
#define freq RF12_868MHZ
#define ACK_TIME 50
#define RETRYDELAY (599 - network + myNodeID*5)
#define RETRIES 5
#define WAITLOOPS 5
#define WAITINTERVAL 59000
#define MAXSENDSKIPS 6
#define MAXFAILURES 2
#define MINVCC 35000

int16_t sentT = -1000;
int16_t sentH = -1000;
int16_t sentP = -1000;
#define THRES_T 2
#define THRES_H 7
#define THRES_P 10
byte skipCount = 0;
byte failureCount = 0;
uint32_t measurementCount = 0;
uint32_t sendingCount = 0;
byte recoveryCount = 0;

#define CMD_temperature 11
#define CMD_temperature2 13
#define CMD_pressure 14
#define CMD_humidity 17
#define CMD_PowerSupply 252
#define CMD_RemoteRSSI 101
#define CMD_MeasurementCount 102
#define CMD_SendingCount 103
#define CMD_RecoveryCount 104

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

#if defined(BME280present)
#include <SoftwareI2C.h>
#include "MyAttinyBME280.h"
MyAttinyBME280 bme280(BME280_I2Caddress, BME280_SDA, BME280_SCL);
extern SoftwareI2C TinyWireM;
#endif

int printChar(char var, FILE *stream) {
    if (var == '\n') Serial.print('\r');
    Serial.print(var);
    return 0;
}

FILE out = { 0 };

struct DataPacket
{
    byte powerCmd;
    uint16_t power;		// mV * 10
    byte senderRssiCmd;
    byte senderRssi;	// dB * -1
    byte measurementCountCmd;
    uint32_t measurementCount;
    byte sendingCountCmd;
    uint32_t sendingCount;
    byte recoveryCountCmd;
    byte recoveryCount;
#if defined(DHT22present)
    byte tempCmd;
    int16_t temp;		// °C * 10
    byte humidityCmd;
    int16_t humidity;   // % * 10
#endif
#if defined(BMP180present)
    byte temp2Cmd;
    int16_t temp2;		// °C * 10
    byte pressureCmd;
    int16_t pressure;	// hPa * 10
#endif
#if defined(BME280present)
    byte tempCmd;
    int16_t temp;		// °C * 10
    byte humidityCmd;
    int16_t humidity;	// % * 10
    byte pressureCmd;
    int16_t pressure;	// hPa * 10
#endif

    DataPacket()
    {
        powerCmd = CMD_PowerSupply;
        senderRssiCmd = CMD_RemoteRSSI;
        measurementCountCmd = CMD_MeasurementCount;
        sendingCountCmd = CMD_SendingCount;
        recoveryCountCmd = CMD_RecoveryCount;
#if defined(DHT22present)
        tempCmd = CMD_temperature;
        humidityCmd = CMD_humidity;
#endif
#if defined(BMP180present)
        temp2Cmd = CMD_temperature2;
        pressureCmd = CMD_pressure;
#endif
#if defined(BME280present)
        tempCmd = CMD_temperature;
        humidityCmd = CMD_humidity;
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

uint8_t writeReg(uint8_t addr, uint8_t val) { return RF69::control(addr | 0x80, val); }
uint8_t readReg(uint8_t addr) { return RF69::control(addr, 0); }

void PrintRFReg(uint16_t addr)
{
    printf_P(PSTR("%02X = %02X\n"), addr, readReg(addr));
}

void DumpRFRegs()
{
    printf_P(PSTR("RF Registers:\n"));
    printf_P(PSTR("AA  0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n\n"));

    for (uint16_t line = 0; line < 5; line++)
    {
        printf_P(PSTR("%02X "), line * 16);

        for (uint16_t r = 0; r < 16; r++)
        {
            uint16_t addr = line * 16 + r;
            if (addr == 0)
            {
                printf_P(PSTR("XX "));
                continue;
            }

            uint8_t val = readReg(addr);

            printf_P(PSTR("%02X "), val);
        }
        printf_P(PSTR("\n"));

    }
    printf_P(PSTR("\n"));

    PrintRFReg(0x58);
    PrintRFReg(0x5a);
    PrintRFReg(0x5c);
    PrintRFReg(0x5f);
    PrintRFReg(0x6f);
    PrintRFReg(0x71);
    printf_P(PSTR("\n"));
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

void deepSleep(word ms)
{
#if defined(USESERIAL) || defined(USESERIAL2) || defined(USESERIAL3)
    Serial.flush();
#endif
    Sleepy::loseSomeTime(ms);
}

#if defined(BME280present)

void PrintBMEReg(const char* name, byte addr)
{
    printf_P(PSTR("%s (%02X) = %02X\n"), name, addr, bme280.read8(addr));
}

void DumpBMERegs()
{
    PrintBMEReg("id       ", 0xD0);
    PrintBMEReg("reset    ", 0xE0);
    PrintBMEReg("ctrl hum ", 0xF2);
    PrintBMEReg("status   ", 0xF3);
    PrintBMEReg("ctrl meas", 0xF4);
    PrintBMEReg("config   ", 0xF5);

    PrintBMEReg("press1   ", 0xF7);
    PrintBMEReg("press2   ", 0xF8);
    PrintBMEReg("press3   ", 0xF9);

    PrintBMEReg("temp1    ", 0xFA);
    PrintBMEReg("temp2    ", 0xFB);
    PrintBMEReg("temp3    ", 0xFC);

    PrintBMEReg("hum1     ", 0xFD);
    PrintBMEReg("hum2     ", 0xFE);
}

#if defined(BME280_5V)
void turnOnBME280()
{
    // POWER ON
    digitalWrite(BME280_POWER, HIGH);
    delay(2);

    TinyWireM.begin(BME280_SDA, BME280_SCL);
}

void turnOffBME280()
{
    // POWER OFF
    digitalWrite(BME280_POWER, LOW);
    pinMode(BME280_SDA, INPUT);
    pinMode(BME280_SCL, INPUT);
    pinMode(BME280_SDA, INPUT);
}

void readBME280(int16_t& t, int16_t& p, int16_t& h)
{
    turnOnBME280();

    if (bme280.isReady())
    {
#if defined(USESERIAL)
        Serial.println("BME280 OK");
#endif

        //Serial.println("before init"); DumpBMERegs(&bme280);
        bme280.setWeatherMonitoring();
        bme280.init();
        //Serial.println("after init"); DumpBMERegs(&bme280);
        bme280.startSingleMeas();
        bme280.readAll(t, p, h);

#if defined(BME280raw)
        Serial.println();
        Serial.print("adc_T = "); Serial.println(bme280.adc_T);
        Serial.print("adc_H = "); Serial.println(bme280.adc_H);
        Serial.print("adc_P = "); Serial.println(bme280.adc_P);
        Serial.println();
        Serial.print("T = "); Serial.println(t);
        Serial.print("H = "); Serial.println(h);
        Serial.print("P = "); Serial.println(p);
        Serial.println();
        Serial.print("dig_T1 = "); Serial.println(bme280.dig_T1);
        Serial.print("dig_T2 = "); Serial.println(bme280.dig_T2);
        Serial.print("dig_T3 = "); Serial.println(bme280.dig_T3);
        Serial.println();
        Serial.print("dig_H1 = "); Serial.println(bme280.dig_H1);
        Serial.print("dig_H2 = "); Serial.println(bme280.dig_H2);
        Serial.print("dig_H3 = "); Serial.println(bme280.dig_H3);
        Serial.print("dig_H4 = "); Serial.println(bme280.dig_H4);
        Serial.print("dig_H5 = "); Serial.println(bme280.dig_H5);
        Serial.print("dig_H6 = "); Serial.println(bme280.dig_H6);
        Serial.println();
        Serial.print("dig_P1 = "); Serial.println(bme280.dig_P1);
        Serial.print("dig_P2 = "); Serial.println(bme280.dig_P2);
        Serial.print("dig_P3 = "); Serial.println(bme280.dig_P3);
        Serial.print("dig_P4 = "); Serial.println(bme280.dig_P4);
        Serial.print("dig_P5 = "); Serial.println(bme280.dig_P5);
        Serial.print("dig_P6 = "); Serial.println(bme280.dig_P6);
        Serial.print("dig_P7 = "); Serial.println(bme280.dig_P7);
        Serial.print("dig_P8 = "); Serial.println(bme280.dig_P8);
        Serial.print("dig_P9 = "); Serial.println(bme280.dig_P9);
        Serial.println();
#endif
    }
    else
    {
#if defined(USESERIAL)
        Serial.println("BME280 ERR");
#endif
        t = -1000;
        p = -1000;
        h = -1000;
    }

    turnOffBME280();
}
#endif

#endif

void InitRF()
{
    rf12_initialize(myNodeID, freq, network);
#if defined(isHCW)
    RF69::control(0x11 | 0x80, 0b01011111); // PA1 on
    RF69::control(0x13 | 0x80, 0b00011010); // OCP on 95mA
#endif
    rf12_sleep(RF12_SLEEP);
    //DumpRFRegs();
}

void reset()
{
    recoveryCount++;

#if defined(USESERIAL)
    Serial.print("\nRESET (");
    Serial.print(recoveryCount);
    Serial.println(") ...");
    Serial.flush();
#endif

    //asm("call 0");
    InitRF();
}

void setup()
{
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);

#if defined(DHT22present)
    pinMode(DHT22_POWER, OUTPUT);
    digitalWrite(DHT22_POWER, LOW);
#endif

#if defined(USESERIAL) || defined(USESERIAL2) || defined(USESERIAL3)
    Serial.begin(38400);
    Serial.println("setup");
    fdev_setup_stream(&out, printChar, NULL, _FDEV_SETUP_WRITE);
    stdout = &out;
#endif

#if defined(BMP180present)
    deepSleep(50);
    bmp180.getCalibData();
#endif

#if defined(BME280present)
#if defined(BME280_5V)
    pinMode(BME280_POWER, OUTPUT);
    digitalWrite(BME280_POWER, LOW);
#else // 3V3
    if (bme280.isReady())
    {
#if defined(USESERIAL)
        Serial.println("BME280 OK");
#endif
        //Serial.println("before init"); DumpBMERegs(&bme280);
        bme280.setWeatherMonitoring();
        bme280.init();
        //Serial.println("after init"); DumpBMERegs(&bme280);
    }
    else
    {
#if defined(USESERIAL)
        Serial.println("BME280 ERR");
#endif
    }

#endif
#endif

    InitRF();

    deepSleep(1000);
    digitalWrite(LED, LOW);
}

void loop()
{
    // MEASURE VCC
    data.power = readVcc() * 10;

    if (data.power < MINVCC)
    {
#if defined(USESERIAL)
        Serial.print("Vcc too low (Vcc=");
        Serial.print(data.power);
        Serial.print(" minVcc=");
        Serial.print(MINVCC);
        Serial.print(") POWER DOWN!\n");
        Serial.flush();
#endif
        Sleepy::powerDown();
        return;
    }

    // MEASURE
    measurementCount++;

#if defined(DHT22present)
#if defined(USESERIAL)
    Serial.println("DHT22");
#endif
    digitalWrite(DHT22_POWER, HIGH);
    deepSleep(2000); // DHT22 requires minimum 2s warm-up after power-on.
    DHT22_ERROR_t errorCode = dht22.readData();
    digitalWrite(DHT22_POWER, LOW);
#endif

#if defined(BMP180present)
#if defined(USESERIAL)
    Serial.println("BMP180");
#endif
    deepSleep(bmp180.startMeas(BMP085::TEMP));
    bmp180.getResult(BMP085::TEMP);

    deepSleep(bmp180.startMeas(BMP085::PRES));
    bmp180.getResult(BMP085::PRES);
#endif

#if defined(BME280present)
#if defined(BME280_5V)
    readBME280(data.temp, data.pressure, data.humidity);
#else // 3V3
    if (bme280.isReady())
    {
#if defined(USESERIAL)
        Serial.println("BME280 OK");
#endif
        bme280.startSingleMeas();
        bme280.readAll(data.temp, data.pressure, data.humidity);

#if defined(BME280raw)
        Serial.println();
        Serial.print("adc_T = "); Serial.println(bme280.adc_T);
        Serial.print("adc_H = "); Serial.println(bme280.adc_H);
        Serial.print("adc_P = "); Serial.println(bme280.adc_P);
        Serial.println();
        Serial.print("T = "); Serial.println(t);
        Serial.print("H = "); Serial.println(h);
        Serial.print("P = "); Serial.println(p);
        Serial.println();
        Serial.print("dig_T1 = "); Serial.println(bme280.dig_T1);
        Serial.print("dig_T2 = "); Serial.println(bme280.dig_T2);
        Serial.print("dig_T3 = "); Serial.println(bme280.dig_T3);
        Serial.println();
        Serial.print("dig_H1 = "); Serial.println(bme280.dig_H1);
        Serial.print("dig_H2 = "); Serial.println(bme280.dig_H2);
        Serial.print("dig_H3 = "); Serial.println(bme280.dig_H3);
        Serial.print("dig_H4 = "); Serial.println(bme280.dig_H4);
        Serial.print("dig_H5 = "); Serial.println(bme280.dig_H5);
        Serial.print("dig_H6 = "); Serial.println(bme280.dig_H6);
        Serial.println();
        Serial.print("dig_P1 = "); Serial.println(bme280.dig_P1);
        Serial.print("dig_P2 = "); Serial.println(bme280.dig_P2);
        Serial.print("dig_P3 = "); Serial.println(bme280.dig_P3);
        Serial.print("dig_P4 = "); Serial.println(bme280.dig_P4);
        Serial.print("dig_P5 = "); Serial.println(bme280.dig_P5);
        Serial.print("dig_P6 = "); Serial.println(bme280.dig_P6);
        Serial.print("dig_P7 = "); Serial.println(bme280.dig_P7);
        Serial.print("dig_P8 = "); Serial.println(bme280.dig_P8);
        Serial.print("dig_P9 = "); Serial.println(bme280.dig_P9);
        Serial.println();
#endif
    }
    else
    {
#if defined(USESERIAL)
        Serial.println("BME280 ERR");
#endif
        data.temp = -1000;
        data.pressure = -1000;
        data.humidity = -1000;
    }
#endif
#endif

    // CALCULATE
#if defined(DHT22present)
    if (errorCode == DHT_ERROR_NONE)
    {
        data.temp = dht22.getTemperatureCInt();
        data.humidity = dht22.getHumidityInt();
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

    int16_t dT = data.temp - sentT;
    int16_t dH = data.humidity - sentH;
    int16_t dP = data.pressure - sentP;

    // TRACE
#if defined(USESERIAL)
#if defined(DHT22present)
    Serial.print(" T=");
    Serial.print(data.temp, DEC);
    Serial.print("(");
    Serial.print(dT, DEC);
    Serial.print(") H=");
    Serial.print(data.humidity, DEC);
    Serial.print("(");
    Serial.print(dH, DEC);
    Serial.print(")");
#endif
#if defined(BMP180present)
    Serial.print(" T2=");
    Serial.print(data.temp2, DEC);
    Serial.print(" P=");
    Serial.print(data.pressure, DEC);
#endif
#if defined(BME280present)
    Serial.print(" T=");
    Serial.print(data.temp, DEC);
    Serial.print("(");
    Serial.print(dT, DEC);
    Serial.print(") H=");
    Serial.print(data.humidity, DEC);
    Serial.print("(");
    Serial.print(dH, DEC);
    Serial.print(") P=");
    Serial.print(data.pressure, DEC);
    Serial.print("(");
    Serial.print(dP, DEC);
    Serial.print(")");
#endif
    Serial.print(" V=");
    Serial.print(data.power, DEC);
    Serial.print(" RSSI(-");
    Serial.print(data.senderRssi, DEC);
    Serial.print(")");
    Serial.println();
#endif

    if ((abs(dT) < THRES_T) && (abs(dH) < THRES_H) && (abs(dP) < THRES_P) &&
        (skipCount < MAXSENDSKIPS))
    {
#if defined(USESERIAL)
        Serial.print("SKIP SEND. thresholds (T=");
        Serial.print(THRES_T);
        Serial.print(" H=");
        Serial.print(THRES_H);
        Serial.print(" P=");
        Serial.print(THRES_P);
        Serial.print(") skipCount=");
        Serial.print(skipCount);
        Serial.println();
#endif
        skipCount++;
    }
    else
    {
        skipCount = 0;

        // SEND
        sendingCount++;

        data.measurementCount = measurementCount;
        data.sendingCount = sendingCount;
        data.recoveryCount = recoveryCount;

#if defined(USESERIAL)
        Serial.print("SEND (");
        Serial.print(sendingCount);
        Serial.println(")");
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
            deepSleep(retryDelay);
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

            failureCount = 0;

            sentT = data.temp;
            sentH = data.humidity;
            sentP = data.pressure;
        }
        else
        {
            failureCount++;

            sentT = -1000;
            sentH = -1000;
            sentP = -1000;

#if defined(LEDFLASHS)
            flashLED(100, 3);
#endif

#if defined(USESERIAL)
            Serial.print(" SEND ERR AFTER ");
            Serial.print(RETRIES, DEC);
            Serial.print(" RETRIES. failureCount=");
            Serial.print(failureCount, DEC);
            Serial.println();
#endif

            if (failureCount >= MAXFAILURES)
                reset();
        }
    }

    // SLEEP
#if defined(USESERIAL)
    Serial.print("SLEEP ");
#endif
    for (int w = 0; w < WAITLOOPS; w++)
    {
#if defined(USESERIAL)
        Serial.print(".");
#endif
        deepSleep(WAITINTERVAL);
    }
#if defined(USESERIAL)
    Serial.println();
#endif
}
