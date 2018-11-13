#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <ArduinoJson.hpp>
#include <ArduinoJson.h>

#include <Adafruit_BME280.h>

#define USESERIAL
#define USELEDFLASHS

#define LED 2
#define LEDON 0
#define LEDOFF 1
#define FLASHINTERVAL 1

#define RTCMEM_DRD 0
#define RTCMEM_CTX 16

#define MAGIC 0x12345678
#define MAGIC64 0x1234567890975310
#define MAXNAMESIZE 32
#define MAXTOPICSIZE 128
#define MAXPAYLOADSIZE 128
#define WIFICONNECTTIMEOUT 10000
#define WIFIMANAGERTIMEOUT 300
#define WAITCONFIGURATION 1000

#define SLEEPTIMEMIN 5e6
#define WAKEMODE_INIT -1
#define WAKEMODE_RFOFF 0
#define WAKEMODE_RFON 1

#define BME280_I2Caddress 0x76
#define BME280_SDA 4
#define BME280_SCL 5

ADC_MODE(ADC_TOUT);

uint32_t startTimestamp;
WiFiClient wifiClient;
PubSubClient mqttClient("0.0.0.0", 1883, 0, wifiClient);

struct _configuration
{
    uint32_t    magic = MAGIC;

    uint32_t    mqttCfgEnabled = 0;

    char        ssid[MAXNAMESIZE] = "";
    char        password[MAXNAMESIZE] = "";
    uint32_t    ipLocal = 0;
    uint32_t    subnet = 0;
    uint32_t    gateway = 0;
    uint32_t    dnsServer = 0;
    uint32_t    mqttServer = 0;

    char        name[MAXNAMESIZE] = "ESP-00-00-00-00-00-00";
    char        mqttTopicBase[MAXTOPICSIZE] = "tho/nodeESP";
    uint32_t    sleepTime = 300000000;
    uint32_t    maxIntervalTime = 3600000000;

    float       vccMultiplier = 11.0;
    float       vccThreshold = 0.19;
    float       temperatureThreshold = 0.19;
    float       humidityThreshold = 0.9;
    float       pressureThreshold = 0.9;
};
_configuration configuration;
bool configurationChanged = false;

struct _context
{
    uint64_t magic = MAGIC64;

    uint32_t wakeMode = WAKEMODE_INIT;
    uint32_t sleepCount = 0;
    uint32_t publishCount = 0;
    uint32_t intervalTime = 0;
    uint32_t resetCount = 0;
    uint32_t publishSettings = 1;

    float vcc0 = 0;
    float temperature0 = 0;
    float humidity0 = 0;
    float pressure0 = 0;
    float vcc = 0;
    float temperature = 0;
    float humidity = 0;
    float pressure = 0;
};
_context context;

bool LoadConfiguration()
{
#if defined(USESERIAL)
    Serial.println();
#endif
    _configuration cfg;

    EEPROM.begin(512);
    EEPROM.get(0, cfg);
    EEPROM.end();

    if (cfg.magic != MAGIC)
    {
#if defined(USESERIAL)
        Serial.printf("CFG: wrong magic %X\n", cfg.magic);
#endif
        snprintf(configuration.name, MAXNAMESIZE, "ESP-%s", WiFi.macAddress().c_str());
        return false;
    }

    configuration = cfg;
#if defined(USESERIAL)
    Serial.printf("CFG: configuration loaded\n");
#endif

    return true;
}

void SaveConfiguration()
{
    EEPROM.begin(512);
    EEPROM.put(0, configuration);
    EEPROM.end();

#if defined(USESERIAL)
    Serial.printf("\nCFG: configuration saved\n");
#endif

    context.publishSettings = true;
}

void PrintConfiguration(struct _configuration& cfg)
{
#if defined(USESERIAL)
    Serial.printf("mqttCfgEnabled  : %lu\n", cfg.mqttCfgEnabled);
    Serial.printf("ssid            : %s\n", cfg.ssid);
    Serial.printf("password        : %s\n", cfg.password);
    Serial.printf("ipLocal         : %s\n", IPAddress(cfg.ipLocal).toString().c_str());
    Serial.printf("subnet          : %s\n", IPAddress(cfg.subnet).toString().c_str());
    Serial.printf("gateway         : %s\n", IPAddress(cfg.gateway).toString().c_str());
    Serial.printf("dnsServer       : %s\n", IPAddress(cfg.dnsServer).toString().c_str());
    Serial.printf("mqttServer      : %s\n", IPAddress(cfg.mqttServer).toString().c_str());
    Serial.println();
    Serial.printf("name            : %s\n", cfg.name);
    Serial.printf("mqttTopicBase   : %s\n", cfg.mqttTopicBase);
    Serial.printf("sleepTime       : %lu\n", cfg.sleepTime);
    Serial.printf("maxIntervalTime : %lu\n", cfg.maxIntervalTime);
    Serial.println();
    Serial.printf("vccMultiplier        : %s\n", String(cfg.vccMultiplier, 3).c_str());
    Serial.printf("vccThreshold         : %s\n", String(cfg.vccThreshold, 2).c_str());
    Serial.printf("temperatureThreshold : %s\n", String(cfg.temperatureThreshold, 2).c_str());
    Serial.printf("humidityThreshold    : %s\n", String(cfg.humidityThreshold, 2).c_str());
    Serial.printf("pressureThreshold    : %s\n", String(cfg.pressureThreshold, 2).c_str());
#endif
}

void PrintConfiguration() { PrintConfiguration(configuration); }

void UpdateConfiguration(String message)
{
#if defined(USESERIAL)
    Serial.printf("\nUPDCFG: message[%s]\n", message.c_str());
#endif

    bool updated = false;
    StaticJsonBuffer<JSON_OBJECT_SIZE(32) + 40> jsonBuffer;

    JsonObject& root = jsonBuffer.parse(message);

    if (root.containsKey("magic"))
    {
        configuration.magic = root["magic"];
        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: magic[%X]\n", configuration.magic);
#endif
    }

    if (root.containsKey("mqttCfgEnabled"))
    {
        configuration.mqttCfgEnabled = root["mqttCfgEnabled"];
        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: mqttCfgEnabled[%lu]\n", configuration.mqttCfgEnabled);
#endif
    }

    if (root.containsKey("ssid"))
    {
        strncpy(configuration.ssid, root["ssid"], 16);
        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: ssid[%s]\n", configuration.ssid);
#endif
    }

    if (root.containsKey("password"))
    {
        strncpy(configuration.password, root["password"], MAXNAMESIZE);
        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: password[%s]\n", configuration.password);
#endif
    }

    if (root.containsKey("ipLocal"))
    {
        IPAddress ip;
        ip.fromString(root["ipLocal"].as<char*>());
        configuration.ipLocal = ip;
        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: ipLocal[%s]\n", IPAddress(configuration.ipLocal).toString().c_str());
#endif
    }

    if (root.containsKey("subnet"))
    {
        IPAddress ip;
        ip.fromString(root["subnet"].as<char*>());
        configuration.subnet = ip;
        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: subnet[%s]\n", IPAddress(configuration.subnet).toString().c_str());
#endif
    }

    if (root.containsKey("gateway"))
    {
        IPAddress ip;
        ip.fromString(root["gateway"].as<char*>());
        configuration.gateway = ip;
        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: gateway[%s]\n", IPAddress(configuration.gateway).toString().c_str());
#endif
    }

    if (root.containsKey("dnsServer"))
    {
        IPAddress ip;
        ip.fromString(root["dnsServer"].as<char*>());
        configuration.dnsServer = ip;
        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: dnsServer[%s]\n", IPAddress(configuration.dnsServer).toString().c_str());
#endif
    }

    if (root.containsKey("mqttServer"))
    {
        IPAddress ip;
        ip.fromString(root["mqttServer"].as<char*>());
        configuration.mqttServer = ip;
        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: mqttServer[%s]\n", IPAddress(configuration.mqttServer).toString().c_str());
#endif
    }

    if (root.containsKey("name"))
    {
        strncpy(configuration.name, root["name"], MAXNAMESIZE);
        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: name[%s]\n", configuration.name);
#endif
    }

    if (root.containsKey("sleepTime"))
    {
        configuration.sleepTime = root["sleepTime"];
        if (configuration.sleepTime < SLEEPTIMEMIN)
            configuration.sleepTime = SLEEPTIMEMIN;
        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: sleepTime[%lu]\n", configuration.sleepTime);
#endif
    }

    if (root.containsKey("maxIntervalTime"))
    {
        configuration.maxIntervalTime = root["maxIntervalTime"];
        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: maxIntervalTime[%lu]\n", configuration.maxIntervalTime);
#endif
    }

    if (root.containsKey("vccMultiplier"))
    {
        configuration.vccMultiplier = root["vccMultiplier"];
        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: vccMultiplier[%s]\n", String(configuration.vccMultiplier, 3).c_str());
#endif
    }

    if (root.containsKey("vccThreshold"))
    {
        configuration.vccThreshold = root["vccThreshold"];
        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: vccThreshold[%s]\n", String(configuration.vccThreshold, 2).c_str());
#endif
    }

    if (root.containsKey("temperatureThreshold"))
    {
        configuration.temperatureThreshold = root["temperatureThreshold"];
        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: temperatureThreshold[%s]\n", String(configuration.temperatureThreshold, 2).c_str());
#endif
    }

    if (root.containsKey("humidityThreshold"))
    {
        configuration.humidityThreshold = root["humidityThreshold"];
        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: humidityThreshold[%s]\n", String(configuration.humidityThreshold, 2).c_str());
#endif
    }

    if (root.containsKey("pressureThreshold"))
    {
        configuration.pressureThreshold = root["pressureThreshold"];
        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: pressureThreshold[%s]\n", String(configuration.pressureThreshold, 2).c_str());
#endif
    }

    if (updated)
        configurationChanged = true;
}

bool LoadContext()
{
#if defined(USESERIAL)
    Serial.println();
#endif
    _context ctx;
    ESP.rtcUserMemoryRead(RTCMEM_CTX, (uint32_t*)&ctx, sizeof(ctx));

    if (ctx.magic != MAGIC64)
    {
#if defined(USESERIAL)
        Serial.printf("CTX: wrong magic %X\n", ctx.magic);
#endif
        return false;
    }

    context = ctx;

#if defined(USESERIAL)
    Serial.printf("CTX: context loaded.\n");
#endif
    return true;
}

void SaveContext()
{
    ESP.rtcUserMemoryWrite(RTCMEM_CTX, (uint32_t*)&context, sizeof(context));

#if defined(USESERIAL)
    Serial.printf("\nCTX: context saved.\n");
#endif
}

void PrintContext(struct _context& ctx)
{
#if defined(USESERIAL)
    Serial.printf("publishSettings : %d\n", ctx.publishSettings);
    Serial.printf("wakeMode        : %s\n", wakeModeText(ctx.wakeMode).c_str());
    Serial.printf("sleepCount      : %lu\n", ctx.sleepCount);
    Serial.printf("publishCount    : %lu\n", ctx.publishCount);
    Serial.printf("intervalTime    : %lu\n", ctx.intervalTime);
    Serial.printf("resetCount      : %lu\n", ctx.resetCount);
    Serial.printf("vcc             : %s(%s) V\n", String(ctx.vcc, 3).c_str(), String(ctx.vcc - ctx.vcc0, 3).c_str());
    Serial.printf("temperature     : %s(%s) C\n", String(ctx.temperature, 1).c_str(), String(ctx.temperature - ctx.temperature0, 1).c_str());
    Serial.printf("humidity        : %s(%s) %%\n", String(ctx.humidity, 1).c_str(), String(ctx.humidity - ctx.humidity0, 1).c_str());
    Serial.printf("pressure        : %s(%s) hPa\n", String(ctx.pressure, 1).c_str(), String(ctx.pressure - ctx.pressure0, 1).c_str());
#endif
}

void PrintContext() { PrintContext(context); }

String wakeModeText(uint32_t wakeMode)
{
    switch (wakeMode)
    {
    case WAKEMODE_INIT:  return String("INIT");
    case WAKEMODE_RFOFF: return String("RFOFF");
    case WAKEMODE_RFON:  return String("RFON");
    default:             return String(wakeMode);
    }
}

void flashLED()
{
#if defined(USELEDFLASHS)
    digitalWrite(LED, LEDON);
    delay(FLASHINTERVAL);
    digitalWrite(LED, LEDOFF);
#endif
}

bool InitialWiFiConfiguration()
{
    digitalWrite(LED, LEDON);

#if defined(USESERIAL)
    Serial.printf("\nWIFIMAN: Starting WifiManager for %d seconds ...\n", WIFIMANAGERTIMEOUT);
#endif

    WiFiManager wifiManager;
    WiFiManagerParameter paramName("Name", "node name", configuration.name, MAXNAMESIZE);
    wifiManager.addParameter(&paramName);
    WiFiManagerParameter paramMQTT("MQTTIP", "MQTT server IP", (configuration.mqttServer != 0) ? IPAddress(configuration.mqttServer).toString().c_str() : "192.168.20.64", 16);
    wifiManager.addParameter(&paramMQTT);
    WiFiManagerParameter paramMQTTcfg("MQTTCFG", "config via MQTT enabled", String((configuration.mqttCfgEnabled != 0) ? 1 : 0).c_str(), 3);
    wifiManager.addParameter(&paramMQTTcfg);
    WiFiManagerParameter paramSleepTime("SleepTime", "sleeping time (ms)", String(configuration.sleepTime / 1000).c_str(), 10);
    wifiManager.addParameter(&paramSleepTime);
    WiFiManagerParameter paramMaxIntervalTime("maxIntervalTime", "max interval time (ms)", String(configuration.maxIntervalTime / 1000).c_str(), 10);
    wifiManager.addParameter(&paramMaxIntervalTime);
    WiFiManagerParameter paramVccMultiplier("VCCM", "Vcc multiplier", String(configuration.vccMultiplier, 3).c_str(), 10);
    wifiManager.addParameter(&paramVccMultiplier);

    wifiManager.setConfigPortalTimeout(WIFIMANAGERTIMEOUT);

    bool succeeded;
    if (succeeded = wifiManager.startConfigPortal())
    {
#if defined(USESERIAL)
        Serial.printf("WIFIMAN: OK. SSID[%s] PWD[%s]\n", WiFi.SSID().c_str(), WiFi.psk().c_str());
#endif
        configuration.mqttCfgEnabled = atoi(paramMQTTcfg.getValue());

        strncpy(configuration.ssid, WiFi.SSID().c_str(), MAXNAMESIZE);
        strncpy(configuration.password, WiFi.psk().c_str(), MAXNAMESIZE);
        configuration.ipLocal = WiFi.localIP();
        configuration.subnet = WiFi.subnetMask();
        configuration.gateway = WiFi.gatewayIP();
        configuration.dnsServer = WiFi.dnsIP();

        IPAddress mqttIP;
        mqttIP.fromString(paramMQTT.getValue());
        configuration.mqttServer = mqttIP;

        strncpy(configuration.name, paramName.getValue(), MAXNAMESIZE);

        configuration.sleepTime = atoi(paramSleepTime.getValue()) * 1000;
        configuration.maxIntervalTime = atoi(paramMaxIntervalTime.getValue()) * 1000;
        configuration.vccMultiplier = atof(paramVccMultiplier.getValue());

        PrintConfiguration();
        SaveConfiguration();
    }
    else
    {
#if defined(USESERIAL)
        Serial.printf("WIFIMAN: FAILED. SSID[%s] PWD[%s]\n", WiFi.SSID().c_str(), WiFi.psk().c_str());
#endif
    }

    digitalWrite(LED, LEDOFF);
    return succeeded;
}

bool InitWiFi()
{
    uint32_t t0 = millis();

    //WiFi.mode(WIFI_STA);
    //WiFi.setPhyMode(WIFI_PHY_MODE_11N);
    //WiFi.setSleepMode(WIFI_MODEM_SLEEP);
    WiFi.forceSleepWake();
    if (configuration.ipLocal != 0)
    {
        WiFi.config(configuration.ipLocal, configuration.gateway, configuration.subnet, configuration.dnsServer);
    }

    WiFi.begin(configuration.ssid, configuration.password);

    wl_status_t wifiStatus;
    while ((wifiStatus = WiFi.status()) != WL_CONNECTED)
    {
        delay(50);

        if (millis() - t0 > WIFICONNECTTIMEOUT)
            break;
    }

    uint32_t dt = millis() - t0;

    if (wifiStatus == WL_CONNECTED)
    {
#if defined(USESERIAL)
        Serial.printf("WiFi connected. localIP[%s] dt[%d ms]\n", WiFi.localIP().toString().c_str(), dt);
#endif
    }
    else
    {
#if defined(USESERIAL)
        Serial.printf("WiFi connection FAILED. status[%d] dt[%d ms]\n", wifiStatus, dt);
#endif
    }

    return (wifiStatus == WL_CONNECTED);
}

bool InitMQTT()
{
    uint32_t t0 = millis();

    mqttClient.setCallback(mqttCallback);
    mqttClient.setServer(configuration.mqttServer, 1883);
    bool connected = mqttClient.connect(WiFi.localIP().toString().c_str());

    uint32_t dt = millis() - t0;

    if (connected)
    {
#if defined(USESERIAL)
        Serial.printf("MQTT connected. dt[%d ms]\n", dt);
#endif
    }
    else
    {
#if defined(USESERIAL)
        Serial.printf("MQTT connection FAILED. dt[%d ms]\n", dt);
#endif
    }

    return connected;
}

String GetTopic(const char* subTopic)
{
    String topic(configuration.mqttTopicBase);

    topic += "/";
    topic += configuration.name;
    topic += "/";
    topic += subTopic;

    return topic;
}

bool mqttPublish(const char* subTopic, const char* message, bool retained)
{
    uint32_t t0 = millis();

    String topic = GetTopic(subTopic);

#if defined(USESERIAL)
    Serial.printf("MQTT: sending topic[%s] msg[%s] r[%d] ... ", topic.c_str(), message, retained);
#endif

    if (!mqttClient.connected())
    {
#if defined(USESERIAL)
        Serial.print("FAILED. NOT CONNECTED.\n");
#endif
        return false;
    }

    bool succeeded = mqttClient.publish(topic.c_str(), message, retained);

    uint32_t dt = millis() - t0;

#if defined(USESERIAL)
    Serial.printf("%s. dt[%d ms]\n", succeeded ? "OK" : "FAILED", dt);
#endif

    return succeeded;
}

bool mqttSubscribe(const char* subTopic)
{
    uint32_t t0 = millis();

    String topic = GetTopic(subTopic);

#if defined(USESERIAL)
    Serial.printf("MQTT: subscribing topic[%s] ... ", topic.c_str());
#endif

    if (!mqttClient.connected())
    {
#if defined(USESERIAL)
        Serial.print("FAILED. NOT CONNECTED.\n");
#endif
        return false;
    }

    bool succeeded = mqttClient.subscribe(topic.c_str());

    uint32_t dt = millis() - t0;

#if defined(USESERIAL)
    Serial.printf("%s. dt[%d ms]\n", succeeded ? "OK" : "FAILED", dt);
#endif

    return succeeded;
}

void mqttCallback(char* topic, uint8_t* payload, uint length)
{
    String message;
    for (uint i = 0; i < length; i++)
        message += (char)payload[i];

#if defined(USESERIAL)
    Serial.printf("topic[%s] payload(%d)[%s]\n", topic, length, message.c_str());
#endif

    String strTopic(topic);

    if (strTopic.endsWith("/CFG"))
    {
        UpdateConfiguration(message);
    }
}

void CheckForNewConfiguration()
{
    if (configuration.mqttCfgEnabled != 0)
    {
#if defined(USESERIAL)
        Serial.printf("CHECKING FOR NEW CONFIGURATION FOR [%d ms]\n", WAITCONFIGURATION);
#endif
    }
    else
    {
#if defined(USESERIAL)
        Serial.printf("MQTT CONFIGURATION DISABLED\n");
#endif
        return;
    }

    mqttSubscribe("CFG");

    uint32_t dt0 = millis();
    while (millis() - dt0 < WAITCONFIGURATION)
    {
        if (!mqttClient.loop())
        {
#if defined(USESERIAL)
            Serial.printf("LEAVING LOOP dt[%d ms]\n", millis() - dt0);
#endif
            break;
        }
        if (configurationChanged)
        {
#if defined(USESERIAL)
            Serial.printf("CFG CHANGED dt[%d ms]\n", millis() - dt0);
#endif
            break;
        }
    }

    mqttClient.unsubscribe("CFG");

    if (configurationChanged)
    {
        configurationChanged = false;
        mqttPublish("CFG", "", true);

        PrintConfiguration();
        SaveConfiguration();
    }
}

float readVcc() // A0 voltage divider: 1M - 100k => 11:1 (vccMultiplier)
{
    context.vcc = analogRead(A0) * configuration.vccMultiplier / 1024.0;
    float dVcc = context.vcc - context.vcc0;
    bool changed = (fabs(dVcc) >= configuration.vccThreshold);

#if defined(USESERIAL)
    Serial.printf("\nVcc=%s(%s) V changed[%d]\n", String(context.vcc, 3).c_str(), String(dVcc, 3).c_str(), changed);
#endif

    if (context.vcc < 2.9)
    {
#if defined(USESERIAL)
        Serial.printf("DEEP DISCHARGE PROTECTION. Sleeping forever.");
#endif
        ESP.deepSleep(0, RF_DISABLED);
    }

    return changed;
}

bool ReadBME280()
{
    bool changed = false;

    Adafruit_BME280 bme;

    Wire.begin(BME280_SDA, BME280_SCL);
    if (bme.begin(BME280_I2Caddress))
    {
#if defined(USESERIAL)
        Serial.printf("\nBME280 OK.\n");
#endif
        bme.takeForcedMeasurement();
        context.temperature = bme.readTemperature();
        context.humidity = bme.readHumidity();
        context.pressure = bme.readPressure() / 100.0;

        float dT = context.temperature - context.temperature0;
        float dH = context.humidity - context.humidity0;
        float dp = context.pressure - context.pressure0;

        changed = (
            (fabs(dT) >= configuration.temperatureThreshold) ||
            (fabs(dH) >= configuration.humidityThreshold) ||
            (fabs(dp) >= configuration.pressureThreshold));

#if defined(USESERIAL)
        Serial.printf("T=%s(%s) H=%s(%s) P=%s(%s) changed[%d]\n",
            String(context.temperature, 1).c_str(),
            String(dT, 2).c_str(),
            String(context.humidity, 1).c_str(),
            String(dH, 2).c_str(),
            String(context.pressure, 1).c_str(),
            String(dp, 2).c_str(),
            changed);
#endif
    }
    else
    {
#if defined(USESERIAL)
        Serial.printf("\nBME280 NOT FOUND.\n");
#endif
    }

    return changed;
}

void PublishState()
{
#if defined(USESERIAL)
    Serial.println();
#endif
    if (InitWiFi())
    {
        if (InitMQTT())
        {
#if defined(USESERIAL)
            Serial.println();
#endif
            CheckForNewConfiguration();

            const int bufferLen = 1024;
            char buffer[bufferLen];
            buffer[bufferLen - 1] = 0;

            if (context.publishSettings)
            {
#if defined(USESERIAL)
                Serial.println();
#endif
                snprintf(buffer, bufferLen - 1, "{ SETTINGS: { \"mqttCfgEnabled\": %d, \"ssid\": \"%s\", \"ipLocal\": \"%s\", \"mqttServer\": \"%s\", \"sleepTime\": %lu, \"maxIntervalTime\": %lu, \"vccMultiplier\": %s, \"vccThreshold\": %s, \"temperatureThreshold\": %s, \"humidityThreshold\": %s, \"pressureThreshold\": %s } }",
                    configuration.mqttCfgEnabled,
                    configuration.ssid, IPAddress(configuration.ipLocal).toString().c_str(), IPAddress(configuration.mqttServer).toString().c_str(),
                    configuration.sleepTime, configuration.maxIntervalTime,
                    String(configuration.vccMultiplier, 3).c_str(),
                    String(configuration.vccThreshold, 3).c_str(), String(configuration.temperatureThreshold, 2).c_str(), String(configuration.humidityThreshold, 2).c_str(), String(configuration.pressureThreshold, 2).c_str()
                );

                bool succeeded = mqttPublish("SETTINGS", buffer, true);

                if (succeeded)
                    context.publishSettings = false;
            }

#if defined(USESERIAL)
            Serial.println();
#endif
            snprintf(buffer, bufferLen - 1, "{ STATES: { \"sleepCount\": %lu, \"publishCount\": %lu, \"vcc\": %s, \"temperature\": %s, \"humidity\": %s, \"pressure\": %s, \"RSSI\": %ld } }",
                context.sleepCount,
                context.publishCount,
                String(context.vcc, 3).c_str(),
                String(context.temperature, 1).c_str(),
                String(context.humidity, 1).c_str(),
                String(context.pressure, 1).c_str(),
                WiFi.RSSI()
            );

            bool succeeded = mqttPublish("STATES", buffer, true);

            if (succeeded)
            {
                context.vcc0 = context.vcc;
                context.temperature0 = context.temperature;
                context.humidity0 = context.humidity;
                context.pressure0 = context.pressure;
            }

            mqttClient.disconnect();
        }
    }
}

void DeepSleep(uint32_t wakeMode, uint32_t sleepTime)
{
    context.wakeMode = wakeMode;
    context.sleepCount++;
    context.intervalTime += sleepTime;
    SaveContext();
    PrintContext();

    flashLED();

    uint32_t dt = millis() - startTimestamp;
#if defined(USESERIAL)
    Serial.printf("\ndt[%d ms] sleeping[%lu ms] wakeMode[%s] ...\n", dt, sleepTime / 1000, wakeModeText(wakeMode).c_str());
#endif

    ESP.deepSleep((sleepTime == 0) ? 1 : sleepTime, (wakeMode == WAKEMODE_RFON) ? RF_DEFAULT : RF_DISABLED);
}

void setup()
{
    startTimestamp = millis();

#if defined(USESERIAL)
    Serial.begin(115200);
    Serial.printf("\nSETUP\n");
#endif

    pinMode(LED, OUTPUT);
}

void loop()
{
    rst_info* ri = ESP.getResetInfoPtr();
    bool resetDetected = (ri->reason == REASON_EXT_SYS_RST);

#if defined(USESERIAL)
    Serial.printf("\nresetReason[%lu] resetDetected[%d]\n", ri->reason, resetDetected);
#endif

    if (!LoadContext())
        SaveContext();
    PrintContext();

    if (!LoadConfiguration())
        SaveConfiguration();
    PrintConfiguration();

    flashLED();

    if (resetDetected)
    {
        context.resetCount++;

#if defined(USESERIAL)
        Serial.printf("\nRESET DETECTED. resetCount[%lu]\n", context.resetCount);
#endif
        SaveContext();

        if ((context.resetCount > 1) && (context.wakeMode != WAKEMODE_RFON))
            DeepSleep(WAKEMODE_RFON, 1);

#if defined(USESERIAL)
        Serial.printf("WAIT FOR SECOND RESET ...");
#endif
        for (int i = 0; i < 10; i++)
        {
            digitalWrite(LED, LEDON);
            delay(250);
            digitalWrite(LED, LEDOFF);
            delay(250);
        }
#if defined(USESERIAL)
        Serial.printf("TIMEOUT. Resetting resetCount.\n");
#endif

        context.resetCount = 0;
        SaveContext();
    }

    if (context.wakeMode == WAKEMODE_INIT)
    {
#if defined(USESERIAL)
        Serial.printf("\n(WAKEMODE_INIT) INITIALIZATION: starting RF DISABLED ...");
#endif
        DeepSleep(WAKEMODE_RFOFF, 1);
    }

    if (context.wakeMode == WAKEMODE_RFOFF)
    {
#if defined(USESERIAL)
        Serial.printf("\n(WAKEMODE_RFOFF) measuring ...\n");
#endif
        bool changed = readVcc();

        changed |= ReadBME280();

        if (changed || (context.intervalTime >= configuration.maxIntervalTime))
        {
#if defined(USESERIAL)
            Serial.printf("\nSOMETHING CHANGED OR INTERVAL TIMEOUT: starting RF ENABLED ...\n");
#endif
            DeepSleep(WAKEMODE_RFON, 1);
        }

        context.resetCount = 0;
        DeepSleep(WAKEMODE_RFOFF, configuration.sleepTime);
    }

    if ((configuration.ssid[0] == 0) || (context.resetCount > 1))
    {
#if defined(USESERIAL)
        Serial.printf("\nMISSING WIFI CONFIG OR DOUBLE RESET: starting initial configuration ...\n");
#endif

        context.resetCount = 0;
        SaveContext();

        InitialWiFiConfiguration();

        DeepSleep(WAKEMODE_RFOFF, 1);
    }

#if defined(USESERIAL)
    Serial.printf("\n(WAKEMODE_RFON) sending ...\n");
#endif

    context.publishCount++;
    PublishState();

    context.intervalTime = 0;
    context.resetCount = 0;
    DeepSleep(WAKEMODE_RFOFF, configuration.sleepTime);
}
