#include <DoubleResetDetector.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266FtpServer.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <ArduinoJson.hpp>
#include <ArduinoJson.h>
#include <ezTime.h>
#include <fs.h>

#define USESERIAL
#define USELEDFLASHS

#define VERSION "V0.9"
#define VERSIONSTRING VERSION " (ESPFlowSensor " __DATE__ " " __TIME__ ")"

#define LED 2
#define LEDON 0
#define LEDOFF 1
#define FLASHINTERVAL 20

#define SENSOR 13
#define SENSOR_ECHO 12

#define RTCMEM_DRD 0
#define RTCMEM_CTX 16

#define MAGIC 0x11223344
#define MAXNAMESIZE 32
#define MAXTOPICSIZE 128
#define MAXPAYLOADSIZE 128
#define WIFICONNECTTIMEOUT 10000
#define WIFIMANAGERTIMEOUT 300
#define NTPSYNCINTERVAL 6*3600

#define TOPIC_CFG "CFG"
#define TOPIC_SETTINGS "SETTINGS"
#define TOPIC_STATES "STATES"
#define TOPIC_LWT "LWT"
#define LWTMESSAGE "{ \"online\": 0 }"

#define OTA_PATH "/firmware"
#define OTA_USER "admin"
#define OTA_PWD "didhvn#1"

#define WEBCFG_INDEX_PAGE "/index.html"

ADC_MODE(ADC_TOUT);

DoubleResetDetector drd(5, RTCMEM_DRD);
WiFiClient wifiClient;
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;
FtpServer ftpSrv;
PubSubClient mqttClient("0.0.0.0", 1883, 0, wifiClient);
Timezone tz;

struct _configuration
{
    uint32_t    magic = MAGIC;

    char        ssid[MAXNAMESIZE] = "";
    char        password[MAXNAMESIZE] = "";
    uint32_t    ipLocal = 0;
    uint32_t    subnet = 0;
    uint32_t    gateway = 0;
    uint32_t    dnsServer = 0;
    uint32_t    mqttServer = 0;

    char        name[MAXNAMESIZE] = "ESP-00-00-00-00-00-00";
    char        mqttTopicBase[MAXTOPICSIZE] = "tho/nodeESP";

    uint32_t    measureInterval = 333;
    uint32_t    minPublishInterval = 900000;
    uint32_t    resetFlowInterval = 60000;
    uint32_t    countsPerLiter = 5880;  // F (in Hz) = 98 * Q (in l/min) => 5880 impulses per litre

    char        timezone[MAXNAMESIZE] = "CET";
};
_configuration configuration;
bool sendSettings = true;
bool timeInitialized = false;
bool softReset = false;

struct _context
{
    uint32_t    magic = MAGIC;

    uint32_t    publishCount = 0;

    uint32_t    rawCounter = 0;
    float       totalVolumeL = 0;
    float       deltaVolumeL = 0;
    float       dt = 0;
    float       flowLPM = 0;

    float       todayVolumeL = 0;
    float       yesterdayVolumeL = 0;
};
_context context;
bool updateContextFromMQTT = false;
bool contextChanged = false;

void flashLED()
{
#if defined(USELEDFLASHS)
    digitalWrite(LED, LEDON);
    delay(FLASHINTERVAL);
    digitalWrite(LED, LEDOFF);
#endif
}

bool LoadConfiguration()
{
    _configuration cfg;

    EEPROM.begin(512);
    EEPROM.get(0, cfg);
    EEPROM.end();

    if (cfg.magic != MAGIC)
    {
#if defined(USESERIAL)
        Serial.printf("CFG: wrong magic %X\r\n", cfg.magic);
#endif
        snprintf(configuration.name, MAXNAMESIZE, "ESP-%s", WiFi.macAddress().c_str());
        return false;
    }

    configuration = cfg;
#if defined(USESERIAL)
    Serial.printf("CFG: configuration loaded\r\n");
#endif

    return true;
}

void SaveConfiguration()
{
    EEPROM.begin(512);
    EEPROM.put(0, configuration);
    EEPROM.end();

#if defined(USESERIAL)
    Serial.printf("CFG: configuration saved\r\n");
#endif

    sendSettings = true;
}

void PrintConfiguration(struct _configuration& cfg)
{
#if defined(USESERIAL)
    Serial.printf("ssid               : %s\r\n", cfg.ssid);
    Serial.printf("password           : %s\r\n", cfg.password);
    Serial.printf("ipLocal            : %s\r\n", IPAddress(cfg.ipLocal).toString().c_str());
    Serial.printf("subnet             : %s\r\n", IPAddress(cfg.subnet).toString().c_str());
    Serial.printf("gateway            : %s\r\n", IPAddress(cfg.gateway).toString().c_str());
    Serial.printf("dnsServer          : %s\r\n", IPAddress(cfg.dnsServer).toString().c_str());
    Serial.printf("mqttServer         : %s\r\n", IPAddress(cfg.mqttServer).toString().c_str());
    Serial.printf("name               : %s\r\n", cfg.name);
    Serial.printf("mqttTopicBase      : %s\r\n", cfg.mqttTopicBase);
    Serial.printf("measureInterval    : %lu\r\n", cfg.measureInterval);
    Serial.printf("minPublishInterval : %lu\r\n", cfg.minPublishInterval);
    Serial.printf("resetFlowInterval  : %lu\r\n", cfg.resetFlowInterval);
    Serial.printf("countsPerLiter     : %lu\r\n", cfg.countsPerLiter);
    Serial.printf("timezone           : %s\r\n", cfg.timezone);
#endif
}

void PrintConfiguration() { PrintConfiguration(configuration); }

bool UpdateConfiguration(String message)
{
#if defined(USESERIAL)
    Serial.printf("UPDCFG: message[%s]\r\n", message.c_str());
#endif

    bool updated = false;
    StaticJsonBuffer<JSON_OBJECT_SIZE(8) + 40> jsonBuffer;

    JsonObject& root = jsonBuffer.parse(message);

    if (root.containsKey("magic"))
    {
        configuration.magic = root["magic"];
        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: magic[%X]\r\n", configuration.magic);
#endif
    }

    if (root.containsKey("ssid"))
    {
        strncpy(configuration.ssid, root["ssid"], MAXNAMESIZE);
        updated = true;
        softReset = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: ssid[%s]\r\n", configuration.ssid);
#endif
    }

    if (root.containsKey("password"))
    {
        strncpy(configuration.password, root["password"], MAXNAMESIZE);
        updated = true;
        softReset = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: password[%s]\r\n", configuration.password);
#endif
    }

    if (root.containsKey("ipLocal"))
    {
        IPAddress ip;
        ip.fromString(root["ipLocal"].as<char*>());
        configuration.ipLocal = ip;
        updated = true;
        softReset = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: ipLocal[%s]\r\n", IPAddress(configuration.ipLocal).toString().c_str());
#endif
    }

    if (root.containsKey("subnet"))
    {
        IPAddress ip;
        ip.fromString(root["subnet"].as<char*>());
        configuration.subnet = ip;
        updated = true;
        softReset = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: subnet[%s]\r\n", IPAddress(configuration.subnet).toString().c_str());
#endif
    }

    if (root.containsKey("gateway"))
    {
        IPAddress ip;
        ip.fromString(root["gateway"].as<char*>());
        configuration.gateway = ip;
        updated = true;
        softReset = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: gateway[%s]\r\n", IPAddress(configuration.gateway).toString().c_str());
#endif
    }

    if (root.containsKey("dnsServer"))
    {
        IPAddress ip;
        ip.fromString(root["dnsServer"].as<char*>());
        configuration.dnsServer = ip;
        updated = true;
        softReset = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: dnsServer[%s]\r\n", IPAddress(configuration.dnsServer).toString().c_str());
#endif
    }

    if (root.containsKey("mqttServer"))
    {
        IPAddress ip;
        ip.fromString(root["mqttServer"].as<char*>());
        configuration.mqttServer = ip;
        updated = true;
        softReset = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: mqttServer[%s]\r\n", IPAddress(configuration.mqttServer).toString().c_str());
#endif
    }

    if (root.containsKey("name"))
    {
        strncpy(configuration.name, root["name"], MAXNAMESIZE);
        updated = true;
        softReset = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: name[%s]\r\n", configuration.name);
#endif
    }

    if (root.containsKey("measureInterval"))
    {
        configuration.measureInterval = root["measureInterval"];
        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: measureInterval[%lu]\r\n", configuration.measureInterval);
#endif
    }

    if (root.containsKey("minPublishInterval"))
    {
        configuration.minPublishInterval = root["minPublishInterval"];
        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: minPublishInterval[%lu]\r\n", configuration.minPublishInterval);
#endif
    }

    if (root.containsKey("resetFlowInterval"))
    {
        configuration.resetFlowInterval = root["resetFlowInterval"];
        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: resetFlowInterval[%lu]\r\n", configuration.resetFlowInterval);
#endif
    }

    if (root.containsKey("countsPerLiter"))
    {
        configuration.countsPerLiter = root["countsPerLiter"];
        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: countsPerLiter[%lu]\r\n", configuration.countsPerLiter);
#endif
    }

    if (root.containsKey("timezone"))
    {
        strncpy(configuration.timezone, root["timezone"], MAXNAMESIZE);
        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: timezone[%s]\r\n", configuration.timezone);
#endif
        ConfigureTime();
    }

    if (root.containsKey("totalVolumeL"))
    {
        float totalVolumeL = root["totalVolumeL"];
        context.rawCounter = VolumeLToCount(totalVolumeL);
        context.totalVolumeL = CountToVolumeL(context.rawCounter);
        contextChanged = true;
        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: totalVolumeL[%s]\r\n", String(context.totalVolumeL, 3).c_str());
#endif
    }

    if (root.containsKey("todayVolumeL"))
    {
        context.todayVolumeL = root["todayVolumeL"];
        contextChanged = true;
        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: todayVolumeL[%s]\r\n", String(context.todayVolumeL, 3).c_str());
#endif
    }

    if (root.containsKey("yesterdayVolumeL"))
    {
        context.yesterdayVolumeL = root["yesterdayVolumeL"];
        contextChanged = true;
        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCFG: yesterdayVolumeL[%s]\r\n", String(context.yesterdayVolumeL, 3).c_str());
#endif
    }

    return updated;
}

bool LoadContext()
{
    _context ctx;
    ESP.rtcUserMemoryRead(RTCMEM_CTX, (uint32_t*)&ctx, sizeof(ctx));

    if (ctx.magic != MAGIC)
    {
#if defined(USESERIAL)
        Serial.printf("CTX: wrong magic %X\r\n", ctx.magic);
#endif
        return false;
    }

    context = ctx;

#if defined(USESERIAL)
    Serial.printf("CTX: context loaded\r\n");
#endif
    return true;
}

void SaveContext()
{
    ESP.rtcUserMemoryWrite(RTCMEM_CTX, (uint32_t*)&context, sizeof(context));

#if defined(USESERIAL)
    Serial.printf("CTX: context saved\r\n");
#endif
}

void PrintContext(struct _context& ctx)
{
#if defined(USESERIAL)
    Serial.printf("publishCount     : %lu\r\n", context.publishCount);
    Serial.printf("rawCounter       : %lu\r\n", context.rawCounter);
    Serial.printf("totalVolumeL     : %s\r\n", String(context.totalVolumeL, 3).c_str());
    Serial.printf("deltaVolumeL     : %s\r\n", String(context.deltaVolumeL, 3).c_str());
    Serial.printf("todayVolumeL     : %s\r\n", String(context.todayVolumeL, 3).c_str());
    Serial.printf("yesterdayVolumeL : %s\r\n", String(context.yesterdayVolumeL, 3).c_str());
    Serial.printf("dt               : %s\r\n", String(context.dt, 3).c_str());
    Serial.printf("flowLPM          : %s\r\n", String(context.flowLPM, 3).c_str());
#endif
}

void PrintContext() { PrintContext(context); }

bool UpdateContext(String message)
{
#if defined(USESERIAL)
    Serial.printf("UPDCTX: message[%s]\r\n", message.c_str());
#endif

    bool updated = false;
    StaticJsonBuffer<JSON_OBJECT_SIZE(32) + 40> jsonBuffer;

    JsonObject& root = jsonBuffer.parse(message);

    if (!root.containsKey("STATES"))
    {
#if defined(USESERIAL)
        Serial.printf("UPDCTX: missing object \"STATES\"\r\n");
#endif
        return false;
    }

    JsonObject& states = root["STATES"];

    if (states.containsKey("rawCounter"))
    {
        context.rawCounter = states["rawCounter"];
        context.totalVolumeL = CountToVolumeL(context.rawCounter);
        context.deltaVolumeL = 0;
        context.flowLPM = 0;
        context.dt = 0;

        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCTX: rawCounter[%lu]\r\n", context.rawCounter);
#endif
    }

    if (states.containsKey("todayVolumeL"))
    {
        context.todayVolumeL = states["todayVolumeL"];

        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCTX: todayVolumeL[%s]\r\n", String(context.todayVolumeL, 3).c_str());
#endif
    }

    if (states.containsKey("yesterdayVolumeL"))
    {
        context.yesterdayVolumeL = states["yesterdayVolumeL"];

        updated = true;

#if defined(USESERIAL)
        Serial.printf("UPDCTX: yesterdayVolumeL[%s]\r\n", String(context.yesterdayVolumeL, 3).c_str());
#endif
    }

    return updated;
}

bool ConfigureTime()
{
    uint32_t t0 = millis();

    bool succeeded = tz.setLocation(configuration.timezone);
    tz.setDefault();

    uint32_t dt = millis() - t0;

#if defined(USESERIAL)
    Serial.printf("TIMECFG: location[%s] result[%d] dt[%d ms]\r\n", configuration.timezone, succeeded, dt);
#endif

    timeInitialized = false;

    return succeeded;
}

uint8_t dayCounterEventHandle = 0;
void SetDayCounterEvent()
{
    if (dayCounterEventHandle != 0)
    {
        deleteEvent(dayCounterEventHandle);
        dayCounterEventHandle = 0;
    }

    long interval = 86400;
    time_t _now = now();
    time_t _eventTime = _now - (_now % interval) + interval;
    dayCounterEventHandle = setEvent(HandleDayCounter, _eventTime);

#if defined(USESERIAL)
    Serial.printf("(%s): NEXT DAY COUNTER RESET: %s\r\n", dateTime(_now).c_str(), dateTime(_eventTime).c_str());
#endif
}

void HandleDayCounter()
{
    dayCounterEventHandle = 0;

    context.yesterdayVolumeL = context.todayVolumeL;
    context.todayVolumeL = 0;

    contextChanged = true;

    SetDayCounterEvent();
}

void HandleTime()
{
    events();

    if (!timeInitialized)
    {
        if (timeStatus() == timeSet)
        {
            timeInitialized = true;

#if defined(USESERIAL)
            Serial.printf("TIME INITIALIZED: %s\r\n", dateTime().c_str());
#endif
            SetDayCounterEvent();
        }
    }
}

void SetupHttpOTA()
{
    MDNS.begin(configuration.name);
    SPIFFS.begin();

    ftpSrv.begin(OTA_USER, OTA_PWD);

    httpUpdater.setup(&httpServer, OTA_PATH, OTA_USER, OTA_PWD);
    httpServer.begin();

    MDNS.addService("http", "tcp", 80);

#if defined(USESERIAL)
    Serial.printf("OTA: URL[http://%s.lan%s] user[%s] pwd[%s]\r\n", configuration.name, OTA_PATH, OTA_USER, OTA_PWD);
#endif
}

void SetupArduinoOTA()
{
#if defined(USESERIAL)
    ArduinoOTA.onStart([]() {
        Serial.println("ArduinoOTA: Start");
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("ArduinoOTA: ArduinoOTA: End");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("ArduinoOTA: Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("ArduinoOTA: Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
#endif

    ArduinoOTA.begin();

#if defined(USESERIAL)
    Serial.println("ArduinoOTA: Initialized!");
#endif
}

void SetupHttpConfigPage()
{
    httpServer.on("/", HttpHandleRoot);
}

void HttpReturnFail(String msg)
{
    httpServer.sendHeader("Connection", "close");
    httpServer.sendHeader("Access-Control-Allow-Origin", "*");
    httpServer.send(500, "text/plain", msg + "\r\n");
}

bool HttpSendIndex()
{
    if (!SPIFFS.exists(WEBCFG_INDEX_PAGE))
    {
#if defined(USESERIAL)
        Serial.printf("WEBCFG: missing index page [%s]\r\n", WEBCFG_INDEX_PAGE);
#endif
        HttpReturnFail("Index page not found.");
        return false;
    }

    File f = SPIFFS.open(WEBCFG_INDEX_PAGE, "r");
    String index = f.readString();
    f.close();

    index.replace("%countsPerLiter%", String(configuration.countsPerLiter));
    index.replace("%totalVolumeL%", String(context.totalVolumeL, 3));
    index.replace("%todayVolumeL%", String(context.todayVolumeL, 3));
    index.replace("%yesterdayVolumeL%", String(context.yesterdayVolumeL, 3));

    httpServer.send(200, "text/html", index);

#if defined(USESERIAL)
    Serial.printf("WEBCFG: index page [%s] sent.\r\n", WEBCFG_INDEX_PAGE);
#endif
    return true;
}

void HttpHandleRoot()
{
#if defined(USESERIAL)
    Serial.printf("WEBCFG: on/root args[%d]\r\n", httpServer.args());
#endif

    for (int i = 0; i < httpServer.args(); i++)
    {
        if (httpServer.argName(i).equals("countsPerLiter"))
        {
            configuration.countsPerLiter = httpServer.arg(i).toInt();
#if defined(USESERIAL)
            Serial.printf("WEBCFG: countsPerLiter[%lu]\r\n", configuration.countsPerLiter);
#endif
            SaveConfiguration();
        }
        else if (httpServer.argName(i).equals("totalVolumeL"))
        {
            context.rawCounter = VolumeLToCount(httpServer.arg(i).toFloat());
            context.totalVolumeL = CountToVolumeL(context.rawCounter);

#if defined(USESERIAL)
            Serial.printf("WEBCFG: totalVolumeL[%s]\r\n", String(context.totalVolumeL, 3).c_str());
#endif
            contextChanged = true;
        }
        else if (httpServer.argName(i).equals("todayVolumeL"))
        {
            context.todayVolumeL = httpServer.arg(i).toFloat();
#if defined(USESERIAL)
            Serial.printf("WEBCFG: todayVolumeL[%s]\r\n", String(context.todayVolumeL, 3).c_str());
#endif
            contextChanged = true;
        }
        else if (httpServer.argName(i).equals("yesterdayVolumeL"))
        {
            context.yesterdayVolumeL = httpServer.arg(i).toFloat();
#if defined(USESERIAL)
            Serial.printf("WEBCFG: yesterdayVolumeL[%s]\r\n", String(context.yesterdayVolumeL, 3).c_str());
#endif
            contextChanged = true;
        }
    }

    HttpSendIndex();
}

bool InitialWiFiConfiguration()
{
    digitalWrite(LED, LEDON);

#if defined(USESERIAL)
    Serial.printf("WIFIMAN: Starting WifiManager for %d seconds ...\r\n", WIFIMANAGERTIMEOUT);
#endif

    WiFiManager wifiManager;
    WiFiManagerParameter paramName("Name", "node name", configuration.name, MAXNAMESIZE);
    wifiManager.addParameter(&paramName);
    WiFiManagerParameter paramMQTT("MQTTIP", "MQTT server IP", (configuration.mqttServer != 0) ? IPAddress(configuration.mqttServer).toString().c_str() : "192.168.20.64", 16);
    wifiManager.addParameter(&paramMQTT);
    WiFiManagerParameter paramloopInterval("minPublishInterval", "publish interval (ms)", String(configuration.minPublishInterval).c_str(), 10);
    wifiManager.addParameter(&paramloopInterval);

    wifiManager.setConfigPortalTimeout(WIFIMANAGERTIMEOUT);

    bool succeeded;
    if (succeeded = wifiManager.startConfigPortal())
    {
#if defined(USESERIAL)
        Serial.printf("WIFIMAN: OK. SSID[%s] PWD[%s]\r\n", WiFi.SSID().c_str(), WiFi.psk().c_str());
#endif

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

        configuration.minPublishInterval = atoi(paramloopInterval.getValue());

        PrintConfiguration();
        SaveConfiguration();
    }
    else
    {
#if defined(USESERIAL)
        Serial.printf("WIFIMAN: FAILED. SSID[%s] PWD[%s]\r\n", WiFi.SSID().c_str(), WiFi.psk().c_str());
#endif
    }

    digitalWrite(LED, LEDOFF);
    return succeeded;
}

bool InitWiFi()
{
    uint32_t t0 = millis();

    if (configuration.ipLocal != 0)
    {
        WiFi.config(configuration.ipLocal, configuration.gateway, configuration.subnet, configuration.dnsServer);
    }

    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.forceSleepWake();
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
        Serial.printf("WiFi connected. localIP[%s] dt[%d ms]\r\n", WiFi.localIP().toString().c_str(), dt);
#endif
        WiFi.setAutoConnect(true);
        WiFi.setAutoReconnect(true);
        WiFi.setSleepMode(WIFI_MODEM_SLEEP);

        ConfigureTime();

        SetupHttpConfigPage();
        SetupHttpOTA();
        SetupArduinoOTA();
    }
    else
    {
#if defined(USESERIAL)
        Serial.printf("WiFi connection FAILED. status[%d] dt[%d ms]\r\n", wifiStatus, dt);
#endif
    }

    return (wifiStatus == WL_CONNECTED);
}

bool InitMQTT()
{
    uint32_t t0 = millis();

    if (mqttClient.connected())
    {
#if defined(USESERIAL)
        Serial.printf("MQTT already connected.\r\n");
#endif
        return true;
    }

    mqttClient.setCallback(mqttCallback);
    mqttClient.setServer(configuration.mqttServer, 1883);
    bool connected = mqttClient.connect(WiFi.localIP().toString().c_str(), GetTopic(TOPIC_LWT).c_str(), 1, true, LWTMESSAGE);

    uint32_t dt = millis() - t0;

    if (connected)
    {
#if defined(USESERIAL)
        Serial.printf("MQTT connected. dt[%d ms]\r\n", dt);
#endif
        publishOnline();

        mqttSubscribe(TOPIC_CFG);

        if (updateContextFromMQTT)
            mqttSubscribe(TOPIC_STATES);
    }
    else
    {
#if defined(USESERIAL)
        Serial.printf("MQTT connection FAILED. dt[%d ms]\r\n", dt);
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
    Serial.printf("MQTT: publishing topic[%s] msg[%s] r[%d] ... ", topic.c_str(), message, retained);
#endif

    if (!mqttClient.connected())
    {
#if defined(USESERIAL)
        Serial.print("FAILED. NOT CONNECTED.\r\n");
#endif
        return false;
    }

    bool succeeded = mqttClient.publish(topic.c_str(), message, retained);

    uint32_t dt = millis() - t0;

#if defined(USESERIAL)
    Serial.printf("%s. dt[%d ms]\r\n", succeeded ? "OK" : "FAILED", dt);
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
        Serial.print("FAILED. NOT CONNECTED.\r\n");
#endif
        return false;
    }

    bool succeeded = mqttClient.subscribe(topic.c_str());

    uint32_t dt = millis() - t0;

#if defined(USESERIAL)
    Serial.printf("%s. dt[%d ms]\r\n", succeeded ? "OK" : "FAILED", dt);
#endif

    return succeeded;
}

bool mqttUnsubscribe(const char* subTopic)
{
    uint32_t t0 = millis();

    String topic = GetTopic(subTopic);

#if defined(USESERIAL)
    Serial.printf("MQTT: unsubscribing topic[%s] ... ", topic.c_str());
#endif

    if (!mqttClient.connected())
    {
#if defined(USESERIAL)
        Serial.print("FAILED. NOT CONNECTED.\r\n");
#endif
        return false;
    }

    bool succeeded = mqttClient.unsubscribe(topic.c_str());

    uint32_t dt = millis() - t0;

#if defined(USESERIAL)
    Serial.printf("%s. dt[%d ms]\r\n", succeeded ? "OK" : "FAILED", dt);
#endif

    return succeeded;
}

void mqttCallback(char* topic, uint8_t* payload, uint length)
{
    String message;
    for (uint i = 0; i < length; i++)
        message += (char)payload[i];

#if defined(USESERIAL)
    Serial.printf("topic[%s] payload(%d)[%s]\r\n", topic, length, message.c_str());
#endif

    String strTopic(topic);

    if (strTopic.endsWith("/" TOPIC_CFG))
    {
        if (message.length() > 0)
        {
            if (UpdateConfiguration(message))
            {
                PrintConfiguration();
                SaveConfiguration();

                mqttPublish(TOPIC_CFG, "", true);
            }
        }
    }
    else if (strTopic.endsWith("/" TOPIC_STATES))
    {
        if (message.length() > 0)
        {
            if (updateContextFromMQTT)
            {
                if (UpdateContext(message))
                {
                    PrintContext();
                    SaveContext();

                    updateContextFromMQTT = false;
                    contextChanged = true;

                    mqttUnsubscribe(TOPIC_STATES);
                }
            }
        }
    }
}

void publishOnline()
{
    mqttPublish(TOPIC_LWT, "{ \"online\": 1 }", true);
}

void publishStates()
{
    char states[1024];
    snprintf(states, 1023, "{ \"STATES\": { \"rawCounter\": %d, \"totalVolumeL\": %s, \"deltaVolumeL\": %s, \"flowLPM\": %s, \"todayVolumeL\": %s, \"yesterdayVolumeL\": %s, \"publishCount\": %d, \"RSSI\": %d } }",
        context.rawCounter,
        String(context.totalVolumeL, 3).c_str(),
        String(context.deltaVolumeL, 3).c_str(),
        String(context.flowLPM, 3).c_str(),
        String(context.todayVolumeL, 3).c_str(),
        String(context.yesterdayVolumeL, 3).c_str(),
        context.publishCount,
        WiFi.RSSI()
    );

    if (mqttPublish(TOPIC_STATES, states, true))
        flashLED();
}

void publishSettings()
{
    char settings[1024];
    snprintf(settings, 1023, "{ \"SETTINGS\": { \"version\": \"%s\", \"ssid\": \"%s\", \"ipLocal\": \"%s\", \"mqttServer\": \"%s\", \"minPublishInterval\": %lu, \"resetFlowInterval\": %lu, \"measureInterval\": %lu, \"countsPerLiter\": %lu, \"timezone\": \"%s\" } }",
        VERSIONSTRING,
        configuration.ssid, IPAddress(configuration.ipLocal).toString().c_str(), IPAddress(configuration.mqttServer).toString().c_str(),
        configuration.minPublishInterval, configuration.resetFlowInterval,
        configuration.measureInterval, configuration.countsPerLiter,
        configuration.timezone
    );

    bool succeeded = mqttPublish(TOPIC_SETTINGS, settings, true);
}

volatile int pulseCount = 0;
volatile int echo = 0;
void sensorCallback()
{
    pulseCount++;
    echo = (echo == 0) ? 1 : 0;

    digitalWrite(SENSOR_ECHO, echo);
}

float CountToVolumeL(uint32_t count)
{
    if (configuration.countsPerLiter == 0)
        return 0;

    return (float)count / (float)configuration.countsPerLiter;
}

uint32_t VolumeLToCount(float volumeL)
{
    return (uint32_t)(configuration.countsPerLiter * volumeL);
}

uint32_t prevPulseCountTimestamp = 0;
uint32_t prevPulseCount = 0;
void UpdateCounters()
{
    uint32_t currentPulseCountTimestamp = millis();
    uint32_t currentPulseCount = pulseCount;

    uint32_t dt = currentPulseCountTimestamp - prevPulseCountTimestamp;
    uint32_t dp = currentPulseCount - prevPulseCount;

    if (dt > 0)
    {
        prevPulseCount = currentPulseCount;
        prevPulseCountTimestamp = currentPulseCountTimestamp;

        float prevVolumeL = context.totalVolumeL;
        context.rawCounter += dp;
        context.totalVolumeL = CountToVolumeL(context.rawCounter);
        context.deltaVolumeL = context.totalVolumeL - prevVolumeL;
        context.todayVolumeL += context.deltaVolumeL;

        context.dt = (float)dt / 1000.0;
        context.flowLPM = context.deltaVolumeL * 60.0 / context.dt;

        SaveContext();
    }
}

uint32_t prevPulseCount2Timestamp = 0;
uint32_t prevPulseCount2 = 0;
#define stateOFF 0
#define stateON 1
int state = stateOFF;
bool UpdateState()
{
    uint32_t timestamp = millis();
    if ((timestamp - prevPulseCount2Timestamp) < configuration.measureInterval)
        return false;

    uint32_t currentPulseCount = pulseCount;
    uint32_t dp = currentPulseCount - prevPulseCount2;
    bool counting = (dp > 0);

    prevPulseCount2 = currentPulseCount;
    prevPulseCount2Timestamp = timestamp;

    if ((state == stateOFF) && counting)
    {
#if defined(USESERIAL)
        Serial.printf("ON (%lu)\r\n", dp);
#endif
        state = stateON;
        UpdateCounters();
    }
    else if ((state == stateON) && !counting)
    {
#if defined(USESERIAL)
        Serial.println("OFF");
#endif
        state = stateOFF;
        UpdateCounters();
        return true;
    }

    return false;
}

uint32_t waitStartTime;
uint32_t waitInterval;
void setup()
{
    bool doubleResetDetected = drd.detectDoubleReset();

#if defined(USESERIAL)
    Serial.begin(115200);
    Serial.printf("\r\nSETUP doubleResetDetected[%d]\r\n", doubleResetDetected);
#endif

    pinMode(LED, OUTPUT);
    digitalWrite(LED, LEDON);

    pinMode(SENSOR, INPUT);
    pinMode(SENSOR_ECHO, OUTPUT);

    if (!LoadConfiguration())
        SaveConfiguration();
    PrintConfiguration();

    if (!LoadContext())
        updateContextFromMQTT = true;
    PrintContext();

#if defined(USESERIAL)
    Serial.println();
#endif
    if ((configuration.ssid[0] == 0) || doubleResetDetected)
    {
        drd.stop();
        InitialWiFiConfiguration();

        delay(2000);
        ESP.reset();
    }

    setInterval(NTPSYNCINTERVAL);

    delay(1000);
    digitalWrite(LED, LEDOFF);
    drd.stop();

    attachInterrupt(SENSOR, sensorCallback, FALLING);

    waitStartTime = millis();
    waitInterval = configuration.resetFlowInterval;
}

void loop()
{
    // WiFi
    if (WiFi.status() != WL_CONNECTED)
    {
        if (!InitWiFi())
        {
            delay(1000);
            return;
        }
    }

    // MQTT
    if (!mqttClient.connected())
    {
        if (!InitMQTT())
        {
            delay(1000);
            return;
        }
    }

    while (true)
    {
        ArduinoOTA.handle();
        httpServer.handleClient();
        ftpSrv.handleFTP();

        mqttClient.loop();

        HandleTime();

        yield();

        uint32_t currentTime = millis();

        if (UpdateState())
        {
            waitStartTime = currentTime;
            waitInterval = configuration.resetFlowInterval;
            break;
        }

        if (state != stateOFF)
            continue;

        if ((currentTime - waitStartTime) >= waitInterval)
        {
            waitStartTime = currentTime;
            waitInterval = configuration.minPublishInterval;

            UpdateCounters();
            break;
        }

        if (contextChanged)
            break;

        if (softReset)
            break;

        if (sendSettings)
        {
            sendSettings = false;

            publishSettings();
        }
    }

    contextChanged = false;
    context.publishCount++;

#if defined(USESERIAL)
    Serial.printf("\r\nTime(status:%d) %s\r\n", timeStatus(), dateTime().c_str());
#endif
    PrintContext();

#if defined(USESERIAL)
    Serial.println();
#endif
    publishStates();

    if (softReset)
    {
#if defined(USESERIAL)
        Serial.printf("\r\nSOFT RESET REQUESTED.\r\n");
#endif
        SaveContext();

        delay(2000);
        ESP.reset();
    }
}
