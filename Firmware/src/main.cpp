#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <Ticker.h>
#include "main.h"
#include "applications/868gw.h"
#include "applications/fs20.h"
#include "applications/rc433.h"
#include "html.h"
#include "global.h"
#include "HADiscLocal.h"


enum RfmType : uint8_t {
    RFM_TYPE_RFM69xx = 0,
    RFM_TYPE_RFM69Hxx = 1,
    RFM_TYPE_RFM95 = 2,
    RFM_TYPE_RFM96 = 3,
    RFM_TYPE_RFM97 = 4,
    RFM_TYPE_RFM98 = 5
};

enum FreqBand : uint8_t {
    FREQ_BAND_315 = 0,
    FREQ_BAND_433 = 1,
    FREQ_BAND_868 = 2,
    FREQ_BAND_915 = 3
};

PGM_P APP_JSON PROGMEM = "application/json";

static const char FILE_RADIO[] PROGMEM = "radio.json";
static const char FILE_CONFIG[] PROGMEM = "config.json";
static const char HOSTNAME[] PROGMEM = "rfm-gateway";
static const char STR_AP_NAME[] PROGMEM = AP_NAME;
static const char STR_AP_PASS[] PROGMEM = AP_PASS;
static const IPAddress apAddress(4, 3, 2, 1);
static const IPAddress apSubnet(255, 255, 255, 0);
static const uint16_t WEBPORT = 80;
static const uint8_t DNS_PORT = 53;
static const int MQTT_RETRY_INTERVAL_MS = 5000;

AsyncWebServer websrv(WEBPORT);
AsyncWebSocket ws("/ws");
WiFiClient espClient;
WiFiClientSecure espSecClient;
PubSubClient mqtt;
bool rebootFlag = false;
DNSServer dnsServer;
String hostname;
String mqttHost;
String mqttUser;
String mqttPass;
String baseTopic;
uint32_t mqttNextReconnectAt = 0;

Rfm69 *rfm69 = nullptr;

Ticker ledblink;
uint16_t leddata = 0x8000;

JsonDocument discJson;
bool startMdnsFlag = false;
bool startMdnsApFlag = false;
bool mdnsStarted = false;

String getAvailabilityTopic() {
    return baseTopic + F("/status");
}

void ledTickcb() {
    static uint16_t mask = 0x8000;

    digitalWrite(LED_BUILTIN, (leddata & mask) == 0);
    mask >>= 1;
    if (!mask)
        mask = 0x8000;
}

bool startMdns() {
    if (mdnsStarted) {
        SDBGLN("mDNS already started");
        return true;
    }

    if (hostname.isEmpty())
        hostname = FPSTR(HOSTNAME);
    
    mdnsStarted = true;
    
    // Don't close if already running - just restart the service
    if (!MDNS.begin(hostname.c_str())) {
        MDNS.close();
        if (!MDNS.begin(hostname.c_str())) {
            mdnsStarted = false;
            return false;
        }
    }
    
    MDNSResponder::hMDNSService hService = MDNS.addService(hostname.c_str(), "http", "tcp", WEBPORT);
    if (hService) {
        MDNSResponder::hMDNSTxt hTxt = MDNS.addServiceTxt(hService, "model", "rfm-gateway");
        
        // Ensure announcement is sent
        MDNS.update();
        yield();
        MDNS.update();
    } else {
        mdnsStarted = false;
    }
    return true;
}

bool loadRadioSetup() {
    bool result = false;
    File f = LittleFS.open(FPSTR(FILE_RADIO), "r");
    if (f) {
        JsonDocument cfg;
        if (deserializeJson(cfg, f) == DeserializationError::Ok) {
            if (rfm69 != nullptr)
                delete rfm69;

            switch (cfg[F("rfmType")].as<int>()) {
            case RFM_TYPE_RFM69xx:
                rfm69 = new Rfm69;
                rfm69->begin(16, false);
                result = true;
                break;

            case RFM_TYPE_RFM69Hxx:
                rfm69 = new Rfm69;
                rfm69->begin(16, true);
                result = true;
                break;

            default:
                break;
            }

            if (rfm69 != nullptr)
                rfm69->setFCorr(cfg[F("fCorr")]);
        }
        f.close();
    }
    return result;
}

void setConfig(const JsonObject &obj) {
    if (!obj[F("hostname")].isNull()) {
        hostname = obj[F("hostname")].as<String>();
        WiFi.setHostname(hostname.c_str());

        if (WiFi.localIP().isSet())
            startMdns();
    }

    if (!obj[F("mqtt")].isNull()) {
        const JsonObject &jMqtt = obj[F("mqtt")];
        if (mqtt.connected())
            mqtt.disconnect();

        if (jMqtt[F("tls")].as<bool>()) {
            espSecClient.setInsecure();
            mqtt.setClient(espSecClient);
        }
        else
            mqtt.setClient(espClient);

        mqttHost = jMqtt[F("host")].as<String>();
        mqttUser = jMqtt[F("user")].as<String>();
        mqttPass = jMqtt[F("pass")].as<String>();
        if (!jMqtt[F("basetopic")].isNull())
            baseTopic = jMqtt[F("basetopic")].as<String>();
        if (baseTopic.isEmpty())
            baseTopic = F("home/rfm-gateway");

        mqtt.setServer(mqttHost.c_str(), jMqtt[F("port")] | 1883);
        mqtt.setBufferSize(1024);
        mqttNextReconnectAt = 0;
    }

    if (!obj[F("application")].isNull()) {
        if (radioapp != nullptr) {
            delete radioapp;
            radioapp = nullptr;
        }

        JsonObject appSettings = obj[F("appSettings")];

        switch (obj[F("application")].as<int>()) {
        case 0:
            if (rfm69 != nullptr)
                radioapp = new Rc433Transceiver(appSettings);
            break;
        case 1:
            if (rfm69 != nullptr)
                radioapp = new Gw868(appSettings);
            break;
        case 2:
            if (rfm69 != nullptr)
                radioapp = new FS20(appSettings);
            break;

        default:
            break;
        }
    }

    if (rfm69 != nullptr) {
        const int8_t pwr = obj[F("txPwr")] | 13;
        rfm69->setTxPower(pwr);
        const int8_t rxThresh = obj[F("rxThresh")] | -70;
        rfm69->setRxThresh(rxThresh);
        if (radioapp != nullptr)
            if (obj[F("rffreq")].is<double>()) {
                uint32_t freq = obj[F("rffreq")].as<float>() * 1e6;
                setFreq(freq);
            }
            radioapp->restartReceive();
    }
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    (void) server;
    (void) client;
    (void) arg;
    (void) data;
    (void) len;
    switch (type) {
        case WS_EVT_CONNECT:
            //Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
            break;
        case WS_EVT_DISCONNECT:
            //Serial.printf("WebSocket client #%u disconnected\n", client->id());
            break;
        case WS_EVT_DATA:
            //handleWebSocketMessage(arg, data, len);
            break;
        case WS_EVT_PONG:
        case WS_EVT_ERROR:
            break;
    }
}

void wiFiEvent(WiFiEvent_t event) {
    if (event == 7)
        return;

    SDBG("wiFiEvent ");
    SDBGLN((int) event);

    // Defer mDNS (re)start to loop context when STA gets an IP.
    if (event == WIFI_EVENT_STAMODE_GOT_IP) {
        if (hostname.isEmpty())
            hostname = FPSTR(HOSTNAME);
        WiFi.setHostname(hostname.c_str());
        startMdnsFlag = true;
    }

    if (event == WIFI_EVENT_STAMODE_DISCONNECTED) {
    }
}

void mqttCallback(const char topic[], byte* payload, unsigned int length) {
    if (radioapp != nullptr) {
        String sTop(topic);
        sTop = sTop.substring(baseTopic.length() + 1);
        String sPayload;
        sPayload.concat((const char*) payload, length);
        ws.textAll(F("<div class='rfframe'>"));
        ws.textAll(F("Rec. MQTT ~/") + sTop + ": " + sPayload);
        radioapp->onMqttMessage(sTop, sPayload);
        ws.textAll(F("</div>"));
    }
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT); // ESP12 LED, GPIO2 also used for 1wire
    digitalWrite(LED_BUILTIN, HIGH);
    
    ledblink.attach(0.1, ledTickcb);

    bool apMode = false;
    #ifdef DEBUG
        apMode = true;
    #endif
    LittleFS.begin();

    while (analogRead(A0) < 512) {
        yield();
        if (millis() > 2000) {
            apMode = true;
            leddata = 0xA000;
            if (millis() > 10000) { // factory reset
                ledblink.detach();
                LittleFS.remove(FPSTR(FILE_CONFIG));
                LittleFS.end();
                WiFi.disconnect(true);
                ESP.eraseConfig();
                digitalWrite(LED_BUILTIN, LOW);
                while (analogRead(A0) < 512)
                    yield();
                ESP.reset();
                while (true);
            }
        }
    }

    Serial.begin(76800);
    SPI.begin();

    hostname = FPSTR(HOSTNAME);
    
    if (!loadRadioSetup())
        apMode = true;

    if (apMode) {
        leddata = 0xA000;
        WiFi.persistent(false);
        WiFi.softAPConfig(apAddress, apAddress, apSubnet);
        WiFi.softAP(FPSTR(STR_AP_NAME), FPSTR(STR_AP_PASS));
        WiFi.mode(WIFI_AP_STA);
        dnsServer.start(DNS_PORT, "*", apAddress);
        dnsServer.processNextRequest();
        // In AP+STA mode, delay mDNS startup until STA has an IP.
        // Starting on AP first can prevent expected LAN discovery behavior.
        startMdnsApFlag = false;
    }

    File f = LittleFS.open(FPSTR(FILE_CONFIG), "r");
    if (f) {
        JsonDocument cfg;
        if (deserializeJson(cfg, f) == DeserializationError::Ok)
            setConfig(cfg.as<JsonObject>());
        f.close();
    }

    if (hostname.isEmpty())
        hostname = FPSTR(HOSTNAME);

    WiFi.setHostname(hostname.c_str());
    WiFi.onEvent(wiFiEvent);
    WiFi.begin();

    discJson.set(nullptr);
    
    websrv.begin();
    ws.onEvent(onWsEvent);
    websrv.addHandler(&ws);
    websrv.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        #ifdef DEBUG
        if (LittleFS.exists(F("/index.html"))) {
            request->send(LittleFS, F("/index.html"), F("text/html"));
            return;
        }
        #endif
        request->send_P(200, F("text/html"), html);
    });

    websrv.on(PSTR("/scan"), HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        JsonObject jobj = doc.to<JsonObject>();

        int n = WiFi.scanComplete();
        jobj[F("status")] = n;
        if (n == -2)
            WiFi.scanNetworks(true);
        else
            if (n >= 0) {
                JsonArray results = jobj[F("results")].to<JsonArray>();
                for (int i=0; i<n; i++) {
                    JsonObject result = results.add<JsonObject>();
                    result[F("ssid")] = WiFi.SSID(i);
                    result[F("rssi")] = WiFi.RSSI(i);
                    result[F("channel")] = WiFi.channel(i);
                }
                WiFi.scanDelete();
            }

        AsyncResponseStream *response = request->beginResponseStream(FPSTR(APP_JSON));
        serializeJson(doc, *response);
        request->send(response);
    });

    websrv.on(PSTR("/setwifi"), HTTP_POST, [] (AsyncWebServerRequest *request) {
        if (request->hasArg(F("ssid")) && request->hasArg(F("pass"))) {
            request->send(200);

            String ssid = request->arg(F("ssid"));
            String pass = request->arg(F("pass"));

            WiFi.disconnect();
            WiFi.persistent(true);
            WiFi.begin(ssid, pass);
            WiFi.setAutoConnect(false); // connect on power on
            WiFi.setAutoReconnect(true);
        }
        else
            request->send(400); // bad request
    });

    websrv.on(PSTR("/status"), HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;

        JsonObject jWifi = doc[F("WiFi")].to<JsonObject>();
        jWifi[F("status")] = WiFi.status();
        jWifi[F("ipsta")] = WiFi.localIP().toString();
        jWifi[F("mac")] = WiFi.macAddress();
        jWifi[F("hostname")] = WiFi.getHostname();
        jWifi[F("sta_ssid")] = WiFi.SSID();

        JsonObject jSystem = doc[F("system")].to<JsonObject>();
        jSystem[F("uptime")] = millis() / 1000;
        jSystem[F("freeHeap")] = ESP.getFreeHeap();
        jSystem[F("firmware")] = F(BUILD_VERSION);

        JsonObject jMqtt = doc[F("mqtt")].to<JsonObject>();
        jMqtt[F("state")] = mqtt.state();
        jMqtt[F("connected")] = mqtt.connected();

        AsyncResponseStream *response = request->beginResponseStream(FPSTR(APP_JSON));
        serializeJson(doc, *response);
        request->send(response); });

    websrv.on(PSTR("/config"), HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        doc.to<JsonObject>();

        File fRadio = LittleFS.open(FPSTR(FILE_RADIO), "r");
        if (fRadio) {
            JsonDocument radio;
            if (deserializeJson(radio, fRadio) == DeserializationError::Ok)
                doc[F("radio")] = radio;
            fRadio.close();
        }

        File fCfg = LittleFS.open(FPSTR(FILE_CONFIG), "r");
        if (fCfg) {
            JsonDocument cfg;
            if (deserializeJson(cfg, fCfg) == DeserializationError::Ok)
                doc["config"] = cfg;
            fCfg.close();
        }

        AsyncResponseStream *response = request->beginResponseStream(FPSTR(APP_JSON));
        serializeJson(doc, *response);
        request->send(response); 
    });

    websrv.on(PSTR("/config"), HTTP_POST, 
        [] (AsyncWebServerRequest *request) {(void) request;}, 
        [] (AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data, size_t len, bool final) {
            (void) request;
            (void) filename;
            (void) index;
            (void) data;
            (void) len;
            (void) final;
        }, 
        [] (AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        static String confBuf;
        if (!index)
            confBuf.clear();

        confBuf.concat((const char*) data, len);
        if (confBuf.length() != total)
            return;

        JsonDocument doc;
        if (deserializeJson(doc, confBuf) == DeserializationError::Ok) {
            if (!doc[F("radio")].isNull()) {
                File f = LittleFS.open(FPSTR(FILE_RADIO), "w");
                serializeJson(doc[F("radio")], f);
                f.close();
                loadRadioSetup();
            }

            if (!doc[F("config")].isNull()) {
                File f = LittleFS.open(FPSTR(FILE_CONFIG), "w");
                serializeJson(doc[F("config")], f);
                f.close();
                setConfig(doc[F("config")]);
            }
            request->send(200);
        }
        else {
            request->send(400);
        }
        confBuf.clear();
    });

    websrv.on(PSTR("/txtest"), HTTP_POST, [](AsyncWebServerRequest *request) {
            (void) request;
        }, 
        [](AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data, size_t len, bool final) {
            (void) request;
            (void) filename;
            (void) index;
            (void) data;
            (void) len;
            (void) final;
        }, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
            (void) index;
            (void) total;

        request->send(200);

        JsonDocument doc;
        deserializeJson(doc, (char*) data, len);

        if (rfm69 != nullptr) {
            delete rfm69;
            rfm69 = nullptr;
        }

        uint8_t rfmtype = doc[F("rfmType")].as<uint8_t>();
        switch (rfmtype) {
        case RFM_TYPE_RFM69xx:
            rfm69 = new Rfm69();
            rfm69->begin(16, false);
            break;
        case RFM_TYPE_RFM69Hxx:
            rfm69 = new Rfm69();
            rfm69->begin(16, true);
            break;
        default: 
            break;
        }

        if (rfm69 != nullptr) {
            rfm69->txTest(
                doc[F("freq")].as<uint32_t>(),
                doc[F("fCorr")].as<int16_t>(), 
                doc[F("pwr")].as<int8_t>(),
                doc[F("baud")].as<uint16_t>()
            );
        }
    });

    websrv.on(PSTR("/senddisc"), HTTP_POST, 
        [] (AsyncWebServerRequest *request) { // onRequest handler
            (void) request;
        },
        [](AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data, size_t len, bool final) {
            // onUpload handler
            (void) request;
            (void) filename;
            (void) index;
            (void) data;
            (void) len;
            (void) final;
        },
        [] (AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
             // onBody handler
            (void) request;
            (void) data;
            (void) len;

            if (!discJson.isNull()) {
                request->send(400);
                return;
            }

            static String buf;

            if (index == 0)
                buf.clear();

            buf.concat((const char*) data, len);
            if (buf.length() != total)
                return;

            if (deserializeJson(discJson, buf) == DeserializationError::Ok) {
                request->send(200);
            }
            else {
                request->send(400);
            }   
            buf.clear(); 
    });

    websrv.on(PSTR("/update"), HTTP_POST, 
        [] (AsyncWebServerRequest *request) { // onRequest handler
            int httpRes;

            rebootFlag = !Update.hasError();
            if (rebootFlag)
                httpRes = 200;
            else
            httpRes = 500;

            AsyncWebServerResponse *response = request->beginResponse(httpRes);
            response->addHeader(F("Connection"), F("close"));
            request->send(response);
        },
        [] (AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data, size_t len, bool final) { // onUpdate handler
            (void) request;
            (void) filename;
            if (!index) {
                Update.runAsync(true);
                Update.begin(request->contentLength(), U_FLASH);
            }
            Update.write(data, len);
            if (final) {
                Update.end(true);
            }
        }
    );

    websrv.on(PSTR("/reboot"), HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200);
        rebootFlag = true;
    });

    websrv.on(PSTR("/tabs"), HTTP_GET, [](AsyncWebServerRequest *request) {
        if (radioapp != nullptr && radioapp->html != nullptr)
            request->send_P(200, F("text/html"), radioapp->html);
        else
            request->send(404);
    });

    websrv.on(PSTR("/regdump"), HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!rfm69) {
            request->send(404);
            return;
        }

        String result;
        for (uint8_t reg= 0x00; reg <= 0x4F; reg++) {
            uint8_t val = rfm69->readReg(reg);
            result += String(reg, HEX) + ": " + String(val, HEX) + "<br>";
        }
        request->send(200, "text", result);
    });

    websrv.onNotFound([](AsyncWebServerRequest *request) {
        request->send(404);
    });

    mqtt.setCallback(mqttCallback);
    haDisc.begin();
}


void loop() {
    if (startMdnsFlag) {
        startMdnsFlag = false;

        // In AP+STA mode, mDNS may have been started on AP first.
        // Rebind by closing and restarting once STA has an IP.
        if (mdnsStarted) {
            MDNS.close();
            mdnsStarted = false;
        }

        if (startMdns()) {
            MDNS.notifyAPChange();
            MDNS.update();
        }
    }
    
    if (startMdnsApFlag) {
        startMdnsApFlag = false;
        startMdns();
    }

    MDNS.update();
    mqtt.loop();

    if (WiFi.localIP().isSet()) {
        const uint32_t now = millis();
        if (!mqttHost.isEmpty() && !mqtt.connected() && (mqttNextReconnectAt == 0 || (int32_t) (now - mqttNextReconnectAt) >= 0)) {
            String id = WiFi.macAddress();

            id.remove(0, 9);
            int idx;
            while ( (idx = id.indexOf(':')) >= 0)
                id.remove(idx, 1);

            id = String(FPSTR(HOSTNAME)) + id;

            String statusTopic = baseTopic + F("/status");
            bool con = mqtt.connect(
                id.c_str(), 
                mqttUser.c_str(), 
                mqttPass.c_str(),
                statusTopic.c_str(),
                1,
                true,
                "offline"
            );
            if (con) {
                mqttNextReconnectAt = 0;
                mqtt.publish(statusTopic.c_str(), "online", true);
                String subtopic = baseTopic + F("/#");
                mqtt.subscribe(subtopic.c_str());
                ws.textAll(F("MQTT connected"));
            }
            else {
                mqttNextReconnectAt = now + MQTT_RETRY_INTERVAL_MS;
                ws.textAll(F("MQTT failure") + String(mqtt.state()));
            }
        }
    }

    if (rfm69 != nullptr) {
        rfm69->loop();
        if (radioapp != nullptr)
            radioapp->loop();
    }

    if (!discJson.isNull()) {
        if ( (radioapp != nullptr) && mqtt.connected())
            radioapp->sendDiscovery(discJson);
        discJson.set(nullptr);
    }

    if (rebootFlag) {
        delay(500);
        ESP.restart();
    }
}