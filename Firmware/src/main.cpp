#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <DNSServer.h>
#include "main.h"
#include "applications/868gw.h"
#include "applications/fs20.h"
#include "applications/rc433.h"
#include "html.h"

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

static const char FILE_RADIO[] PROGMEM = "radio.json";
static const char FILE_CONFIG[] PROGMEM = "config.json";
static const char APP_JSON[] PROGMEM = "application/json";
static const char HOSTNAME[] PROGMEM = "rfm-gateway";
static const char AP_NAME[] PROGMEM = "RFM-gateway";
static const char AP_PASS[] PROGMEM = "12345678";
static const IPAddress apAddress(4, 3, 2, 1);
static const IPAddress apSubnet(255, 255, 255, 0);
static const uint16_t WEBPORT = 80;
static const uint8_t DNS_PORT = 53;

AsyncWebServer websrv(WEBPORT);
AsyncWebSocket ws("/ws");
WiFiClient espClient;
WiFiClientSecure espSecClient;
PubSubClient mqtt;
bool rebootFlag = false;
DNSServer dnsServer;

String mqttHost;
String mqttUser;
String mqttPass;
String baseTopic;

Rfm69 *rfm69 = nullptr;

void loadRadioSetup() {
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
                break;

            case RFM_TYPE_RFM69Hxx:
                rfm69 = new Rfm69;
                rfm69->begin(16, true);
                break;

            default:
                break;
            }

            if (rfm69 != nullptr)
                rfm69->setFCorr(cfg[F("fCorr")]);
        }
        f.close();
    }
}

void setConfig(const JsonObject &obj) {
    if (obj.containsKey(F("mqtt"))) {
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
        if (jMqtt.containsKey(F("basetopic")))
            baseTopic = jMqtt[F("basetopic")].as<String>();
        if (baseTopic.isEmpty())
            baseTopic = F("home/rfm-gateway");

        mqtt.setServer(mqttHost.c_str(), jMqtt[F("port")] | 1883);
        mqtt.setBufferSize(1024);
    }

    if (obj.containsKey(F("application"))) { 
        if (radioapp != nullptr) {
            delete radioapp;
            radioapp = nullptr;
        }

        switch (obj[F("application")].as<int>()) {
        case 0:
            if (rfm69 != nullptr)
                radioapp = new Rc433Transceiver(obj[F("appSettings")].as<JsonObject>());
            break;
        case 1:
            if (rfm69 != nullptr)
                radioapp = new Gw868(obj[F("appSettings")].as<JsonObject>());
            break;
        case 2:
            if (rfm69 != nullptr)
                radioapp = new FS20(obj[F("appSettings")].as<JsonObject>());
            break;

        default:
            break;
        }
    }

    if (rfm69 != nullptr) {
        int8_t pwr = obj[F("txPwr")] | 13;
        rfm69->setTxPower(pwr);
    }
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
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

    if (event == WIFI_EVENT_STAMODE_GOT_IP) {
        //WiFi.setHostname(FPSTR(HOSTNAME)->c_str());
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
        radioapp->onMqttMessage(sTop, sPayload);
        ws.textAll("Rec. MQTT ~/" + sTop + ": " + sPayload);
    }
}

void setup() {
    SPI.begin();
    Serial.begin(76800);
    WiFi.begin();
    MDNS.begin(FPSTR(HOSTNAME));
    LittleFS.begin();

    loadRadioSetup();
    
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

    websrv.on("/scan", HTTP_GET, [](AsyncWebServerRequest *request) {
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

    websrv.on("/setwifi", HTTP_POST, [] (AsyncWebServerRequest *request) {
        if (request->hasArg(F("ssid")) && request->hasArg(F("pass"))) {
            request->send(200);

            String ssid = request->arg(F("ssid"));
            String pass = request->arg(F("pass"));
            Serial.print(ssid);
            Serial.println(pass);

            WiFi.disconnect();
            WiFi.persistent(true);
            WiFi.begin(ssid, pass);
            WiFi.setAutoConnect(false); // connect on power on
            WiFi.setAutoReconnect(true);
        }
        else
            request->send(400); // bad request
    });

    websrv.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
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

        AsyncResponseStream *response = request->beginResponseStream(FPSTR(APP_JSON));
        serializeJson(doc, *response);
        request->send(response); });

    websrv.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
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

    websrv.on("/config", HTTP_POST, 
            [] (AsyncWebServerRequest *request) {}, 
            [] (AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data, size_t len, bool final) {}, 
            [] (AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        static String confBuf;
        if (!index)
            confBuf.clear();

        confBuf.concat((const char*) data, len);
        if (confBuf.length() != total)
            return;

        JsonDocument doc;
        if (deserializeJson(doc, confBuf) == DeserializationError::Ok) {
            if (doc.containsKey(F("radio"))) {
                File f = LittleFS.open(FPSTR(FILE_RADIO), "w");
                serializeJson(doc[F("radio")], f);
                f.close();
                loadRadioSetup();
            }

            if (doc.containsKey(F("config"))) {
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

    websrv.on("/txtest", HTTP_POST, [](AsyncWebServerRequest *request) {}, [](AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data, size_t len, bool final) {}, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
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

    websrv.on("/update", HTTP_POST, 
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

    websrv.onNotFound([](AsyncWebServerRequest *request) {
        request->redirect(F("/"));
    });

    File f = LittleFS.open(FPSTR(FILE_CONFIG), "r");
    if (f) {
        JsonDocument cfg;
        if (deserializeJson(cfg, f) == DeserializationError::Ok)
            setConfig(cfg.as<JsonObject>());
        f.close();
    }

    mqtt.setCallback(mqttCallback);
}


void loop() {
    static bool btnState = true;
    static bool apMode = false;

    #ifdef DEBUG
        apMode = true;
    #endif
    if (btnState) {
        if (analogRead(A0) > 512)
            btnState = false;
        else
            if (millis() > 2000) {
                //start access point
                btnState = false;
                apMode = true;
            }
    }

    if (apMode) {
        WiFi.persistent(false);
        WiFi.softAPConfig(apAddress, apAddress, apSubnet);
        WiFi.softAP(FPSTR(AP_NAME), FPSTR(AP_PASS));
        WiFi.mode(WIFI_AP_STA);
        dnsServer.start(DNS_PORT, "*", apAddress);
        dnsServer.processNextRequest();
    }

    mqtt.loop();

    if (WiFi.localIP().isSet()) {
        if (!mqttHost.isEmpty() && !mqtt.connected()) {
            String id = WiFi.macAddress();
            id.remove(0, 9);
            int idx;
            while ( (idx = id.indexOf(':')) >= 0)
                id.remove(idx, 1);
            id = FPSTR(HOSTNAME) + id;

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
                mqtt.publish(statusTopic.c_str(), "online", true);
                String subtopic = baseTopic + "/#";
                mqtt.subscribe(subtopic.c_str());
                ws.textAll(F("MQTT connected"));
            }
            else
                ws.textAll(F("MQTT failure") + String(mqtt.state()));
        }
    }

    if (rfm69 != nullptr) {
        rfm69->loop();
        if (radioapp != nullptr)
            radioapp->loop();
    }

    if (rebootFlag) {
        delay(500);
        ESP.restart();
    }
}