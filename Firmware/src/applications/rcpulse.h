#pragma once
#include "../radioapplication.h"
#include "rccodecs.h"

class RcPulseTransceiver: public RadioApplication {
private:
    uint8_t pulseBuf[200];
    uint8_t bufPos;
    uint8_t bufLen;
    bool lastBit;
    uint16_t pulseLen;
    enum TxMode {
        TX_IDLE,
        TX_SYMBOLS,
        TX_FOOTER1,
        TX_FOOTER2
    } txMode;
    uint8_t txRepeats;
    uint16_t footer[2];
    struct TxQueItem {
        String path;
        String payload;
    };
    std::vector<TxQueItem> txQue;
    void rotateBuf(uint8_t pos);
    bool canHandle(AsyncWebServerRequest *request __attribute__((unused))) override;
    void handleRequest(AsyncWebServerRequest *request __attribute__((unused))) override;
    void handleBody(AsyncWebServerRequest *request __attribute__((unused)), uint8_t *data __attribute__((unused)), size_t len __attribute__((unused)), size_t index __attribute__((unused)), size_t total __attribute__((unused))) override;
public:
    RcPulseTransceiver(const uint32_t freq);
    void loop() override;
    void restartReceive() override;
    bool onMqttMessage(const String topic, const String payload) override;
    void sendPulseBuf(RcCodec &codec);
    bool sendDiscovery(JsonDocument &doc) override;
};