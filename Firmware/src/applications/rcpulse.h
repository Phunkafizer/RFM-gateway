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
    void rotateBuf(uint8_t pos);
    bool canHandle(AsyncWebServerRequest *request __attribute__((unused)));
    void handleRequest(AsyncWebServerRequest *request __attribute__((unused)));
    void handleBody(AsyncWebServerRequest *request __attribute__((unused)), uint8_t *data __attribute__((unused)), size_t len __attribute__((unused)), size_t index __attribute__((unused)), size_t total __attribute__((unused)));
public:
    RcPulseTransceiver();
    void loop();
    void onMqttMessage(const String topic, const String payload);
    void sendPulseBuf(RcCodec &codec);
};