#pragma once

#include <ArduinoJson.h>
#include "main.h"

class RadioApplication: public AsyncWebHandler {
protected:
    uint32_t freq;
public:
    PGM_P html {nullptr};
    RadioApplication();
    virtual ~RadioApplication();
    virtual void loop() = 0;
    virtual bool onMqttMessage(String topic, String payload);
    virtual bool sendDiscovery(JsonDocument &doc) = 0;
    virtual void restartReceive() {}
    void setFreq(uint32_t freq);
    void publish(String topic, JsonDocument &doc);
    
};

extern RadioApplication *radioapp;
