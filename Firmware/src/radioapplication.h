#pragma once

#include <ArduinoJson.h>
#include "main.h"

class RadioApplication: public AsyncWebHandler {
public:
    RadioApplication();
    virtual ~RadioApplication();
    virtual void loop() = 0;
    virtual bool onMqttMessage(String topic, String payload);
    virtual bool sendDiscovery(JsonDocument &doc) = 0;
    void publish(String topic, JsonDocument &doc);
};

extern RadioApplication *radioapp;
