#pragma once

#include <ArduinoJson.h>
#include "main.h"

class RadioApplication: public AsyncWebHandler {
public:
    RadioApplication();
    virtual ~RadioApplication();
    virtual void loop() = 0;
    virtual void onMqttMessage(String topic, String payload) {}
};

extern RadioApplication *radioapp;
