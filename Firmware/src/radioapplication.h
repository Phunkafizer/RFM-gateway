#pragma once

#include <ArduinoJson.h>
#include "main.h"

class RadioApplication {
public:
    virtual void loop() = 0;
    virtual ~RadioApplication() {}

};

extern RadioApplication *radioapp;
