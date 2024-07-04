#pragma once

#include "../radioapplication.h"

class Gw868: public RadioApplication {
private:
    uint8_t currentRxMode;
    uint16_t rxModes; // bitmask
    uint8_t currentRxLen;
    unsigned long nextSwitch;
    uint32_t interval;
public:
    Gw868(const JsonObject &conf);
    void loop();
};