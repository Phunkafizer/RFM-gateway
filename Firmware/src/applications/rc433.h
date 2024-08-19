#pragma once

#include "rcpulse.h"

class Rc433Transceiver: public RcPulseTransceiver {
private:
public:
    Rc433Transceiver(const JsonObject &conf);
    ~Rc433Transceiver();
};
