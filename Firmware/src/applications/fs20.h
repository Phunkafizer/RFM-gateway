#pragma once

#include "rcpulse.h"

class FS20: public RcPulseTransceiver {
private:
public:
    FS20(const JsonObject &conf);
    ~FS20();
};