#pragma once
#include "HADiscovery.h"

class RfmHADiscovery: public HADiscovery {
public:
    RfmHADiscovery();
    void begin();
    using HADiscovery::publish;
    bool publish(const bool avail = true);
};

extern RfmHADiscovery haDisc;