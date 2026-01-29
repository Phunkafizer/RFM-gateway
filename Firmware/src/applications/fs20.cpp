#include "fs20.h"

FS20::FS20(const JsonObject &conf):
        RcPulseTransceiver(868350000UL) {
    (void) conf;
    new FS20Codec;
}

FS20::~FS20() {
    RcCodec::freeCodecs();
}