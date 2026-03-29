#include "rc433.h"
#include "rccodecs.h"

Rc433Transceiver::Rc433Transceiver(const JsonObject &conf):
        RcPulseTransceiver(433920000UL) {

    
    if (!conf[F("codecs")].is<JsonArray>()) {
        new ITTristate;
        new IT32;
        new PilotaCasa;
        new Emylo;
        new EV1527Codec;
        return;
    }

    JsonArray codecs = conf[F("codecs")].as<JsonArray>();
    for (JsonVariant codec : codecs) {
        switch (codec.as<int>()) {
        case 0:
            new ITTristate;
            break;

        case 1:
            // brennenstuhl
            break;

        case 2:
            new IT32;
            break;

        case 3:
            new PilotaCasa;
            break;

        case 4:
            new Emylo;
            break;

        case 5:
            new EV1527Codec;
            break;
        }
    }
}

Rc433Transceiver::~Rc433Transceiver() {
    RcCodec::freeCodecs();
}


