#include "rc433.h"
#include "rccodecs.h"

Rc433Transceiver::Rc433Transceiver(const JsonObject &conf) {
    new ITTristate;
    new IT32;
    new PilotaCasa;
    new EV1527Codec;
    new Emylo;
}

Rc433Transceiver::~Rc433Transceiver() {
    RcCodec::freeCodecs();
}


