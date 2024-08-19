#include "rcpulse.h"

const uint8_t SEPERATION_LEN = 120;

RcPulseTransceiver::RcPulseTransceiver():
        bufPos(0),
        bufLen(0),
        lastBit(false),
        pulseLen(1),
        txMode(TX_IDLE) {
    //rfm69->setFreq(868350000UL);
    rfm69->setFreq(433920000UL);

    Rfm69::Rfm69Config cfg[] = {
        {Rfm69::RegRxBw, 2<<5 | Rfm69::RXBWASK_250KHZ},
        {Rfm69::RegSyncConfig, 1<<6}, 
        {Rfm69::RegRssiThresh, 180}, // /-0.5 dBm
        {Rfm69::RegDataModul, 1<<3}, // OOK
        {Rfm69::RegOokPeak, 1<<6 | 3<<0}, // peak threshold, increment every 8 chips
        {Rfm69::RegPreambleMsb, 0},
        {Rfm69::RegPreambleLsb, 0},
        {Rfm69::RegPacketConfig1, 0x00}, // fixed or unlimited length, no whitening, no crc
    };

    rfm69->writeConfig(cfg, sizeof(cfg) / sizeof(cfg[0]));
    rfm69->setBitrate(BITRATE);
    rfm69->setTxPower(13);

    rfm69->startReceive(0);
}

void RcPulseTransceiver::loop() {
    if (txMode > TX_IDLE) {
        while (rfm69->getFifoLevel() < Rfm69::FIFO_FULL) {
            uint8_t txbyte = 0x00;
            uint8_t bitmask = 0x80;

            while (bitmask) {
                if (lastBit)
                    txbyte |= bitmask;
                bitmask >>= 1;

                pulseLen--;
                if (pulseLen == 0) {
                    switch (txMode) {
                    case TX_SYMBOLS:
                        if (bufPos == bufLen) {
                            pulseLen = 5;
                            txMode = TX_FOOTER1;
                            break;
                        }
                        else
                            pulseLen = pulseBuf[bufPos++];
                        break;

                    case TX_FOOTER1:
                        pulseLen = 150;
                        txMode = TX_FOOTER2;
                        break;

                    case TX_FOOTER2:
                        if (txRepeats > 0) {
                            txRepeats--;
                            txMode = TX_SYMBOLS;
                            bufPos = 0;
                            pulseLen = pulseBuf[bufPos++];
                        }
                        else {
                            txMode = TX_IDLE;
                            rfm69->startReceive(0);
                        return;
                        }
                    default:
                        break;
                    }

                    lastBit = !lastBit;
                }
            }

            rfm69->writeFifo(&txbyte, sizeof(txbyte));
        }
        return;
    }


    uint8_t buf[32];
    uint8_t len = rfm69->getPayload(buf, sizeof(buf));

    for (uint8_t i=0; i<len; i++) {
        for (uint8_t mask=0x80; mask > 0; mask >>= 1) {
            if ( ((buf[i] & mask) != 0) != lastBit ) {
                
                if (pulseLen > SEPERATION_LEN)
                    pulseLen = SEPERATION_LEN;
                    
                pulseBuf[bufPos++] = pulseLen;
                if (bufPos == sizeof(pulseBuf))
                    bufPos = 0;
                if (bufLen < sizeof(pulseBuf))
                    bufLen++;
            
                lastBit = !lastBit;
                pulseLen = 1;
            }
            else {
                pulseLen++;
                if ((pulseLen > SEPERATION_LEN) && (bufLen > 0) ) {
                    pulseBuf[bufPos] = SEPERATION_LEN;

                    // rotate ringbuffer so that seperation pulse is located at the end (bufLen)
                    uint8_t rot = (bufPos + sizeof(pulseBuf) - bufLen + 1) % sizeof(pulseBuf);
                    rotateBuf(rot);

                    if (RcCodec::decode(pulseBuf, bufLen) == 0) {
                        // no matching decoder found
                        String l = F("RAW: ");
                        for (uint8_t i=0; i<bufLen; i++)
                            l += String(pulseBuf[i]) + ' ';
                        ws.textAll(l);
                    }

                    bufLen = 0;
                }
            }
        }
    }
}

void RcPulseTransceiver::rotateBuf(uint8_t pos) {
    uint8_t next = pos;
    uint8_t first = 0;
    while (first != next) {
        //swap
        uint8_t tmp = pulseBuf[first];
        pulseBuf[first++] = pulseBuf[next];
        pulseBuf[next++] = tmp;
        
        if (next == sizeof(pulseBuf)) {
            next = pos;
        }
        else if (first == pos) {
            pos = next;
        }
    }
}

bool RcPulseTransceiver::canHandle(AsyncWebServerRequest *request __attribute__((unused))) {
    if (request->url().startsWith(F("/send")))
        return true;
    return false;
}

void RcPulseTransceiver::handleRequest(AsyncWebServerRequest *request __attribute__((unused))) {
    if (request->method() == HTTP_GET) {
        String path = request->url().substring(6, -1);
        
        String payload = path;
        while (true) {
            int i = payload.indexOf(F("/"));
            if (i == -1)
                break;
            payload = payload.substring(i + 1, -1);
        }

        RcCodec* codec = RcCodec::encode(path, payload, pulseBuf, bufLen);
        if (codec) {
            sendPulseBuf(*codec);
            request->send(200);
        }
        else
            request->send(400, F("text/plain"), F("parameter error"));
    }
}

void RcPulseTransceiver::handleBody(AsyncWebServerRequest *request __attribute__((unused)), uint8_t *data __attribute__((unused)), size_t len __attribute__((unused)), size_t index __attribute__((unused)), size_t total __attribute__((unused))) {
    static String tmp;

    if (index == 0)
        tmp.clear();

    tmp.concat((char*) data, len);
    if (tmp.length() == total) {
        JsonDocument doc;
        if (deserializeJson(doc, tmp) == DeserializationError::Ok) {
            RcCodec *codec = RcCodec::encode(doc.as<JsonObject>(), pulseBuf, bufLen);
            if (codec) {
                sendPulseBuf(*codec);
                request->send(200);
            }
            else
                request->send(400, F("text/plain"), F("parameter error"));
        }
        else
            request->send(400, F("text/plain"), F("parse error"));
            
        tmp.clear();
    }
}

void RcPulseTransceiver::onMqttMessage(const String topic, const String payload) {
    RcCodec* codec = nullptr;

    if (topic.substring(topic.length() - 4, -1).compareTo(F("/set")) == 0) {
        codec = RcCodec::encode(topic, payload, pulseBuf, bufLen);
    }
    else if (topic.compareTo(F("send")) == 0) {
        JsonDocument doc;
        if (deserializeJson(doc, payload) == DeserializationError::Ok)
            codec = RcCodec::encode(doc.as<JsonObject>(), pulseBuf, bufLen);
    }

    if (codec)
        sendPulseBuf(*codec);
}

void RcPulseTransceiver::sendPulseBuf(RcCodec& codec) {
    txMode = TX_SYMBOLS;
    lastBit = true;
    txRepeats = codec.getTxRepeats();
    bufPos = 0;
    pulseLen = pulseBuf[bufPos++];
    rfm69->send(nullptr, 0, false); // start transmitting packet with unlimited length
}
