#include "rcpulse.h"

const uint8_t SEPERATION_LEN = 120;
const uint8_t MIN_NUM_PULSES = 48;

RcPulseTransceiver::RcPulseTransceiver(const uint32_t freq):
        bufPos(0),
        bufLen(0),
        lastBit(false),
        pulseLen(1),
        txMode(TX_IDLE) {
    Rfm69::Rfm69Config cfg[] = {
        {Rfm69::RegRxBw, 2<<5 | Rfm69::RXBWASK_250KHZ},
        {Rfm69::RegSyncConfig, 1<<6}, // no sync, FifoFillCondition set
        {Rfm69::RegRssiThresh, 160}, // /-0.5 dBm
        {Rfm69::RegDataModul, 1<<3}, // OOK
        {Rfm69::RegOokPeak, 1<<6 | 3<<0}, // peak threshold, increment every 8 chips
        {Rfm69::RegPreambleMsb, 0},
        {Rfm69::RegPreambleLsb, 0},
        {Rfm69::RegPacketConfig1, 0x00}, // fixed or unlimited length, no whitening, no crc
    };

    rfm69->writeConfig(cfg, sizeof(cfg) / sizeof(cfg[0]));
    rfm69->setFreq(freq);
    rfm69->setBitrate(BITRATE);
    rfm69->setTxPower(13);
    rfm69->startReceive(0);
}

void RcPulseTransceiver::restartReceive() {
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
                            pulseLen = footer[0];
                            txMode = TX_FOOTER1;
                            break;
                        }
                        else
                            pulseLen = pulseBuf[bufPos++];
                        break;

                    case TX_FOOTER1:
                        pulseLen = footer[1];
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
    if (txQue.size() > 0) {
        onMqttMessage(txQue[0].path, txQue[0].payload);
        txQue.erase(txQue.begin());
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
                if (pulseLen > SEPERATION_LEN) {
                    pulseBuf[bufPos] = SEPERATION_LEN;

                    if (bufLen >= MIN_NUM_PULSES) {
                        // rotate ringbuffer so that seperation pulse is located at the end (bufLen)
                        uint8_t rot = (bufPos + sizeof(pulseBuf) - bufLen + 1) % sizeof(pulseBuf);
                        rotateBuf(rot);

                        uint8_t cnt1 = 0;
                        for (uint8_t i=0; i<bufLen; i++) {
                            if (pulseBuf[i] == 1)
                                cnt1++;
                        }

                        if (cnt1 < 5) {
                            if (!RcCodec::decode(pulseBuf, bufLen)) {
                                // no matching decoder found
                                String l = F("RAW: ");
                                for (uint8_t i=0; i<bufLen; i++)
                                    l += String(pulseBuf[i]) + ' ';
                                l += F("<hr>");
                                ws.textAll(l);
                            }
                        }
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
    if (request->url().startsWith(F("/send/")))
        return true;

    return false;
}

void RcPulseTransceiver::handleRequest(AsyncWebServerRequest *request) {
    if (request->method() == HTTP_GET) {
        if (txMode != TX_IDLE) {
            request->send(409, F("text/plain"), F("transmitter busy"));
            return;
        }
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
            RcCodec *codec = RcCodec::encode(doc, pulseBuf, bufLen);
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

bool RcPulseTransceiver::onMqttMessage(const String topic, const String payload) {
    RcCodec* codec = nullptr;

    if (topic.substring(topic.length() - 4, -1).compareTo(F("/set")) == 0) {
        if (txMode != TX_IDLE) {
            txQue.push_back({topic, payload});
            return true;
        }
        codec = RcCodec::encode(topic, payload, pulseBuf, bufLen);
    }
    else if (topic.compareTo(F("send")) == 0) {
        if (txMode != TX_IDLE) {
            txQue.push_back({topic, payload});
            return true;
        }
        JsonDocument doc;
        if (deserializeJson(doc, payload) == DeserializationError::Ok)
            codec = RcCodec::encode(doc, pulseBuf, bufLen);
    }
    else 
        return false;

    if (codec)
        sendPulseBuf(*codec);
    else
        ws.textAll(F("encoding error!"));

    return true;
}

bool RcPulseTransceiver::sendDiscovery(JsonDocument &doc) {
    return RcCodec::sendDiscovery(doc);
}

void RcPulseTransceiver::sendPulseBuf(RcCodec& codec) {
    txMode = TX_SYMBOLS;
    lastBit = true;
    txRepeats = codec.getTxRepeats();
    codec.getFooter(footer);
    bufPos = 0;
    pulseLen = pulseBuf[bufPos++];
    rfm69->send(nullptr, 0, false); // start transmitting packet with unlimited length
}
