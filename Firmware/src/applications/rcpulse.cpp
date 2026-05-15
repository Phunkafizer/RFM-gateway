#include "rcpulse.h"

const uint8_t SEPERATION_LEN = 120;
const uint8_t MIN_NUM_PULSES = 48;
PGM_P EP_RF_CAPABILITIES = "/api/rf/capabilities";
PGM_P EP_RF_TRANSMIT = "/api/rf/transmit";

RcPulseTransceiver::RcPulseTransceiver(const uint32_t freq, const uint32_t f_low, const uint32_t f_high):
        freq(freq),
        f_low(f_low),
        f_high(f_high),
        bufPos(0),
        bufLen(0),
        lastBit(false),
        pulseLen(1),
        txMode(TX_IDLE) {
    Rfm69::Rfm69Config cfg[] = {
        {Rfm69::RegRxBw, Rfm69::DccFreq::DCC_0_125 | Rfm69::RXBWASK_250KHZ},
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
    switch (txMode) {
    case TX_DATA: {
        bool txDone = false;
        while (!txDone && (rfm69->getFifoLevel() < Rfm69::FIFO_FULL)) {
            uint8_t txbyte = 0x00;
            uint8_t bitmask = 0x80;
            uint8_t bitsWritten = 0;

            while (bitmask) {
                if (pulseLen == 0) {
                    if (bufPos == bufLen) {
                        if (txRepeats > 0) {
                            txRepeats--;
                            bufPos = 0;
                        }
                        else {
                            txDone = true;
                            break;
                        }
                    }

                    pulseLen = RcCodec::decodeTb(pulseBuf[bufPos++]);
                }

                if (lastBit)
                    txbyte |= bitmask;
                bitmask >>= 1;
                bitsWritten++;

                pulseLen--;
                if (pulseLen == 0)
                    lastBit = !lastBit;
            }

            // Flush final partial byte as padded low level to avoid truncating end of frame.
            if (bitsWritten > 0)
                rfm69->writeFifo(&txbyte, sizeof(txbyte));
        }

        if (txDone)
            txMode = TX_DRAIN;

        return;
    }

    case TX_DRAIN:
        if (rfm69->getFifoLevel() == Rfm69::FIFO_EMPTY) {
            txMode = TX_IDLE;
            rfm69->startReceive(0);
        }
        return;

    case TX_IDLE:
    default:
        break;
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
                            ws.textAll(F("<div class='rfframe'>"));
                            if (!RcCodec::decode(pulseBuf, bufLen)) {
                                // no matching decoder found
                                String l = F("RAW: ");
                                for (uint8_t i=0; i<bufLen; i++)
                                    l += String(pulseBuf[i]) + ' ';
                                ws.textAll(l);
                            }
                            ws.textAll(F("</div>"));
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

    if (request->method() == HTTP_GET && request->url().compareTo(FPSTR(EP_RF_CAPABILITIES)) == 0)
        return true;

    if (request->method() == HTTP_POST && request->url().compareTo(FPSTR(EP_RF_TRANSMIT)) == 0)
        return true;


    return false;
}

void RcPulseTransceiver::handleRequest(AsyncWebServerRequest *request) {
    if (request->method() == HTTP_GET) {
        if (request->url().compareTo(FPSTR(EP_RF_CAPABILITIES)) == 0) {
            JsonDocument doc;
            doc[F("device_name")] = F("RFM Gateway");

            JsonArray ranges = doc[F("supported_frequency_ranges")].to<JsonArray>();
            JsonArray range = ranges.add<JsonArray>();
            range.add(f_low);
            range.add(f_high);

            JsonArray mods = doc[F("supported_modulations")].to<JsonArray>();
            mods.add(F("ook"));

            AsyncResponseStream *response = request->beginResponseStream(FPSTR(APP_JSON));
            serializeJson(doc, *response);
            request->send(response);
            return;
        }

        if (!request->url().startsWith(F("/send/"))) {
            request->send(404, F("text/plain"), F("not found"));
            return;
        }

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
            txRepeats = codec->getTxRepeats();
            sendPulseBuf();
            request->send(200);
        }
        else
            request->send(400, F("text/plain"), F("parameter error"));

        return;
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
            if (request->url().compareTo(FPSTR(EP_RF_TRANSMIT)) == 0) {
                Serial.println("Transmit request: " + tmp);

                JsonArray timings = doc[F("timings_us")].as<JsonArray>();
                bufLen = 0;
                for (JsonVariant v : timings)
                    pulseBuf[bufLen++] = RcCodec::encodeTb(abs(v.as<int16_t>()));

                if (bufLen == 0) {
                    request->send(400, F("text/plain"), F("missing timings"));
                    return;
                }

                txRepeats = doc[F("repeat_count")] | 3;
                sendPulseBuf();
                request->send(200);
                return;
            }

            RcCodec *codec = RcCodec::encode(doc, pulseBuf, bufLen);
            if (codec) {
                txRepeats = codec->getTxRepeats();
                sendPulseBuf();
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

    if (codec) {
        txRepeats = codec->getTxRepeats();
        sendPulseBuf();
    }
    else
        ws.textAll(F("encoding error!"));

    return true;
}

bool RcPulseTransceiver::sendDiscovery(JsonDocument &doc) {
    return RcCodec::sendDiscovery(doc);
}

void RcPulseTransceiver::sendPulseBuf() {
    txMode = TX_DATA;
    lastBit = true;
    bufPos = 0;
    pulseLen = 0;
    rfm69->send(nullptr, 0, false); // start transmitting packet with unlimited length
}
