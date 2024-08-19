#include "868gw.h"

enum Gw868RxModes: uint8_t {
    RXMODE_TX29,        // Technloline TX21, TX25, TX27, TX29, TX37, 17241 bit/s
    RXMODE_TX35,        // Technoline TX35, 9579 bit/s
    RXMODE_TX22,        // TX22, 8842 bit/s
    RXMODE_EC3K,        // voltcraft energycount
    RXMODE_BRESSER,     // Bresser 7-in-1
    RXMODE_EMT7170      // EMT 7110, 9579 bit/s
};

static const struct {
    Gw868RxModes mode;
    uint16_t bitrate;
    uint8_t sync[8];
    uint8_t syncLen;
    uint8_t rxLen;
} MODETAB[] = {
    {RXMODE_TX29,       17241, {0x2D, 0xD4}, 2, 5},
    {RXMODE_TX35,       9579, {0x2D, 0xD4}, 2, 5},
    {RXMODE_TX22,       8842, {0x2D, 0xD4}, 2, 5},
    {RXMODE_EC3K,       20000, {0x13, 0xF1, 0x85, 0xD3, 0xAC}, 5, 60},
    {RXMODE_BRESSER,    8000, {0x2D, 0xD4}, 2, 25},
    {RXMODE_EMT7170,    9579, {0x2D, 0xD4}, 2, 12}
};

RadioApplication *radioapp = nullptr;

class LaCrosseDecoder {
public:
    static bool decode(const uint8_t *data, const size_t len, const int8_t rssi) {
        if (len < 5)
            return false;

        uint8_t crc8 = 0;
        for (int i = 0; i < 5; i++) {
            crc8 ^= data[i];
            for (int j = 0; j < 8; j++)
            if ((crc8 & 0x80) != 0)
                crc8 = (crc8 << 1) ^ 0x31;
            else
                crc8 <<= 1;
        }
        if (crc8 != 0)
            return false;

        uint8_t nibbles[10];
        for (uint8_t i = 0; i < sizeof(nibbles); i++) {
            nibbles[i] = (data[i / 2] >> (4 - ((i % 2) * 4))) & 0x0F;
        }

        uint16_t id = (nibbles[1] << 4 | nibbles[2]) & 0xFC;
        int16_t t = nibbles[3] * 100 + nibbles[4] * 10 + nibbles[5] - 400;
        uint8_t rh = (nibbles[6] << 4 | nibbles[7]) & 0x7F;
        if (rh == 0x7d) // flag for second temperature sensor (TX-25)
            id += 0x100;


        bool init = (data[1] & 0x20) != 0;
        bool batlow = (data[3] & 0x80) != 0;

        String line = "LaCrosse ID ";
        line += String(id, HEX) + ", " + String(t / 10.0) + " °C, ";
        
        if (rh <= 100)
            line += String(rh) + F(" %, ");

        if (init)
            line += F("init, ");

        if (batlow)
            line += F("low bat");
        else
            line += F("bat ok");

        ws.textAll(line);

        if (mqtt.connected()) {
            String topic = baseTopic + F("/lacrosse/") + String(id, HEX) + '/';
            JsonDocument payload;

            payload["T"] = t / 10.0;
            mqtt.publish((topic + 'T').c_str(), String(t / 10.0).c_str());

            if (rh != 0x6a) {
                mqtt.publish((topic + "RH").c_str(), String(rh).c_str());
                payload["RH"] = rh;
            }

            payload[F("batlow")] = batlow;
            payload[F("init")] = init;

            mqtt.beginPublish((topic + "state").c_str(), measureJson(payload), false);
            serializeJson(payload, mqtt);
            mqtt.endPublish();
        }

        return true;
    }
};

class EC3KDecoder {
public:
    static bool decode(uint8_t *buf, const size_t len, const int8_t rssi) {
        const uint8_t PAYLOADLEN = 41;

        if (len < (PAYLOADLEN + 2)) // payload len + 2x HDLC flag
            return false;

        descramble(buf, len);
        if (buf[0] != 0x7e)
            return false; // no HDLC frame!

        uint8_t *payload = &buf[1];
        int l = unstuffrev(payload, len - 1);

        if (l != PAYLOADLEN)
            return false;
      
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i<PAYLOADLEN; i++)
            crc = crc_ccitt_update(crc, payload[i]);

        if ( (crc != 0xF0B8) )
            return false;
        
        String line = F("EC3K ID ");

        uint16_t id = getWord(&payload[0], 4);
        line += String(id, HEX);
        
        line += F(", P: ");
        double p = getWord(&payload[15], 4) / 10.0;
        line += String(p);
        line += F(" W, Pmax: ");
        
        double pmax = getWord(&payload[17], 4) / 10.0;
        line += String(pmax);
        line += F(" W, E: ");

        uint64_t e64 = (uint64_t) getWord(&payload[33], 4) << 28 | (uint32_t) getWord(&payload[12]) << 12 | getWord(&payload[14]) >> 4;
        double eDbl = e64 / 3600.0 / 1000;
        line += String(eDbl);
        line += F(" kWh");

        ws.textAll(line);

        if (mqtt.connected()) {
            JsonDocument payload;
            String topic = baseTopic + F("/EC3K/") + String(id, HEX) + '/';

            payload[F("P")] = p;
            mqtt.publish((topic + "P").c_str(), String(p).c_str());

            payload[F("Pmax")] = pmax;

            payload[F("E")] = eDbl;
            mqtt.publish((topic + "E").c_str(), String(eDbl, 6).c_str());

            mqtt.beginPublish((topic + F("state")).c_str(), measureJson(payload), false);
            serializeJson(payload, mqtt);
            mqtt.endPublish();
        }
        return true;
    }
private:
    uint16_t static getWord(const uint8_t *buf, const uint8_t offset = 0) {
        uint16_t result;
        result = buf[0] << 8 | buf[1];
        
        result <<= offset;
        result |= buf[2] >> (8 - offset);
        return result;
    }

    static uint16_t crc_ccitt_update(uint16_t crc, uint8_t data) {
        data ^= crc & 0xFF;
        data ^= data << 4;

        return ((((uint16_t) data << 8) | (crc >> 8)) ^ (uint8_t) (data >> 4)
                ^ ((uint16_t) data << 3));
    }

    static uint8_t count1bits(const uint32_t v) {
        uint32_t tmp = v;
        uint8_t result = 0;
        while (tmp != 0) {
            result++;
            tmp &= tmp - 1;
        }
        return result;
    }

    static void descramble(uint8_t *buf, const size_t len) {
        uint32_t lfsr = 0xF185D3AC;
        const uint32_t POLY = 0x31801;
        for (size_t i = 0; i < len; i++) {
            uint8_t ob = 0;
            for (uint8_t bit = 0; bit < 8; bit++) {
                uint8_t inbit = (buf[i] >> 7) & 0x01;
                uint8_t outbit = inbit ^ (count1bits(lfsr & POLY) & 0x01);
                lfsr = lfsr << 1 | inbit;
                buf[i] <<= 1;
                ob = ob << 1 | outbit;
            }
            buf[i] = ob ^ 0xFF;
        }
    }

    static int unstuffrev(uint8_t *buf, const size_t len) {
        uint8_t cnt1bits = 0;
        uint8_t ob = 0;
        uint8_t iob = 0;
        uint8_t *po = buf;
        for (size_t i = 0; i < len; i++) {
            for (uint8_t bit = 0; bit < 8; bit++) {
                uint8_t inbit = buf[i] & 0x80;
                buf[i] <<= 1;
                if ( (cnt1bits >= 5) && (inbit == 0) ) {
                    if (cnt1bits == 6)
                        return (po - buf);
                    cnt1bits = 0;
                    continue;
                }
                if (inbit)
                    cnt1bits++;
                else
                    cnt1bits = 0;

                ob >>= 1;
                ob |= inbit;
                iob++;
                if (iob == 8) {
                    iob = 0;
                    *po++ = ob;
                }
            }
        }
        return -1;
    }
};

class EMT7170Decoder {
public:
    static bool decode(const uint8_t *data, const size_t len, const int8_t rssi) {
        if (len < 12)
            return false;

        uint8_t check = 0;
        for (uint8_t i = 0; i<12; i++)
            check += data[i];
        
        if (check != 0)
            return false;

        String line = F("EMT7170 ID ");
        uint32_t id = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
        line += String(id, HEX);

        double w = ((data[4] << 8 | data[5]) & 0x3FFF) / 2.0;
        line += F(", P: ");
        line += String(w);
        line += F(" W, I: ");

        uint16_t amps = (data[6] << 8 | data[7]) / 1000.0;
        line += String(amps);
        line += " A, U:";

        double v = data[8] / 2.0 + 128;
        line += String(v);
        line += F(" V, E: ");

        double e = ((data[9] << 8 | data[10]) & 0x3FFF) / 100.0 / 3600;
        line += String(e, 6);
        line += F(" kWh");

        ws.textAll(line);

        if (mqtt.connected()) {
            String topic = baseTopic + F("/EMT7170/") + String(id, HEX) + '/';
            JsonDocument payload;

            payload[F("P")] = w;
            mqtt.publish((topic + 'P').c_str(), String(w, 1).c_str());

            payload[F("U")] = v;
            mqtt.publish((topic + 'U').c_str(), String(v).c_str());

            payload[F("I")] = amps;
            mqtt.publish((topic + 'I').c_str(), String(amps).c_str());

            payload[F("E")] = e;
            mqtt.publish((topic + 'E').c_str(), String(e, 6).c_str());

            mqtt.beginPublish((topic + F("state")).c_str(), measureJson(payload), false);
            serializeJson(payload, mqtt);
            mqtt.endPublish();
        }

        return true;
    }
};

class Bresser7in1Decoder {
public:
    static bool decode(uint8_t *data, const size_t len, const int8_t rssi) {
        if (len < 25)
            return false;

        for (size_t i=0; i<25; i++)
            data[i] ^= 0xAA;

        uint16_t dig = lfsr_digest16(&data[2], 23, 0x8810, 0xba95);
        uint16_t msgdig = data[0] << 8 | data[1];

        if ( (dig ^ msgdig) != 0x6df1)
            return false;

        String line = F("Bresser7in1 ID: ");

        uint16_t id = data[2] << 8 | data[3];
        line += String(id, HEX);
        
        int16_t t_raw = bcdToInt(&data[14], 3);
        if (t_raw > 600)
            t_raw -= 1000;
        double t = t_raw / 10.0;

        line += F(", T: ") + String(t) + F(" °C, RH: ");

        uint8_t rh = bcdToInt(&data[16], 2);
        line += String(rh) + F(" %, ");

        double rain = bcdToInt(&data[10], 6) / 10.0;
        line += F(", rain: ") + String(rain) + F(" mm, Vgust: ");

        double vGust = bcdToInt(&data[7], 3) / 10.0;
        line += String(vGust) + F(" m/s, Vavg: ");

        double vAvg = bcdToInt(&data[8], 3, true) / 10.0;
        line += String(vAvg) + F(" m/s, Wdir: ");

        uint16_t wDir = bcdToInt(&data[4], 3);
        line += String(wDir) + F(" °, Ev: ");

        uint32_t ev = bcdToInt(&data[17], 6);
        line += String(ev) + F(" lx, UVidx: ");

        double uvIndex = bcdToInt(&data[20], 3) / 10.0;
        line += String(uvIndex) + F(", bat low ");

        uint8_t flags = data[15] & 0x0f;
        bool batlow = (flags & 0x06) != 0;
        line += String(batlow);

        ws.textAll(line);

        if (mqtt.connected()) {
            String topic = baseTopic + F("/Bresser-7in1/") + String(id, HEX) + '/';
            JsonDocument payload;

            payload[F("T")] = t;
            mqtt.publish((topic + 'T').c_str(), String(t, 1).c_str());

            payload[F("RH")] = rh;
            mqtt.publish((topic + "RH").c_str(), String(rh).c_str());

            payload[F("rain")] = rain;
            payload[F("Vgust")] = vGust;
            payload[F("Vavg")] = vAvg;
            payload[F("Wdir")] = wDir;
            payload[F("Ev")] = ev;
            payload[F("UVidx")] = uvIndex;
            payload[F("batlow")] = batlow;

            mqtt.beginPublish((topic + F("state")).c_str(), measureJson(payload), false);
            serializeJson(payload, mqtt);
            mqtt.endPublish();
        }

        return true;
    }
private:
    static uint32_t bcdToInt(const uint8_t *buf, const uint8_t digits, const bool shift = false) {
        uint32_t result = 0;
        for (uint8_t i=0; i<digits; i++) {
            result *= 10;
            uint8_t i2 = shift ? (i + 1) : i;
            result += (buf[i2 / 2] >> (4 - (i2 % 2) * 4)) & 0x0F;
        }
        return result;
    }
    static uint16_t lfsr_digest16(const uint8_t *buf, const size_t len, const uint16_t gen, const uint16_t key) {
        uint16_t sum = 0;
        uint16_t k = key;
        for (size_t i=0; i<len; i++) {
            for (int b=7; b>=0; b--) {
                if ( ((buf[i] >> b) & 1) > 0 )
                    sum ^= k;

                if ( (k & 0x01) != 0 )
                    k = (k >> 1) ^ gen;
                else
                    k >>= 1;
            }
        }
        return sum;
    }

};

Gw868::Gw868(const JsonObject &conf):
        currentRxMode(-1),
        nextSwitch(0) {
    rfm69->setFreq(868300000UL);

    Rfm69::Rfm69Config cfg[] = {
        {Rfm69::RegRxBw, 2<<5 | Rfm69::RXBWFSK_250KHZ},
        {Rfm69::RegRssiThresh, 195}, // /-0.5 dBm
        {Rfm69::RegDataModul, 0<<3}, // packet mode, FSK
        
        {Rfm69::RegPacketConfig1, 0x00}, // fixed or unlimited length, no whitening, no crc
    };
    rfm69->writeConfig(cfg, sizeof(cfg) / sizeof(cfg[0]));

    rxModes = conf[F("rxmodes")];
    interval = (conf[F("interval")] | 10) * 1000UL;
}

void Gw868::loop() {
    if ((millis() >= nextSwitch)) {
        nextSwitch = millis() + interval;

        currentRxMode++;
        while (true) {
            if ( (rxModes & (1<<currentRxMode)) != 0) {
                if ( (currentRxMode != RXMODE_EMT7170) || ((rxModes & (1<<RXMODE_TX35)) == 0) )
                    break;
            }

            currentRxMode++;
            if (currentRxMode >= (sizeof(MODETAB) / sizeof(MODETAB[0])))
                currentRxMode = 0;
        }

        auto mode = &MODETAB[currentRxMode];
        String line;
        line = F("Switch to mode ");
        line += String(currentRxMode);
        line += F(", ");
        line += String(mode->bitrate);
        line += F(" bit/s, synclen ");
        line += String(mode->syncLen);
        ws.textAll(line);

        rfm69->setBitrate(mode->bitrate);
        rfm69->setSync(mode->sync, mode->syncLen);
        currentRxLen = mode->rxLen;
        if ( (currentRxMode == RXMODE_TX35) && ((rxModes & (1<<RXMODE_EMT7170)) != 0) ) {
            currentRxLen = 12;
        }
        rfm69->startReceive(currentRxLen);
    }

    if (rfm69->payloadReady()) {
        uint8_t buf[60];
        rfm69->getPayload(buf);
        int rssi = rfm69->getRssi();

        String line;
        line = F("RFM payload: ");
        
        for (uint8_t i = 0; i < currentRxLen; i++) {
            if (buf[i] < 0x10)
                line += '0';
            line += String(buf[i], HEX) + ' ';
        }

        line += String(rssi) + F(" dBm");
        ws.textAll(line);

        switch (currentRxMode) {
        case RXMODE_TX35:
        case RXMODE_EMT7170:
            if (EMT7170Decoder::decode(buf, currentRxLen, rssi))
                break;
            // no break here!

        case RXMODE_TX29:
            LaCrosseDecoder::decode(buf, currentRxLen, rssi);
            break;

        case RXMODE_EC3K:
            EC3KDecoder::decode(buf, currentRxLen, rssi);
            break;

        case RXMODE_BRESSER:
            Bresser7in1Decoder::decode(buf, currentRxLen, rssi);
            break;
        }

        rfm69->startReceive(currentRxLen);
    }
}