#include "868gw.h"
#include "HADiscLocal.h"

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

static const char STR_T[] PROGMEM = "T";
static const char STR_RH[] PROGMEM = "RH";
static const char STR_RAIN[] PROGMEM = "rain";
static const char STR_VAVG[] PROGMEM = "Vavg";
static const char STR_EV[] PROGMEM = "Ev";
static const char STR_WDIR[] PROGMEM = "Wdir";
static const char STR_P[] PROGMEM = "P";
static const char STR_E[] PROGMEM = "E";
static const char STR_U[] PROGMEM = "U";
static const char STR_UVIDX[] PROGMEM = "UVidx";
static const char STR_RSSI[] PROGMEM = "rssi";

const struct {
    const char* field;
    const char *devClass;
    const char *unit;
} DISC_FIELDS[] PROGMEM = {
    {STR_T,     "temperature",      "°C"},
    {STR_RH,    "humidity",         "%"},
    {STR_RAIN,  "precipitation",    "mm"},
    {STR_VAVG,  "wind_speed",       "m/s"},
    {STR_EV,    "illuminance",      "lx"},
    {STR_WDIR,  "wind_direction",   "°"},
    {STR_P,     "power",            "W"},
    {STR_E,     "energy",           "kWh"},
    {STR_U,     "voltage",          "v"},
    {STR_UVIDX, "uv_index",         ""},
    {STR_RSSI,  "signal_strength",  "dBm"}
};

RadioApplication *radioapp = nullptr;

Decoder::Decoder(const char *name):
        name(name) {
}

bool Decoder::operator==(const String& other) const {
    return String(name) == other;
}

String Decoder::getTopic(String id) {
    return baseTopic + F("/") + String(name) + F("/") + id + F("/");
}

void Decoder::publish(String id, JsonDocument &payload, const int8_t rssi) {
    String topic = getTopic(id) + F("state");
    payload[F("protocol")] = String(name);
    payload[F("id")] = id;
    payload[FPSTR(STR_RSSI)] = rssi;

    radioapp->publish(topic, payload);
}

LaCrosseDecoder::LaCrosseDecoder():
        Decoder(PSTR("lacrosse")) {
}

bool LaCrosseDecoder::decode(const uint8_t *data, const size_t len, const int8_t rssi) {
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
    for (uint8_t i = 0; i < sizeof(nibbles); i++)
        nibbles[i] = (data[i / 2] >> (4 - ((i % 2) * 4))) & 0x0F;

    uint16_t id = (nibbles[1] << 4 | nibbles[2]) & 0xFC;
    double t = (nibbles[3] * 100 + nibbles[4] * 10 + nibbles[5] - 400) / 10.0;
    uint8_t rh = (nibbles[6] << 4 | nibbles[7]) & 0x7F;
    if (rh == 0x7d) // flag for second temperature sensor (TX-25)
        id += 0x100;

    bool init = (data[1] & 0x20) != 0;
    bool batlow = (data[3] & 0x80) != 0;

    if (mqtt.connected()) {
        String topic = getTopic(String(id, HEX));
        JsonDocument payload;

        payload[FPSTR(STR_T)] = t;

        if (rh < 100)
            payload["RH"] = rh;

        payload[F("batlow")] = batlow;
        payload[F("init")] = init;

        publish(String(id, HEX), payload, rssi);
    }

    return true;
}

EC3KDecoder::EC3KDecoder():
        Decoder(PSTR("EC3K")) {
}

bool EC3KDecoder::decode(uint8_t *buf, const size_t len, const int8_t rssi) {
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
    
    uint16_t id = getWord(&payload[0], 4);
    double p = getWord(&payload[15], 4) / 10.0;    
    double pmax = getWord(&payload[17], 4) / 10.0;

    uint64_t e64 = (uint64_t) getWord(&payload[33], 4) << 28 | (uint32_t) getWord(&payload[12]) << 12 | getWord(&payload[14]) >> 4;
    double eDbl = e64 / 3600.0 / 1000;

    if (mqtt.connected()) {
        JsonDocument payload;
        String topic = getTopic(String(id, HEX));

        payload[FPSTR(STR_P)] = p;
        mqtt.publish((topic + "P").c_str(), String(p).c_str());

        payload[F("Pmax")] = pmax;

        payload[FPSTR(STR_E)] = eDbl;
        mqtt.publish((topic + "E").c_str(), String(eDbl, 6).c_str());

        publish(String(id, HEX), payload, rssi);
    }
    return true;
}

uint16_t EC3KDecoder::getWord(const uint8_t *buf, const uint8_t offset) {
    uint16_t result;
    result = buf[0] << 8 | buf[1];
    
    result <<= offset;
    result |= buf[2] >> (8 - offset);
    return result;
}

uint16_t EC3KDecoder::crc_ccitt_update(uint16_t crc, uint8_t data) {
    data ^= crc & 0xFF;
    data ^= data << 4;

    return ((((uint16_t) data << 8) | (crc >> 8)) ^ (uint8_t) (data >> 4)
            ^ ((uint16_t) data << 3));
}

uint8_t EC3KDecoder::count1bits(const uint32_t v) {
    uint32_t tmp = v;
    uint8_t result = 0;
    while (tmp != 0) {
        result++;
        tmp &= tmp - 1;
    }
    return result;
}

void EC3KDecoder::descramble(uint8_t *buf, const size_t len) {
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

int EC3KDecoder::unstuffrev(uint8_t *buf, const size_t len) {
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


EMT7170Decoder::EMT7170Decoder():
        Decoder(PSTR("EMT7170")) {
}

bool EMT7170Decoder::decode(const uint8_t *data, const size_t len, const int8_t rssi) {
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
        String topic = getTopic(String(id, HEX));
        JsonDocument payload;

        payload[FPSTR(STR_P)] = w;
        mqtt.publish((topic + 'P').c_str(), String(w, 1).c_str());

        payload[FPSTR(STR_U)] = v;
        mqtt.publish((topic + 'U').c_str(), String(v).c_str());

        payload[F("I")] = amps;
        mqtt.publish((topic + 'I').c_str(), String(amps).c_str());

        payload[FPSTR(STR_E)] = e;
        mqtt.publish((topic + 'E').c_str(), String(e, 6).c_str());

        publish(String(id, HEX), payload, rssi);
    }

    return true;
}


Bresser7in1Decoder::Bresser7in1Decoder():
        Decoder(PSTR("Bresser7in1")) {
}

bool Bresser7in1Decoder::decode(uint8_t *data, const size_t len, const int8_t rssi) {
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

    uint8_t rh = bcdToInt(&data[16], 2);
    double rain = bcdToInt(&data[10], 6) / 10.0;
    double vGust = bcdToInt(&data[7], 3) / 10.0;
    line += String(vGust) + F(" m/s, Vavg: ");

    double vAvg = bcdToInt(&data[8], 3, true) / 10.0;
    uint16_t wDir = bcdToInt(&data[4], 3);
    uint32_t ev = bcdToInt(&data[17], 6);
    double uvIndex = bcdToInt(&data[20], 3) / 10.0;

    uint8_t flags = data[15] & 0x0f;
    bool batlow = (flags & 0x06) != 0;

    if (mqtt.connected()) {
        String topic = getTopic(String(id, HEX));
        JsonDocument payload;

        payload[FPSTR(STR_T)] = t;
        mqtt.publish((topic + 'T').c_str(), String(t, 1).c_str());

        payload[FPSTR(STR_RH)] = rh;
        mqtt.publish((topic + "RH").c_str(), String(rh).c_str());

        payload[FPSTR(STR_RAIN)] = rain;
        payload[F("Vgust")] = vGust;
        payload[FPSTR(STR_VAVG)] = vAvg;
        payload[FPSTR(STR_WDIR)] = wDir;
        payload[FPSTR(STR_EV)] = ev;
        payload[FPSTR(STR_UVIDX)] = uvIndex;
        payload[F("batlow")] = batlow;

        publish(String(id, HEX), payload, rssi);
    }

    return true;
}

uint32_t Bresser7in1Decoder::bcdToInt(const uint8_t *buf, const uint8_t digits, const bool shift) {
    uint32_t result = 0;
    for (uint8_t i=0; i<digits; i++) {
        result *= 10;
        uint8_t i2 = shift ? (i + 1) : i;
        result += (buf[i2 / 2] >> (4 - (i2 % 2) * 4)) & 0x0F;
    }
    return result;
}

uint16_t Bresser7in1Decoder::lfsr_digest16(const uint8_t *buf, const size_t len, const uint16_t gen, const uint16_t key) {
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
        line = F("<hr>RFM payload: ");
        
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
            if (emt7170.decode(buf, currentRxLen, rssi))
                break;
            [[fallthrough]];

        case RXMODE_TX29:
        case RXMODE_TX22:
            lacrosse.decode(buf, currentRxLen, rssi);
            break;

        case RXMODE_EC3K:
            ec3k.decode(buf, currentRxLen, rssi);
            break;

        case RXMODE_BRESSER:
            bresser7in1.decode(buf, currentRxLen, rssi);
            break;
        }

        ws.textAll(F("<hr>"));

        rfm69->startReceive(currentRxLen);
    }
}

bool Gw868::sendDiscovery(JsonDocument &doc) {
    const String proto = doc[F("protocol")].as<String>();
    const String id = doc[F("id")].as<String>();
    String haName = proto + String(" ") + id;
    String topic = baseTopic + F("/") + proto + F("/") + id + F("/state");

    auto generateHaId = [&](const char *suffix) {
        String haId = proto + F("_") + id + F("_") + String(suffix);
        return haId;
    };

    auto haPublish = [&](const char *suffix) {
        haDisc.setStateTopic(topic);
        String tmpl = F("{{ value_json.") + String(suffix);
        tmpl += F(" }}");
        haDisc.setValueTemplate(tmpl);
        haDisc.setExpire(300); // expire after 5 min
        haDisc.setAvailability(getAvailabilityTopic());
        haDisc.publish();
    };

    for (unsigned int i=0; i<sizeof(DISC_FIELDS)/sizeof(DISC_FIELDS[0]); i++) {
        if (!doc[FPSTR(DISC_FIELDS[i].field)].isNull()) {    
            String haId = generateHaId(DISC_FIELDS[i].field);
            haDisc.createSensor(haName, haId);
            haDisc.setDeviceClass(FPSTR(DISC_FIELDS[i].devClass));
            haDisc.setUnit(FPSTR(DISC_FIELDS[i].unit));
            haPublish(DISC_FIELDS[i].field);
        }
    }
    return true;

/*
    if (lacrosse == proto)
        Serial.println("THIS IS LACROSSE DISC!");*/
}