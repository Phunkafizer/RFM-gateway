#pragma once
#include "../radioapplication.h"

class Decoder {
private:
    const char *name;
protected:
    Decoder(const char *name);
    String getTopic(String id);
public:
    void publish(String topic, JsonDocument &payload, const int8_t rssi);
    bool operator==(const String& other) const;
};

class LaCrosseDecoder: public Decoder {
public:
    LaCrosseDecoder();
    bool decode(const uint8_t *data, const size_t len, const int8_t rssi);
};

class EC3KDecoder: public Decoder {
private:
    uint16_t getWord(const uint8_t *buf, const uint8_t offset = 0);
    uint16_t crc_ccitt_update(uint16_t crc, uint8_t data);
    uint8_t count1bits(const uint32_t v);
    void descramble(uint8_t *buf, const size_t len);
    int unstuffrev(uint8_t *buf, const size_t len);
public:
    EC3KDecoder();
    bool decode(uint8_t *buf, const size_t len, const int8_t rssi);
};

class EMT7170Decoder: public Decoder {
public:
    EMT7170Decoder();
    bool decode(const uint8_t *data, const size_t len, const int8_t rssi);
};

class Bresser7in1Decoder: public Decoder {
private:
    uint32_t bcdToInt(const uint8_t *buf, const uint8_t digits, const bool shift = false);
    uint16_t lfsr_digest16(const uint8_t *buf, const size_t len, const uint16_t gen, const uint16_t key);
public:
    Bresser7in1Decoder();
    bool decode(uint8_t *data, const size_t len, const int8_t rssi);
};


class Gw868: public RadioApplication {
private:
    uint8_t currentRxMode;
    uint16_t rxModes; // bitmask
    uint8_t currentRxLen;
    unsigned long nextSwitch;
    uint32_t interval;
    LaCrosseDecoder lacrosse;
    EC3KDecoder ec3k;
    EMT7170Decoder emt7170;
    Bresser7in1Decoder bresser7in1;
public:
    Gw868(const JsonObject &conf);
    void loop();
    bool sendDiscovery(JsonDocument &doc) override;
};