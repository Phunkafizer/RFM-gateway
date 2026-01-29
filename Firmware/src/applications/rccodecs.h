#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <vector>

const uint8_t SYMBOLBUFSIZE = 70;
extern const uint16_t BITRATE;

class RcCodec {
private:
    RcCodec *next;
    uint32_t lastDecode;
    uint8_t localSymbolBuf[SYMBOLBUFSIZE];
    static RcCodec *codecs;
    static RcCodec* find(const String name);
    virtual void getDiscoveryFields(JsonDocument &doc, std::vector<JsonVariant> &fields) {(void) doc; (void) fields;}
    uint16_t tbToPulses(const uint8_t tb) const;
protected:
    struct CodecParams {
        uint8_t getNumSymbols() const;
        uint16_t timebase; // timebase in µS
        uint16_t timebase_min; //
        uint16_t timebase_max;
        uint8_t numSymbols; // number of symbols in a frame
        uint8_t numSymbolsAutoTimebase; // number of symbols to use for automatic timebase calculation. 0 = no automatic calculation
        uint8_t numTableSymbols; // number of symbols in symbolTable
        uint8_t pulsesPerSymbol; // number of pulses per symbol
        uint8_t qDecode; // "quality factor": matching windows go from x-(x/qDecode) to x+(x/qDecode)
        uint8_t footer[2];
        uint8_t txRepeats; // number of tx tries
        uint8_t symbolTable[];
    } *params;
    virtual uint8_t encodePulses(uint8_t *pulseBuf);
    virtual bool encodeSymbols(String path, String payload) = 0;
    virtual bool encodeSymbols(const JsonObject &obj) = 0;
    virtual bool decodePulses(const uint8_t *pulseBuf, const uint8_t len, const uint8_t numSymbols = 0);
    virtual void onDecodedPulses() = 0;
    void matchSymbols(const uint8_t *pulseBuf, const uint8_t len);
    String getPathSegment(const String path, const uint8_t index);
    void publish(String payload, JsonDocument &doc);
    void encodeBinLSB(const uint32_t val, const uint8_t bits, const uint8_t iHighSymbol=1);
    void encodeBinMSB(const uint32_t val, const uint8_t bits);
    uint32_t decodeBinLSB(const uint8_t start = 0, const uint8_t len = 0);
    uint32_t decodeBinMSB(const uint8_t start = 0, const uint8_t len = 0);
    PGM_P name;
public:
    static uint8_t symbolBuf[SYMBOLBUFSIZE];
    static uint8_t symbolBufLen;
    RcCodec();
    virtual ~RcCodec();
    static void freeCodecs();
    static RcCodec* encode(String path, String payload, uint8_t *pulseBuf, uint8_t &pulseBufLen);
    static RcCodec* encode(const JsonObject& obj, uint8_t *pulseBuf, uint8_t &pulseBufLen);
    static bool decode(const uint8_t *pulseBuf, const uint8_t len);
    static bool sendDiscovery(JsonDocument doc);
    uint8_t getTxRepeats() const;
    void getFooter(uint16_t footer[2]) const;
};

/* Tristate coding (Intertechno old, ...)
          ____              ____
Bit 0 : _|    |____________|    |____________ 
         |<4a>|<---12a---->|<4a>|<---12a--->|
          ____________      ____________ 
Bit 1 : _|            |____|            |____ 
         |<---12a---->|<4a>|<---12a---->|<4a>|
          ____              ____________ 
Bit f:  _|    |____________|            |____ 
         |<4a>|<---12a---->|<---12a---->|<4a>|
          ____              ____
Bit x:  _|    |____________|    |____
         |<4a>|<---12a---->|<4a>|<4a>|
          ____ 
Sync :  _|    |___________ _ _ _ ____________ 
         |<4a>|<-----------124a------------->|
*/
class TristateCodec: public RcCodec {
private:
    static RcCodec::CodecParams defParams;
public:
    TristateCodec();
};

class ITTristate: public TristateCodec {
private:
    void getDiscoveryFields(JsonDocument &doc, std::vector<JsonVariant> &fields) override;
protected:
    void onDecodedPulses() override;
    bool encodeSymbols(const char house, const uint8_t group, const uint8_t channel, const bool on);
    bool encodeSymbols(String path, String payload) override;
    bool encodeSymbols(const JsonObject &obj) override;
public:
    ITTristate();
};


/* pulse position coding (Intertechno self learning, HAMA, Nexus, Telldus, ...
             ____      ____
Bit 0    : _|    |____|    |____________________

            |<1T>|<1T>|<1T>|<--------5T-------->|
             ____                      ____
Bit 1    : _|    |____________________|    |____

            |<1T>|<--------5T-------->|<1T>|<1T>|
             ____      ____ 
Bit X    : _|    |____|    |____

            |<1T>|<1T>|<1T>|<1T>|
             ____                       
Sync     : _|    |_________ _ _ _ ________

            |<1T>|<----------11T--------->|
*/
class IT32: public RcCodec {
private:
    static RcCodec::CodecParams defParams;
    void getDiscoveryFields(JsonDocument &doc, std::vector<JsonVariant> &fields) override;
protected:
    uint8_t encodePulses(uint8_t *pulseBuf) override;
    bool encodeSymbols(String path, String payload) override;
    bool encodeSymbols(const JsonObject &obj) override;
    bool decodePulses(const uint8_t *pulseBuf, const uint8_t len, const uint8_t numSymbols = 0) override;
    void onDecodedPulses() override;
public:
    IT32();
};


/* pulse width coding 1:2 (Pilota casa, ...)
             ________      
Bit 0    : _|        |____

            |  <2T>  |<1T>|
             ____ 
Bit 1    : _|    |________

            |<1T>|  <2T>  |
*/
class PilotaCasa: public RcCodec {
private:
    static struct CmdTable {
        uint8_t data;
        uint8_t group; // 1..4, 0 all
        uint8_t channel; // 1..4 0 all
        uint8_t cmd; // 0 off, 1 on
    } cmdTable[];
    static RcCodec::CodecParams defParams;
    void getDiscoveryFields(JsonDocument &doc, std::vector<JsonVariant> &fields) override;
protected:
    bool encodeSymbols(String path, String payload) override;
    bool encodeSymbols(const JsonObject &obj) override;
    void onDecodedPulses() override;
public:
    PilotaCasa();
};


/* pulse width coding 1:3 (devices with chip EV1527. EMYLO, logilight, REV doorbell, Heitronic doorbell)
   https://www.sunrom.com/download/EV1527.pdf

             ____ 
Bit 0    : _|    |____________
            |<1T>|    <3T>    |
             ____________      
Bit 1    : _|            |____
            |    <3T>    |<1T>|
*/
class EV1527Codec: public RcCodec {
private:
    static RcCodec::CodecParams defParams;
    void getDiscoveryFields(JsonDocument &doc, std::vector<JsonVariant> &fields) override;
protected:
    void encodeSymbols(const uint32_t code, const uint8_t data);
    bool encodeSymbols(String path, String payload) override;
    bool encodeSymbols(const JsonObject &obj) override;
    void onDecodedPulses() override;
    void decodeSymbols(uint32_t &id, uint8_t &data);
public:
    EV1527Codec();
};

class Emylo: public EV1527Codec {
private:
    void getDiscoveryFields(JsonDocument &doc, std::vector<JsonVariant> &fields) override;
protected:
    bool encodeSymbols(String path, String payload) override;
    bool encodeSymbols(const JsonObject &obj) override;
    void onDecodedPulses() override;
public:
    Emylo();

};


/* pulse width coding 2:3
https://www.mikrocontroller.net/attachment/15335/fs20-protokoll.txt
             ____ 
Bit 0    : _|    |____
            |<2T>|<2T>|
             ______      
Bit 1    : _|      |______
            | <3T> | <3T> |
*/
class FS20Codec: public RcCodec {
private:
    static RcCodec::CodecParams defParams;
    bool checkEvenParity(const uint16_t value);
    bool getByte(const uint8_t pos, uint8_t &value);
    void encodeSymbols(const uint16_t house, const uint8_t address, const uint16_t command);
    void encodeByte(const uint8_t b);
    uint16_t strToCmd(const String &str);
    void getDiscoveryFields(JsonDocument &doc, std::vector<JsonVariant> &fields) override;
protected:
    bool encodeSymbols(String path, String payload) override;
    bool encodeSymbols(const JsonObject &obj) override;
    bool decodePulses(const uint8_t *pulseBuf, const uint8_t len, const uint8_t numSymbols = 0) override;
    void onDecodedPulses() override;
public:
    FS20Codec();
};