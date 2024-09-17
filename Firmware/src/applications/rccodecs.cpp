#include "rccodecs.h"
#include "main.h"

const uint16_t BITRATE = 20000;
const uint16_t PULSEWIDTHUS = 1000000UL / BITRATE; // samplingtime of tranceiver / µS

static const char STR_COMMAND[] PROGMEM = "command";

RcCodec* RcCodec::codecs = nullptr;
uint8_t RcCodec::symbolBuf[SYMBOLBUFSIZE];
uint8_t RcCodec::symbolBufLen;

RcCodec::RcCodec():        
        next(codecs),
        lastDecode(0) {
    codecs = this;
    name = nullptr;
}

RcCodec::~RcCodec() {
    if (next) {
        free(next);
        next = nullptr;
    }
}

void RcCodec::freeCodecs() {
    if (codecs != nullptr)
        delete(codecs);
    codecs = nullptr;
}

bool RcCodec::decode(const uint8_t *pulseBuf, const uint8_t len) {
    bool decoded = false;
    RcCodec *codec = codecs;
    while (codec != nullptr) {
        if (codec->decodePulses(pulseBuf, len)) {
            if (codec->lastDecode < (millis() - 500)) {
                codec->onDecodedPulses();
            }
            else
                if (memcmp(codec->localSymbolBuf, codec->symbolBuf, codec->symbolBufLen) != 0)
                    codec->onDecodedPulses();

            memcpy(codec->localSymbolBuf, codec->symbolBuf, codec->symbolBufLen);
            codec->lastDecode = millis();
            decoded = true;
        }
        codec = codec->next;
    }
    return decoded;
}

uint8_t RcCodec::getTxRepeats() const {
    return params->txRepeats;
}

/**
 * @brief decodes a pulsebuf using given codec params
 * Decoded symbols are saved in static variable "symbolBuffer"
 * @return number of successfully decoded symbols
 */
bool RcCodec::decodePulses(const uint8_t *pulseBuf, const uint8_t len) {
    if (len < params->numSymbols * params->pulsesPerSymbol + 2) // enough pulses in received buffer?
        return false;

    const uint8_t *ppb = pulseBuf + len - params->numSymbols * params->pulsesPerSymbol - 2;
    
    uint16_t timebase = params->timebase;
    // automatic timebase calculation
    if (params->numSymbolsAutoTimebase > 0) {
        uint16_t sumPulses = 0;
        for (uint8_t i=0; i<params->numSymbolsAutoTimebase * params->pulsesPerSymbol; i++)
            sumPulses += ppb[i];

        //calculate duration of a symbol, we always use 1st symbol in symTable
        uint8_t symDur = 0;
        for (uint8_t i=0; i<params->pulsesPerSymbol; i++)
            symDur += params->symbolTable[i];

        timebase = (uint32_t) sumPulses * PULSEWIDTHUS / params->numSymbolsAutoTimebase / symDur;
        if ( (timebase < params->timebase_min) || (timebase > params->timebase_max) )
            return false;
    }

    uint8_t *symTabLow = (uint8_t*) malloc(params->numTableSymbols * params->pulsesPerSymbol);
    uint8_t *symTabHigh = (uint8_t*) malloc(params->numTableSymbols * params->pulsesPerSymbol);

    // calculate matching windows for pulses

    #ifdef DEBUGMATCHINGTABLES
    Serial.print("Matching table: ");
    #endif
    for (uint8_t s=0; s<params->numTableSymbols; s++) {
        for (uint8_t p=0; p<params->pulsesPerSymbol; p++) {
            uint8_t i = s * params->pulsesPerSymbol + p;
            
            // calculate matching windows in units of receiver's samplingtime
            symTabLow[i] = ((timebase * params->symbolTable[i] * (params->qDecode-1)) / params->qDecode + PULSEWIDTHUS / 2) / PULSEWIDTHUS;
            symTabHigh[i] = ((timebase * params->symbolTable[i] * (params->qDecode+1)) / params->qDecode + PULSEWIDTHUS / 2) / PULSEWIDTHUS;
            #ifdef DEBUGMATCHINGTABLES
            Serial.print(String(symTabLow[i]) + '-' + String(symTabHigh[i]) + ',');
            #endif
        }
    }
    #ifdef DEBUGMATCHINGTABLES
    Serial.println("");
    #endif

    uint8_t bp = 0;
    symbolBufLen = 0;

    while (bp + params->pulsesPerSymbol <= len) {
        uint8_t s;

        #ifdef DEBUGRCDECODER
        Serial.print("Matching pulses ");
        for (uint8_t p=0; p<params->pulsesPerSymbol; p++)
            Serial.print(String(pulseBuf[bp+p]) + ' ');
        Serial.print(": ");
        #endif

        for (s=0; s<params->numTableSymbols; s++) {
            uint8_t p;

            for (p=0; p<params->pulsesPerSymbol; p++) {
                const uint8_t ist = s * params->pulsesPerSymbol + p; // calculate index in symboltable
                const uint8_t ipb = bp + p; // calculate index in pulse buffer
                if (ipb == 0) // skip very first pulse, it could be distorted by leading noise
                    continue;
                if ( (ppb[ipb] < symTabLow[ist]) || (ppb[ipb] > symTabHigh[ist]) )
                    break; //current pulse does not match to symbol to check
            }

            #ifdef DEBUGRCDECODER
            Serial.print("p " + String(p) + ", ");
            #endif

            if (p == params->pulsesPerSymbol) {
                // matching symbol found, continue with checking next pulses!
                break;
            }
            // check for next symbol
        }

        #ifdef DEBUGRCDECODER
        Serial.println("s " + String(s));
        #endif

        if (s == params->numTableSymbols) {
            //no matching symbol found
            break;
        }
        else
            symbolBuf[symbolBufLen++] = s;

        bp += params->pulsesPerSymbol;
    }

    #ifdef DEBUGRCDECODER
    String str;
    for (uint8_t i=0; i<symbolBufLen; i++)
        str += char('0' + symbolBuf[i]);
    Serial.println("RX Symbols: " + str);
    #endif

    free(symTabLow);
    free(symTabHigh);

    return (params->numSymbols == bp / params->pulsesPerSymbol);
}

RcCodec* RcCodec::find(const String name) {
    RcCodec* codec = codecs;
    while (codec != nullptr) {
        if (codec->name != nullptr) {
            if (name.compareTo(codec->name) == 0)
                return codec;
        }
        codec = codec->next;
    }
    return nullptr;
}

RcCodec* RcCodec::encode(String path, String payload, uint8_t *pulseBuf, uint8_t &pulseBufLen) {
    String name = path.substring(0, path.indexOf('/'));

    RcCodec* codec = find(name);
    if (codec != nullptr) {
        // strip off protocol name
        path = path.substring(path.indexOf('/') + 1, -1);
        if (codec->encodeSymbols(path, payload)) {
            pulseBufLen = codec->encodePulses(pulseBuf);
            return codec;
        }
    }
    
    return nullptr;
}

RcCodec* RcCodec::encode(const JsonObject& obj, uint8_t *pulseBuf, uint8_t &pulseBufLen) {
    String name = obj[F("protocol")];

    RcCodec* codec = find(name);
    if (codec != nullptr) {
        if (codec->encodeSymbols(obj)) {
            pulseBufLen = codec->encodePulses(pulseBuf);
            return codec;
        }
    }

    return nullptr;
}

uint8_t RcCodec::encodePulses(uint8_t *pulseBuf) {
    uint8_t result = 0;
    
    for (uint8_t s=0; s<symbolBufLen; s++) {
        for (uint8_t p=0; p<params->pulsesPerSymbol; p++) {
            uint16_t pulse = (params->symbolTable[symbolBuf[s] * params->pulsesPerSymbol + p] * params->timebase + (PULSEWIDTHUS / 2)) / PULSEWIDTHUS;
            pulseBuf[result++] = pulse;
        }
    }
    return result;
}


String RcCodec::getPathSegment(const String path, const uint8_t index) {
    String tmp = path;

    uint8_t pos = tmp.indexOf('/');
    uint8_t left = index;
    while (left > 0) {
        tmp = tmp.substring(pos + 1, -1);
        pos = tmp.indexOf('/');
        left--;
    }
    return tmp.substring(0, pos);
}

/**
 * @brief encodes a value to symbolBuf
 * 
 * @param val value to be encoded
 * @param bits number of bits to be encoded
 * @param highSym index of symbol to be used for a high bit in value
*/
void RcCodec::encodeBinLSB(const uint32_t val, const uint8_t bits, const uint8_t iHighSymbol) {
    for (uint32_t mask=1; mask<=1UL<<(bits-1); mask <<= 1)
        symbolBuf[symbolBufLen++] = (val & mask) != 0 ? iHighSymbol : 0;
}

uint32_t RcCodec::decodeBinLSB() {
    uint32_t result = 0;
    for (uint8_t i=0; i<symbolBufLen; i++)
        if (symbolBuf[i] > 0)
            result |= 1<<i;
    return result;
}

/**
 * publish RF received data
 */
void RcCodec::publish(String path, String payload, JsonDocument &doc) {
    String topic = baseTopic + '/' + name + '/' + path;
    mqtt.publish(topic.c_str(), payload.c_str());

    ws.textAll(F("Received protocol ") + String(name));
    ws.textAll(F("MQTT: ") + topic + F("/set ") + payload);
    
    topic = F("send/") + String(name) + "/" + path;
    if (!payload.isEmpty())
        topic += "/" + payload;
    
    ws.textAll(F("HTTP: <a href=\"") + topic + F("\" target=\"_blank\">http://") + WiFi.localIP().toString() + "/" + topic + F("</a>"));

    doc[F("protocol")] = FPSTR(name);
    topic = baseTopic + F("/received");
    mqtt.beginPublish(topic.c_str(), measureJson(doc), false);
    serializeJson(doc, mqtt);
    mqtt.endPublish();
}


RcCodec::CodecParams TristateCodec::defParams = {
    350,        // timebase
    300, 400,   // timebase min / max
    12,         // numSymbols
    4,          // numSymbolsAutoTimebase; Leading 4 symbols are housecode, following symbols maybe shorter (multicast) because of symbol X
    4,          // numTableSymbols
    4,          // number of pulses per symbol
    2,          // rx quality factor q, matching windows s-(s/q) <= x <= s+(s/q)
    {1, 31},    // footer
    5,          // tx tries
    {
        1, 3, 1, 3,     // symbol 0: bit 0
        3, 1, 3, 1,     // symbol 1: bit 1
        1, 3, 3, 1,     // symbol 2: bit F
        1, 3, 1, 1,     // symbol 3: bit X (used by intertechno, used for setting all channels in all groups in one house)
    }
};

TristateCodec::TristateCodec() {
    params = &defParams;
}


ITTristate::ITTristate() {
    name = PSTR("ittristate");
}

bool ITTristate::encodeSymbols(const char house, const uint8_t group, const uint8_t channel, const bool on) {
    char sub = 0;
    if ( (house >= 'A') && (house <= 'P') )
        sub = 'A';
    else if ( (house >= 'a') && (house <= 'p') )
        sub = 'a';
    else
        return false;

    symbolBufLen = 0;
    encodeBinLSB(house - sub, 4, 2); // intertechno tristate uses symbol 'F' (index 2) as high

    if ( (channel < 1) || (channel > 4) || (group < 1) || (group > 4) ) {
        encodeBinLSB(3, 2, 3); // switch all channels
        encodeBinLSB(3, 2, 3); // switch all groups
    }
    else {
        encodeBinLSB(channel - 1, 2, 2);
        encodeBinLSB(group - 1, 2, 2);
    }
    
    encodeBinLSB(2, 2, 2); // bit 9-10 fix 0F
    encodeBinLSB(on ? 3 : 1, 2, 2); // on = FF
    return true;
}

bool ITTristate::encodeSymbols(String path, String payload) {
    // path for ittristate: <house A-P>/<group 1-4>/<channel 1-4>
    
    String sHouse = getPathSegment(path, 0);
    uint8_t group = getPathSegment(path, 1).toInt();
    uint8_t channel = getPathSegment(path, 2).toInt();
    bool on = payload == F("on");

    if (sHouse.length() == 1)
        return encodeSymbols(sHouse.charAt(0), group, channel, on);
    return false;
}

bool ITTristate::encodeSymbols(const JsonObject &obj) {
    if ( !obj[F("house")].is<const char*>() || !obj[FPSTR(STR_COMMAND)].is<const char*>() )
        return false;

    String sHouse = obj[F("house")].as<String>();
    uint8_t group = obj[F("group")];
    uint8_t channel = obj[F("channel")];
    bool on = obj[FPSTR(STR_COMMAND)].as<String>() == F("on");

    return encodeSymbols(sHouse.charAt(0), group, channel, on);
}

void ITTristate::onDecodedPulses() {
    uint16_t bin = decodeBinLSB();

    char house = (bin & (0x0F)) + 'A';
    uint8_t act = bin >> 10;
    String payload;
    if (act == 0b11)
        payload = F("on");
    else
        if (act == 0b01)
            payload = F("off");
        else
            return;

    String path(house);
    JsonDocument doc;    

    doc[F("house")] = String(house);

    if ( (symbolBuf[4] == 3) && (symbolBuf[5] == 3) && (symbolBuf[6] == 3) && (symbolBuf[7] == 3) ) {
        // switch all channels & groups
    }
    else {
        uint8_t channel = ((bin >> 4) & 0x03) + 1;
        uint8_t group = ((bin >> 6) & 0x03) + 1;
        path += '/' + String(group) + '/' + String(channel); 
        doc[F("channel")] = channel;
        doc[F("group")] = group;
    }

    doc[FPSTR(STR_COMMAND)] = payload;
    publish(path, payload, doc);
}





RcCodec::CodecParams IT32::defParams = {
    275,        // timebase
    250, 350,   // timebase min / max
    32,         // numSymbols; dimmers may have for symbols
    27,         // 27 symbols are always 0 or 1
    3,          // number of symbols in symboltable
    4,          // number of pulses per symbol
    3,          // rx quality factor q, matching windows s-(s/q) <= x <= s+(s/q)
    {1, 39},    // footer
    5,          // tx tries
    {
        1, 1, 1, 5,     // symbol 0: 0b0
        1, 5, 1, 1,     // symbol 1: 0b1
        1, 1, 1, 1      // symbol 2: x
    }
};

IT32::IT32() {
    name = PSTR("intertechno");
    params = &defParams;
}

uint8_t IT32::encodePulses(uint8_t *pulseBuf) {
    pulseBuf[0] = 1;
    pulseBuf[1] = 60;
    return RcCodec::encodePulses(&pulseBuf[2]) + 2;
}

bool IT32::encodeSymbols(String path, String payload) {
    // path for intertechno: <id>/<channel>
    uint32_t id = getPathSegment(path, 0).toInt();
    uint8_t channel = getPathSegment(path, 1).toInt();

    symbolBufLen = 0;

    encodeInt(id, 26);
    encodeInt(0, 1); // group bit

    payload.toLowerCase();
    if (payload == F("on"))
        encodeInt(1, 1);
    else
        encodeInt(0, 1);

    encodeInt(channel - 1, 4);
    return true;
}

bool IT32::encodeSymbols(const JsonObject &obj) {
    return true;

}

bool IT32::decodePulses(const uint8_t *pulseBuf, const uint8_t len) {
    // TODO check also for special (longer) frames for dimmers
    return RcCodec::decodePulses(pulseBuf, len);
}

void IT32::onDecodedPulses() {
    uint32_t data = 0;
    for (int i=0; i<symbolBufLen; i++) {
        data <<= 1;
        if (symbolBuf[i] != 0)
            data |= 1;
    }

    uint32_t id = data >> 6;
    bool group = (data >> 5) & 0x01;
    bool on = (data >> 4) & 0x01;
    uint8_t channel = (data & 0x0F);

    String path = String(id);
    if (!group)
        path += '/' + String(channel + 1);
    String payload = on ? F("on") : F("off");

    JsonDocument doc;
    doc[F("id")] = id;
    doc[FPSTR(STR_COMMAND)] = payload;
    
    publish(path, payload, doc);
}

void IT32::encodeInt(const uint32_t val, const uint8_t bits) {
    for (uint32_t mask=1<<(bits-1); mask>0; mask >>= 1)
        symbolBuf[symbolBufLen++] = (val & mask) != 0 ? 1 : 0;
}



PilotaCasa::CmdTable PilotaCasa::cmdTable[] = {
    {0b110001, 1, 1, 1}, {0b111110, 1, 1, 0}, {0b011001, 1, 2, 1}, {0b010001, 1, 2, 0},
	{0b101001, 1, 3, 1}, {0b100001, 1, 3, 0}, {0b111010, 2, 1, 1}, {0b110010, 2, 1, 0},
	{0b010110, 2, 2, 1}, {0b011010, 2, 2, 0}, {0b100110, 2, 3, 1}, {0b101010, 2, 3, 0},
	{0b110111, 3, 1, 1}, {0b111011, 3, 1, 0}, {0b011111, 3, 2, 1}, {0b010111, 3, 2, 0},
	{0b101111, 3, 3, 1}, {0b100111, 3, 3, 0}, {0b111101, 4, 1, 1}, {0b110101, 4, 1, 0},
	{0b010011, 4, 2, 1}, {0b011101, 4, 2, 0}, {0b100011, 4, 3, 1}, {0b101101, 4, 3, 0},
	{0b101100, 0, 0, 1}, {0b011100, 0, 0, 0}
};

RcCodec::CodecParams PilotaCasa::defParams = {
    600,        // timebase
    500, 700,   // timebase min / max
    32,         // numSymbols
    32,         // numSymbolsAutoTimebase
    2,          // numTableSymbols
    2,          // number of pulses per symbol
    3,          // rx quality factor q, matching windows s-(s/q) <= x <= s+(s/q)
    {1, 40},    // footer TODO check values!
    5,          // tx tries
    {
        2, 1,   // symbol 0: 0b0
        1, 2,   // symbol 1: 0b1
    }
};

PilotaCasa::PilotaCasa() {
    name = PSTR("pilota");
    params = &defParams;
}

bool PilotaCasa::encodeSymbols(String path, String payload) {
    // path for pilota: <id>/<group>/<channel>
    uint32_t id = getPathSegment(path, 0).toInt();
    uint8_t channel = getPathSegment(path, 1).toInt();
    uint8_t group = getPathSegment(path, 2).toInt();

    payload.toLowerCase();
    uint8_t cmd = payload == "on" ? 1 : 0;

    uint32_t data = 0;
    data |= id << 8;

    for (uint8_t i=0; i<sizeof(cmdTable) / sizeof(cmdTable[0]); i++) {
        if ( (cmdTable[i].group == group) && (cmdTable[i].channel == channel) && (cmdTable[i].cmd == cmd) ) {

            for (uint8_t bit=0; bit<32; bit++) {
                symbolBuf[bit] = (data & 0x80000000UL) != 0 ? 1 : 0;
                data <<= 1;
            }
            symbolBufLen = 32;
            return true;
        }
    }
    return false;
}

bool PilotaCasa::encodeSymbols(const JsonObject &obj) {
    return true;
}

void PilotaCasa::onDecodedPulses() {
    uint32_t data = 0;
    for (int i=0; i<symbolBufLen; i++) {
        data <<= 1;
        if (symbolBuf[i] != 0)
            data |= 1;
    }

    uint16_t id = data >> 8;
    uint8_t cmd = (data >> 24) & 0x3F;
    uint8_t i=0;
    for (i=0; i<sizeof(cmdTable) / sizeof(cmdTable[0]); i++) {
        if (cmd == cmdTable[i].data)
            break;
    }

    if (i<sizeof(cmdTable) / sizeof(cmdTable[0])) {
        String path = String(id);
        if (cmdTable[i].channel > 0) {
            path += '/' + String(cmdTable[i].group) + '/' + String(cmdTable[i].channel);
        }
        String payload = cmdTable[i].cmd == 0 ? F("off") : F("on");

        JsonDocument doc;
        doc[F("id")] = id;
        doc[F("group")] = cmdTable[i].group;
        doc[F("channel")] = cmdTable[i].channel;
        doc[FPSTR(STR_COMMAND)] = payload;
        publish(path, payload, doc);
    }
}

RcCodec::CodecParams EV1527Codec::defParams = {
    250,        // timebase
    220, 360,   // timebase min / max
    24,         // numSymbols
    24,         // numSymbolsAutoTimebase
    2,          // numTableSymbols
    2,          // number of pulses per symbol
    3,          // rx quality factor q, matching windows s-(s/q) <= x <= s+(s/q)
    {1, 31},    // footer
    5,          // tx tries
    {
        1, 3,   // symbol 0: 0b0
        3, 1,   // symbol 1: 0b1
    }
};

EV1527Codec::EV1527Codec() {
    name = PSTR("EV1527");
    params = &defParams;
}

void EV1527Codec::encodeSymbols(const uint32_t code, const uint8_t data) {
    symbolBufLen = 0;
    encodeBinLSB(code, 20);
    encodeBinLSB(data, 4);
}

bool EV1527Codec::encodeSymbols(String path, String payload) {
    // path for EV1527Codec: <ID>/<data>
    const uint32_t code = getPathSegment(path, 0).toInt();
    const uint8_t data = getPathSegment(path, 1).toInt();
    encodeSymbols(code, data);
    return true;
}

bool EV1527Codec::encodeSymbols(const JsonObject &obj) {
    return true;
}

void EV1527Codec::decodeSymbols(uint32_t &id, uint8_t &data) {
    id = 0;
    for (int i=0; i<20; i++) {
        if (symbolBuf[i] != 0)
            id |= 1UL<<i;
    }

    data = 0;
    for (int i=0; i<4; i++) {
        if (symbolBuf[i+20] != 0)
            data |= 1<<i;
    }
}

void EV1527Codec::onDecodedPulses() {
    uint32_t id;
    uint8_t data;
    decodeSymbols(id, data);

    JsonDocument doc;
    doc[F("id")] = id;
    doc[F("data")] = data;
    publish(String(id) + '/' + data, F("press"), doc);
}


Emylo::Emylo() {
    name = PSTR("emylo");
}

bool Emylo::encodeSymbols(String path, String payload) {
    // path for Emylo: <ID>, payload: 'A'-'D'
    const uint32_t code = getPathSegment(path, 0).toInt();

    payload.toUpperCase();
    const char c = payload.charAt(0);
    if ( (c < 'A') || c > 'D')
        return false;

    /*
        0b1000 -> button A
        0b0100 -> button B
        0b0010 -> button C
        0b0001 -> button D
    */
    const uint8_t data = 0b1000 >> (c - 'A');
    EV1527Codec::encodeSymbols(code, data);
    return true;
}

bool Emylo::encodeSymbols(const JsonObject &obj) {
    return true;
}

void Emylo::onDecodedPulses() {
    uint32_t id;
    uint8_t data;
    decodeSymbols(id, data);

    char key;
    switch (data) {
    case 8: key = 'A'; break;
    case 4: key = 'B'; break;
    case 2: key = 'C'; break;
    case 1: key = 'D'; break;
    default: key = 0; break;
    }

    if (key) {
        JsonDocument doc;
        doc[F("id")] = id;
        doc[F("key")] = String(key);
        publish(String(id), String(key), doc);
    }
}


RcCodec::CodecParams FS20Codec::defParams = {
    200,        // timebase
    150, 250,   // timebase min / max
    10,         // 13 sync + 18 housecode + 9 address + 9 command + 9 checksum + 1 stop, optional + 9 bits extended command TODO numSymbols
    0,          // numSymbolsAutoTimebase; no automatic timebase calculation, because symbols are different in lengths
    2,          // 
    2,          // 2 pulses per symbol
    5,          // rx quality factor
    {1, 30},    // TODO check values!
    5,          // tx tries
    {
        2, 2,   // symbol 0: 0b0
        3, 3    // symbol 1: 0b1
    }
};

FS20Codec::FS20Codec() {
    name = PSTR("FS20");
    params = &defParams;
}

bool FS20Codec::encodeSymbols(String path, String payload) {
    // TODO 
    return false;
}

bool FS20Codec::encodeSymbols(const JsonObject &obj) {
    return true;
}

bool FS20Codec::decodePulses(const uint8_t *pulseBuf, const uint8_t len) {
    if (len >= 117) {
        const uint8_t *p = pulseBuf + len - 115; // omitt decoding of 1st symbol as first pulse might be enlarged by noise

        uint8_t numSymbols = RcCodec::decodePulses(p, len - 2);
        return numSymbols;
    }

    return 0;
}

void FS20Codec::onDecodedPulses() {

}