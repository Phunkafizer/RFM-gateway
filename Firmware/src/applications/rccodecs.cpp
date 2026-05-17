#include "rccodecs.h"
#include "main.h"
#include "HADiscLocal.h"
#include "radioapplication.h"

const uint16_t BITRATE = 20000;
const uint16_t PULSEWIDTHUS = 1000000UL / BITRATE; // samplingtime of tranceiver / µS

static const char STR_PROTOCOL[] PROGMEM = "protocol";
static const char STR_ID[] PROGMEM = "id";
static const char STR_COMMAND[] PROGMEM = "command";
static const char STR_HOUSE[] PROGMEM = "house";
static const char STR_ADDRESS[] PROGMEM = "address";
static const char STR_CHANNEL[] PROGMEM = "channel";
static const char STR_GROUP[] PROGMEM = "group";
static const char STR_DATA[] PROGMEM = "data";
static const char STR_KEY[] PROGMEM = "key";
static const char STR_TIMEBASE[] PROGMEM = "timebase";
static const char STR_ON[] PROGMEM = "ON";
static const char STR_OFF[] PROGMEM = "OFF";
static const char STR_EXT[] PROGMEM = "ext";
static const char STR_PRESS[] PROGMEM = "press";
static const char STR_ENCODING_ERROR[] PROGMEM ="encoding error!";

RcCodec* RcCodec::codecs = nullptr;
uint8_t RcCodec::symbolBuf[SYMBOLBUFSIZE];
uint8_t RcCodec::symbolBufLen;
uint16_t RcCodec::last_timebase = 0;

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
            if ( (codec->lastDecode < (millis() - 500)) || (memcmp(codec->localSymbolBuf, codec->symbolBuf, codec->symbolBufLen) != 0) ) {
                codec->onDecodedPulses();
            }

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
 * @brief decodes given pulsebuf to symbols
 * Decoded symbols are saved in static variable "symbolBuffer"
 * number of decoded symbols is saved in static variable "symbolBufLen"
 */
void RcCodec::matchSymbols(const uint8_t *pulseBuf, const uint8_t len) {
    uint16_t timebase = params->timebase;
    symbolBufLen = 0;

    // automatic timebase calculation
    if (params->numSymbolsAutoTimebase > 0) {
        uint16_t sumPulses = 0;
        for (uint8_t i=0; i<params->numSymbolsAutoTimebase * params->pulsesPerSymbol; i++)
            sumPulses += pulseBuf[i];

        //calculate duration of a symbol, we always use 1st symbol in symTable
        uint8_t symDur = 0;
        for (uint8_t i=0; i<params->pulsesPerSymbol; i++)
            symDur += params->symbolTable[i];

        timebase = (uint32_t) sumPulses * PULSEWIDTHUS / params->numSymbolsAutoTimebase / symDur;
        if ( (timebase < params->timebase_min) || (timebase > params->timebase_max) )
            return;
    }
    last_timebase = timebase;

    const uint8_t SYMTAB_MAX = 32;
    static uint8_t symTabLow[SYMTAB_MAX];
    static uint8_t symTabHigh[SYMTAB_MAX];

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
                if ( (pulseBuf[ipb] < symTabLow[ist]) || (pulseBuf[ipb] > symTabHigh[ist]) ) {
                     //current pulse does not match to symbol to check, try matching next symbol
                    #ifdef DEBUGRCDECODER
                    Serial.print("!");
                    #endif
                    break;
                }
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

        if (s == params->numTableSymbols) {
            //no matching symbol found
            break;
        }
        else {
            #ifdef DEBUGRCDECODER
            Serial.println("s " + String(s));
            #endif
            symbolBuf[symbolBufLen++] = s;
        }

        bp += params->pulsesPerSymbol;
    }
    #ifdef DEBUGSYMBOLBUF
    String str = F("Decoded symbols: ");
    for (uint8_t i=0; i<symbolBufLen; i++)
        str += char('0' + symbolBuf[i]);
    Serial.println(str);
    #endif
}

bool RcCodec::decodePulses(const uint8_t *pulseBuf, const uint8_t len, const uint8_t numSymbols) {
    const uint8_t ns = (numSymbols == 0) ? params->numSymbols : numSymbols;
    const uint8_t np = ns * params->pulsesPerSymbol;

    if (len < np + 2) // enough pulses in received buffer?
        return false;

    const uint8_t *buf = pulseBuf + len - np - 2;
    matchSymbols(buf, np);
    
    return (symbolBufLen == ns) && checkSymbolBuf();
}

bool RcCodec::checkSymbolBuf() {
    return true;
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
        symbolBufLen = 0;
        if (codec->encodeSymbols(path, payload)) {
            #ifdef DEBUGSYMBOLBUF
            Serial.print(F("Encoded symbols: "));
            for (uint8_t i=0; i<symbolBufLen; i++)
                Serial.print((char) (symbolBuf[i] + '0'));
            Serial.println("");
            #endif
            pulseBufLen = codec->encodePulses(pulseBuf);
            return codec;
        }
    }
    ws.textAll(FPSTR(STR_ENCODING_ERROR));
    return nullptr;
}

RcCodec* RcCodec::encode(JsonDocument &doc, uint8_t *pulseBuf, uint8_t &pulseBufLen) {
    RcCodec* codec = find(doc[FPSTR(STR_PROTOCOL)]);
    if (codec != nullptr) {
        symbolBufLen = 0;
        if (codec->encodeSymbols(doc)) {
            uint16_t timebase = doc[FPSTR(STR_TIMEBASE)].isNull() ? codec->params->timebase : doc[FPSTR(STR_TIMEBASE)];
            pulseBufLen = codec->encodePulses(pulseBuf, timebase);
            return codec;
        }
    }
    ws.textAll(FPSTR(STR_ENCODING_ERROR));
    return nullptr;
}

static const uint8_t SHIFT_LUT[] = {0, 1, 2, 4};

uint8_t RcCodec::tbToPulses(const uint8_t ticks, const uint16_t timebase) const {
    uint16_t tb = (timebase == 0) ? params->timebase : timebase;
    return encodeTb(tb * ticks);
}

/**
    * @brief encodes a time duration in µS to a value in units of receiver's sampling time
    * The result is packed into 1 byte, 2 bits for exponent, 6 bits for mantissa
*/
uint8_t RcCodec::encodeTb(const uint16_t time) {
    uint16_t ticks = (time + (PULSEWIDTHUS / 2)) / PULSEWIDTHUS;

    // pack into 1 byte, 2 bits for exponent, 6 bits for mantissa
    uint8_t exp_idx = 0;
    uint16_t mant = ticks;

    while (true) {
        uint8_t shift = SHIFT_LUT[exp_idx];
        if (shift > 0)
            mant = (ticks + (1 << (shift - 1))) >> shift;

        if ( (mant <= (1<<6)) || (exp_idx == sizeof(SHIFT_LUT) - 1) )
            break;

        exp_idx++;
    }

    return (exp_idx << 6) | ((mant - 1) & 0x3F);
}

uint16_t RcCodec::decodeTb(const uint8_t packed) {
    const uint8_t exp_idx = (packed >> 6) & 0x03;
    const uint16_t mant = (packed & 0x3F) + 1;
    return mant << SHIFT_LUT[exp_idx];
}

/**
 * @brief encodes pulsebuf from symbols
 * set symbolBuf and symbolBufLen before calling this function
 * @return number of pulses in pulsebuf
 */
uint8_t RcCodec::encodePulses(uint8_t *pulseBuf, const uint16_t timebase) {
    uint8_t result = 0;

    for (uint8_t s=0; s<symbolBufLen; s++) {
        for (uint8_t p=0; p<params->pulsesPerSymbol; p++) {
            pulseBuf[result++] = tbToPulses(params->symbolTable[symbolBuf[s] * params->pulsesPerSymbol + p], timebase);
        }
    }
    pulseBuf[result++] = tbToPulses(params->footer[0], timebase);
    pulseBuf[result++] = tbToPulses(params->footer[1], timebase);
    return result;
}

String RcCodec::getPathSegment(const String path, const uint8_t index) {
    String tmp = path;

    int pos = tmp.indexOf('/');
    uint8_t left = index;
    while (left > 0) {
        if (pos == -1)
            return String();
        tmp = tmp.substring(pos + 1, -1);
        pos = tmp.indexOf('/');
        left--;
    }
    return tmp.substring(0, pos);
}

bool RcCodec::sendDiscovery(JsonDocument &doc) {
    RcCodec* codec = find(doc[FPSTR(STR_PROTOCOL)]);
    if (codec != nullptr) {
        std::vector<JsonVariant> fields;
        codec->getDiscoveryFields(doc, fields);
        if (fields.size() == 0)
            return false;

        String haName = String(codec->name);
        String haId = String(codec->name);
        String topic = baseTopic + F("/") + String(codec->name);
        for (auto &field : fields) {
            haName += F(" ") + field.as<String>();
            haId += F("_") + field.as<String>();
            topic += F("/") + field.as<String>();
        }

        String cmdTopic = topic + F("/set");

        bool clear = doc[F("clear")] | false;

        return codec->sendDiscovery(haName, haId, topic, cmdTopic, clear);
    }
    return false;
}

bool RcCodec::sendDiscovery(String &name, String &id, String &stateTopic, String &cmdTopic, const bool clear) {
    haDisc.createSwitch(name, id, cmdTopic);
    haDisc.setStateTopic(stateTopic);
    haDisc.setAvailability(getAvailabilityTopic());
    return haDisc.publish(!clear);
}

void RcCodec::sendMqttState(JsonDocument &doc) {
    if (mqtt.connected()) {
        std::vector<JsonVariant> fields;
        getDiscoveryFields(doc, fields);
        String topic = baseTopic + F("/") + String(name);
        for (auto &field : fields) {
            topic += F("/") + field.as<String>();
        }
        String payload = doc[FPSTR(STR_COMMAND)];
        
        mqtt.publish(topic.c_str(), payload.c_str());
        radioapp->publish("", doc); // creates discovery button in webUI
    }
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

void RcCodec::encodeBinMSB(const uint32_t val, const uint8_t bits) {
    for (uint32_t mask=1<<(bits-1); mask>0; mask >>= 1)
        symbolBuf[symbolBufLen++] = (val & mask) != 0 ? 1 : 0;
}

uint32_t RcCodec::decodeBinLSB(const uint8_t start, const uint8_t len) {
    uint32_t result = 0;
    uint8_t end = (len != 0) ? len : symbolBufLen;
    for (uint8_t i=0; i<end; i++)
        if (symbolBuf[i + start] > 0)
            result |= 1<<i;
    return result;
}

uint32_t RcCodec::decodeBinMSB(const uint8_t start, const uint8_t len) {
    uint32_t result = 0;
    uint8_t end = (len != 0) ? len : symbolBufLen;
    for (uint8_t i=0; i<end; i++) {
        result <<= 1;
        if (symbolBuf[i + start] > 0)
            result |= 1;
    }
    return result;
}

/**
 * publish RF received data
 */
void RcCodec::publish(String payload, JsonDocument &doc) {
    String path;
    std::vector<JsonVariant> fields;
    getDiscoveryFields(doc, fields);
    for (auto &field : fields) {
        path += '/';
        path += field.as<String>();
    }

    String topic = baseTopic + '/' + FPSTR(name);
    topic += path;

    if (mqtt.connected())
        mqtt.publish(topic.c_str(), payload.c_str());

    ws.textAll(String(F("Received protocol ")) + FPSTR(name));
    ws.textAll(String(F("MQTT: ")) + topic + F("/set ") + payload);
    
    topic = F("send/") + String(name) + path;
    if (!payload.isEmpty()) {
        topic += "/" + payload;
        doc[FPSTR(STR_COMMAND)] = payload;
    }

    ws.textAll(F("HTTP: <a href=\"") + topic + F("\" target=\"_blank\">http://") + WiFi.localIP().toString() + "/" + topic + F("</a>"));

    if (mqtt.connected()) {
        doc[FPSTR(STR_PROTOCOL)] = String(name);
        if (params->numSymbolsAutoTimebase != 0)
            doc[F("timebase")] = last_timebase;

        topic = baseTopic + F("/received");
        radioapp->publish(topic, doc);
    }
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
    5,          // tx repeats
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

bool ITTristate::encodeSymbols(String path, String payload) {
    // path for ittristate: <house A-P>/<group 1-4>/<channel 1-4> or <house A-P> for all
    
    const String sHouse = getPathSegment(path, 0);
    const String sGroup = getPathSegment(path, 1);
    const String sChannel = getPathSegment(path, 2);

    JsonDocument doc;
    doc[FPSTR(STR_HOUSE)] = sHouse;
    if (!sGroup.isEmpty())
        doc[FPSTR(STR_GROUP)] = sGroup.toInt();
    if (!sChannel.isEmpty())
        doc[FPSTR(STR_CHANNEL)] = sChannel.toInt();
    doc[FPSTR(STR_COMMAND)] = payload;

    return encodeSymbols(doc);
}

bool ITTristate::encodeSymbols(JsonDocument &doc) {
    if ( !doc[FPSTR(STR_HOUSE)].is<String>() || !doc[FPSTR(STR_COMMAND)].is<String>() )
        return false;

    if (doc[FPSTR(STR_HOUSE)].as<String>().length() != 1)
        return false;

    String sHouse = doc[FPSTR(STR_HOUSE)];
    sHouse.toUpperCase();
    char house = sHouse.charAt(0);
    if ( (house < 'A') || (house > 'P') )
        return false;

    house -= 'A';

    encodeBinLSB(house, 4, 2); // intertechno tristate uses symbol 'F' (index 2) as high

    const uint8_t channel = doc[FPSTR(STR_CHANNEL)];
    const uint8_t group = doc[FPSTR(STR_GROUP)];
    bool on = doc[FPSTR(STR_COMMAND)].as<String>().equalsIgnoreCase(FPSTR(STR_ON));

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
    doc[FPSTR(STR_PROTOCOL)] = String(name);
    sendMqttState(doc);
    return true;
}

void ITTristate::getDiscoveryFields(JsonDocument &doc, std::vector<JsonVariant> &fields) {
    fields.push_back(doc[FPSTR(STR_HOUSE)]);
    if (!doc[FPSTR(STR_GROUP)].isNull())
        fields.push_back(doc[FPSTR(STR_GROUP)]);
    if (!doc[FPSTR(STR_CHANNEL)].isNull())
        fields.push_back(doc[FPSTR(STR_CHANNEL)]);
}

void ITTristate::onDecodedPulses() {
    uint16_t bin = decodeBinLSB();

    char house = (bin & (0x0F)) + 'A';
    uint8_t act = bin >> 10;
    String payload;
    if (act == 0b11)
        payload = FPSTR(STR_ON);
    else
        if (act == 0b01)
            payload = FPSTR(STR_OFF);
        else
            return;

    JsonDocument doc;    

    doc[FPSTR(STR_HOUSE)] = String(house);

    if ( (symbolBuf[4] == 3) && (symbolBuf[5] == 3) && (symbolBuf[6] == 3) && (symbolBuf[7] == 3) ) {
        // switch all channels & groups
    }
    else {
        uint8_t channel = ((bin >> 4) & 0x03) + 1;
        uint8_t group = ((bin >> 6) & 0x03) + 1;
        doc[FPSTR(STR_CHANNEL)] = channel;
        doc[FPSTR(STR_GROUP)] = group;
    }

    publish(payload, doc);
}



RcCodec::CodecParams IT32::defParams = {
    275,        // timebase
    225, 350,   // timebase min / max
    32,         // numSymbols; dimmers may have for symbols
    27,         // 27 symbols are always 0 or 1
    3,          // number of symbols in symboltable
    4,          // number of pulses per symbol
    3,          // rx quality factor q, matching windows s-(s/q) <= x <= s+(s/q)
    {1, 39},    // footer
    4,          // tx repeats
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

uint8_t IT32::encodePulses(uint8_t *pulseBuf, const uint16_t timebase) {
    // add sync symbol 
    pulseBuf[0] = tbToPulses(1, timebase);
    pulseBuf[1] = tbToPulses(10, timebase);
    return RcCodec::encodePulses(&pulseBuf[2], timebase) + 2;
}

bool IT32::encodeSymbols(String path, String payload) {
    // path for intertechno: <id>/<channel>
    const String sId = getPathSegment(path, 0);
    const String sChannel = getPathSegment(path, 1);
    if (sId.isEmpty() || sChannel.isEmpty())
        return false;

    JsonDocument doc;
    doc[FPSTR(STR_ID)] = sId.toInt();
    doc[FPSTR(STR_CHANNEL)] = sChannel.toInt();
    doc[FPSTR(STR_COMMAND)] = payload;

    return encodeSymbols(doc);
}

bool IT32::encodeSymbols(JsonDocument &doc) {
    if ( !doc[FPSTR(STR_ID)].is<uint32_t>() || !doc[FPSTR(STR_CHANNEL)].is<uint8_t>() || !doc[FPSTR(STR_COMMAND)].is<String>() )
        return false;

    const uint32_t id = doc[FPSTR(STR_ID)];
    const uint8_t channel = doc[FPSTR(STR_CHANNEL)];
    bool on = doc[FPSTR(STR_COMMAND)].as<String>().equalsIgnoreCase(FPSTR(STR_ON));

    encodeBinMSB(id, 26);
    encodeBinMSB(0, 1); // group bit
    encodeBinMSB(on ? 1 : 0, 1);
    encodeBinMSB(channel - 1, 4);
    sendMqttState(doc);
    return true;
}

void IT32::getDiscoveryFields(JsonDocument &doc, std::vector<JsonVariant> &fields) {
    fields.push_back(doc[FPSTR(STR_ID)]);
    if (!doc[FPSTR(STR_CHANNEL)].isNull())
        fields.push_back(doc[FPSTR(STR_CHANNEL)]);
}

bool IT32::decodePulses(const uint8_t *pulseBuf, const uint8_t len, const uint8_t numSymbols) {
    // TODO check also for special (longer) frames for dimmers
    // can be done in new method checkRxLen(), then this overridden method is not needed anymore
    return RcCodec::decodePulses(pulseBuf, len, numSymbols);
}

void IT32::onDecodedPulses() {
    uint32_t data = decodeBinMSB(0, 32);

    const bool group = (data >> 5) & 0x01;
    const bool on = (data >> 4) & 0x01;
    String payload = on ? FPSTR(STR_ON) : FPSTR(STR_OFF);

    JsonDocument doc;
    doc[FPSTR(STR_ID)] = data >> 6;
    if (!group)
        doc[FPSTR(STR_CHANNEL)] = (data & 0x0F) + 1;
    
    publish(payload, doc);
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
    5,          // tx repeats
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
    JsonDocument doc;

    doc[FPSTR(STR_ID)] = getPathSegment(path, 0).toInt();
    doc[FPSTR(STR_GROUP)] = getPathSegment(path, 1).toInt();
    doc[FPSTR(STR_CHANNEL)] = getPathSegment(path, 2).toInt();
    doc[FPSTR(STR_COMMAND)] = payload;

    return encodeSymbols(doc);
}

bool PilotaCasa::encodeSymbols(JsonDocument &doc) {
    if ( !doc[FPSTR(STR_ID)].is<uint32_t>() || !doc[FPSTR(STR_GROUP)].is<uint8_t>() || !doc[FPSTR(STR_CHANNEL)].is<uint8_t>() || !doc[FPSTR(STR_COMMAND)].is<String>() )
        return false;

    uint32_t id = doc[FPSTR(STR_ID)];
    uint8_t group = doc[FPSTR(STR_GROUP)];
    uint8_t channel = doc[FPSTR(STR_CHANNEL)];
    uint8_t cmd = doc[FPSTR(STR_COMMAND)].as<String>().equalsIgnoreCase(FPSTR(STR_ON)) ? 1 : 0;

    uint32_t data = 0xFF;
    data |= id << 8;
    // 2 highest bits in data are not set (device type?)

    for (uint8_t i=0; i<sizeof(cmdTable) / sizeof(cmdTable[0]); i++) {
        if ( (cmdTable[i].group == group) && (cmdTable[i].channel == channel) && (cmdTable[i].cmd == cmd) ) {
            data |= cmdTable[i].data << 24;
            encodeBinMSB(data, 32);
            sendMqttState(doc);
            return true;
        }
    }
    return false;
}

void PilotaCasa::getDiscoveryFields(JsonDocument &doc, std::vector<JsonVariant> &fields) {
    fields.push_back(doc[FPSTR(STR_ID)]);
    if (!doc[FPSTR(STR_GROUP)].isNull())
        fields.push_back(doc[FPSTR(STR_GROUP)]);
    if (!doc[FPSTR(STR_CHANNEL)].isNull())
        fields.push_back(doc[FPSTR(STR_CHANNEL)]);
}

void PilotaCasa::onDecodedPulses() {
    uint32_t data = decodeBinMSB(0, symbolBufLen);

    uint16_t id = data >> 8;
    uint8_t cmd = (data >> 24) & 0x3F;
    // 2 highest bits in data are ignored (device type?)
    uint8_t i=0;
    for (i=0; i<sizeof(cmdTable) / sizeof(cmdTable[0]); i++) {
        if (cmd == cmdTable[i].data)
            break;
    }

    if (i<sizeof(cmdTable) / sizeof(cmdTable[0])) {
        String payload = cmdTable[i].cmd == 0 ? FPSTR(STR_OFF) : FPSTR(STR_ON);

        JsonDocument doc;
        doc[FPSTR(STR_ID)] = id;
        if (cmdTable[i].channel > 0) {
            doc[FPSTR(STR_GROUP)] = cmdTable[i].group;
            doc[FPSTR(STR_CHANNEL)] = cmdTable[i].channel;
        }
        publish(payload, doc);
    }
}

RcCodec::CodecParams EV1527Codec::defParams = {
    250,        // timebase
    180, 360,   // timebase min / max
    24,         // numSymbols
    24,         // numSymbolsAutoTimebase
    2,          // numTableSymbols
    2,          // number of pulses per symbol
    3,          // rx quality factor q, matching windows s-(s/q) <= x <= s+(s/q)
    {1, 31},    // footer
    5,          // tx repeats
    {
        1, 3,   // symbol 0: 0b0
        3, 1,   // symbol 1: 0b1
    }
};

EV1527Codec::EV1527Codec() {
    name = PSTR("EV1527");
    params = &defParams;
}

bool EV1527Codec::encodeSymbols(String path, String payload) {
    // path for EV1527Codec: <ID>/<data>
    (void) payload;

    JsonDocument doc;
    doc[FPSTR(STR_ID)] = getPathSegment(path, 0).toInt();
    doc[FPSTR(STR_DATA)] = getPathSegment(path, 1).toInt();

    return encodeSymbols(doc);
}

bool EV1527Codec::encodeSymbols(JsonDocument &doc) {
    if ( !doc[FPSTR(STR_ID)].is<uint32_t>() || !doc[FPSTR(STR_DATA)].is<uint8_t>() )
        return false;
    encodeBinLSB(doc[FPSTR(STR_ID)], 20);
    encodeBinLSB(doc[FPSTR(STR_DATA)], 4);
    doc[FPSTR(STR_COMMAND)] = String(STR_PRESS);
    sendMqttState(doc);
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

bool EV1527Codec::sendDiscovery(String &name, String &id, String &stateTopic, String &cmdTopic, const bool clear) {
    (void) stateTopic;
    haDisc.createButton(name, id, cmdTopic);
    haDisc.setAvailability(getAvailabilityTopic());
    return haDisc.publish(!clear);
}

void EV1527Codec::getDiscoveryFields(JsonDocument &doc, std::vector<JsonVariant> &fields) {
    fields.push_back(doc[FPSTR(STR_ID)]);
    fields.push_back(doc[FPSTR(STR_DATA)]);
}

void EV1527Codec::onDecodedPulses() {
    uint32_t id;
    uint8_t data;
    decodeSymbols(id, data);

    JsonDocument doc;
    doc[FPSTR(STR_ID)] = id;
    doc[FPSTR(STR_DATA)] = data;
    publish(FPSTR(STR_PRESS), doc);
}


Emylo::Emylo() {
    name = PSTR("emylo");
}

bool Emylo::encodeSymbols(String path, String payload) {
    // path for Emylo: <ID>/<KEY 'A'..'D'>
    (void) payload;
    
    JsonDocument doc;
    doc[FPSTR(STR_ID)] = getPathSegment(path, 0).toInt();
    doc[FPSTR(STR_KEY)] = getPathSegment(path, 1);

    return encodeSymbols(doc);
}

bool Emylo::encodeSymbols(JsonDocument &doc) {
    if ( !doc[FPSTR(STR_ID)].is<uint32_t>() || !doc[FPSTR(STR_KEY)].is<String>() )
        return false;

    String sKey = doc[FPSTR(STR_KEY)];
    sKey.toUpperCase();
    const char key = sKey.charAt(0);
    if ( (key < 'A') || (key > 'D') )
        return false;

    /*
        0b1000 -> button A
        0b0100 -> button B
        0b0010 -> button C
        0b0001 -> button D
    */

    doc[FPSTR(STR_DATA)] = 0b1000 >> (key - 'A');
    return EV1527Codec::encodeSymbols(doc);
}

void Emylo::getDiscoveryFields(JsonDocument &doc, std::vector<JsonVariant> &fields) {
    fields.push_back(doc[FPSTR(STR_ID)]);
    fields.push_back(doc[FPSTR(STR_KEY)]);
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
        doc[FPSTR(STR_ID)] = id;
        doc[FPSTR(STR_KEY)] = String(key);
        publish(FPSTR(STR_PRESS), doc);
    }
}


RcCodec::CodecParams FS20Codec::defParams = {
    200,        // timebase
    150, 250,   // timebase min / max
    58,         // 13 sync + 18 housecode + 9 address + 9 command + 9 checksum + 1 stop, optional + 9 bits extended command
    0,          // numSymbolsAutoTimebase; no automatic timebase calculation, because symbols are different in lengths
    2,          // numTableSymbols
    2,          // 2 pulses per symbol
    5,          // rx quality factor
    {1, 30},    // TODO check values!
    5,          // tx repeats
    {
        2, 2,   // symbol 0: 0b0
        3, 3    // symbol 1: 0b1
    }
};

FS20Codec::FS20Codec() {
    name = PSTR("FS20");
    params = &defParams;
}

void FS20Codec::encodeByte(const uint8_t b) {
    uint16_t val = b << 1;
    // calculate even parity bit
    if (!checkEvenParity(val))
        val |= 0x01;

    encodeBinMSB(val, 9);
}

void FS20Codec::getDiscoveryFields(JsonDocument &doc, std::vector<JsonVariant> &fields) {
    fields.push_back(doc[FPSTR(STR_HOUSE)]);
    fields.push_back(doc[FPSTR(STR_ADDRESS)]);
}

uint16_t FS20Codec::strToCmd(const String &str) {
    if (str == FPSTR(STR_ON))
        return 0x11;
    else
        if (str == FPSTR(STR_OFF))
            return 0x00;
        else
            return strtol(str.c_str(), NULL, 16);
}

bool FS20Codec::encodeSymbols(String path, String payload) {
    const String sHouse = getPathSegment(path, 0);
    const String sAddress = getPathSegment(path, 1);
    if (sHouse.isEmpty() || sAddress.isEmpty())
        return false;

    JsonDocument doc;
    doc[FPSTR(STR_HOUSE)] = sHouse.toInt();
    doc[FPSTR(STR_ADDRESS)] = sAddress.toInt();
    doc[FPSTR(STR_COMMAND)] = payload;

    return encodeSymbols(doc);
}

bool FS20Codec::encodeSymbols(JsonDocument &doc) {
    if ( !doc[FPSTR(STR_HOUSE)].is<int>() || 
         !doc[FPSTR(STR_ADDRESS)].is<int>() ||
         doc[FPSTR(STR_COMMAND)].isNull()
        )
        return false;

    const uint16_t house = doc[FPSTR(STR_HOUSE)];
    const uint8_t address = doc[FPSTR(STR_ADDRESS)];

    uint16_t cmd;
    if (doc[FPSTR(STR_COMMAND)].is<String>())
        cmd = strToCmd(doc[FPSTR(STR_COMMAND)]);
    else
        cmd = doc[FPSTR(STR_COMMAND)];
    
    if (doc[FPSTR(STR_EXT)].is<String>())
        cmd |= strToCmd(doc[FPSTR(STR_EXT)]) << 8;
    else
        cmd |= doc[FPSTR(STR_EXT)].as<uint8_t>() << 8;
    
    encodeBinMSB(0b0000000000001, 13); // sync
    encodeByte(house >> 8);
    encodeByte(house & 0xFF);
    encodeByte(address);
    encodeByte(cmd);
    if (cmd & (1<<5)) // extended command
        encodeByte(cmd >> 8);
    uint8_t csum = ( (house >> 8) + (house & 0xFF) + address + (cmd & 0xFF) + (cmd >> 8) + 6) & 0xFF;
    encodeByte(csum);

    sendMqttState(doc);
    return true;
}

bool FS20Codec::checkEvenParity(const uint16_t value) {
    uint8_t count = 0;
    uint16_t x = value;
    while (x) {
        x &= x - 1;
        count++;
    }
    return (count % 2) == 0;
}

bool FS20Codec::getByte(const uint8_t pos, uint8_t &value) {
    uint16_t tmp = decodeBinMSB(pos, 9);
    if (!checkEvenParity(tmp)) {
        return false;
    }
    value = tmp >> 1; // strip of parity bit
    return true;
}

bool FS20Codec::decodePulses(const uint8_t *pulseBuf, const uint8_t len, const uint8_t numSymbols) {
    if (RcCodec::decodePulses(pulseBuf, len, numSymbols))
        return true;

    return RcCodec::decodePulses(pulseBuf, len, params->numSymbols + 9); // try again with extended frame (with cmd2)
}

bool FS20Codec::checkSymbolBuf() {
    uint16_t head = decodeBinMSB(0, 13);
    if (head != 0b0000000000001)
        return false;

    uint8_t p = 13;
    while (symbolBufLen >= p + 9) {
        uint8_t tmp;
        if (!getByte(p, tmp))
            return false;
        p += 9;
    }
    return true;
}

void FS20Codec::onDecodedPulses() {
    uint8_t hc8;
    uint16_t hc;
    getByte(13, hc8);
    hc = hc8 << 8;

    getByte(22, hc8);
    hc |= hc8;

    uint8_t adr, cmd, cmd2 = 0, csum;
    getByte(31, adr);

    getByte(40, cmd);
    bool ext = (cmd & (1<<5)) != 0;

    getByte(49, csum);

    if (ext) {
        cmd2 = csum;
        if (!getByte(58, csum)) {
            return;
        }
    }

    const uint8_t calcCsum = ((hc & 0xFF) + (hc >> 8) + adr + cmd + cmd2 + 6) & 0xFF;
    if (csum != calcCsum) { // TODO if repeaters are used, checksum may be 1-2 higher
        return;
    }

    JsonDocument doc;    
    String payload;

    switch (cmd) {
        case 0x00: 
            payload = FPSTR(STR_OFF);
            break;
        case 0x10:
        case 0x11:
            payload = FPSTR(STR_ON);
            break;
        default:
            payload = String(cmd | (cmd2 << 8), HEX);
            break;
    }

    doc[FPSTR(STR_HOUSE)] = hc;
    doc[FPSTR(STR_ADDRESS)] = adr;

    publish(payload, doc);
}
