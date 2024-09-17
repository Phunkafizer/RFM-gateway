#include "rfm.h"

#define FXOSC 32E6
#define FSTEP (FXOSC / (1UL<<19))

/**
 * @brief set hardware-environment for RFM module
 * 
 * @param pinSS GPIO for chipselect
  */
void RfmBase::begin(const uint8_t pinSS) {
    this->pinSS = pinSS;
    digitalWrite(pinSS, HIGH);
    pinMode(pinSS, OUTPUT);
}

uint8_t RfmBase::readReg(const uint8_t reg) {
    digitalWrite(pinSS, LOW);
    uint8_t result;
    SPI.transfer(reg);
    result = SPI.transfer(0);
    digitalWrite(pinSS, HIGH);
    return result;
}

uint16_t RfmBase::readReg16(const uint8_t reg) {
    digitalWrite(pinSS, LOW);
    uint16_t result;
    SPI.transfer(reg);
    result = SPI.transfer(0x00);
    result <<= 8;
    result |= SPI.transfer(0x00);
    digitalWrite(pinSS, HIGH);
    return result;
}

void RfmBase::readFifo(uint8_t *buf, const size_t size) {
    digitalWrite(pinSS, LOW);
    SPI.transfer(0x00); // FIFO reg
    for (size_t i=0; i<size; i++)
        *buf++ = SPI.transfer(0x00);

    digitalWrite(pinSS, HIGH);
}

void RfmBase::writeReg(const uint8_t reg, const uint8_t value) {
    digitalWrite(pinSS, LOW);
    SPI.transfer(0x80 | reg);
    SPI.transfer(value);
    digitalWrite(pinSS, HIGH);
}

void RfmBase::writeReg16(const uint8_t reg, const uint16_t value) {
    uint8_t buf[2];
    buf[0] = value >> 8;
    buf[1] = value & 0xFF;
    writeRegBuf(reg, buf, sizeof(buf));
}

void RfmBase::writeRegBuf(const uint8_t reg, const uint8_t *buf, const size_t size) {
    digitalWrite(pinSS, LOW);

    SPI.transfer(0x80 | reg);
    uint8_t *p = (uint8_t *) buf;
    for (size_t i=0; i<size; i++)
        SPI.transfer(*p++);

    digitalWrite(pinSS, HIGH);
}

void RfmBase::setReg(const uint8_t reg, const uint8_t set, const uint8_t clear) {
    uint8_t val = readReg(reg);
    val &= ~clear;
    val |= set;
    writeReg(reg, val);
}

/**
 * @brief set hardware-environment for RFM69 module
 * 
 * @param pinSS GPIO for chipselect
 * @param isHighPower true if H version is used (RFM69H(W), RFM69HC(W))
 */
void Rfm69::begin(const uint8_t pinSS, const bool isHighPower) {
    RfmBase::begin(pinSS);
    highPower = isHighPower;
    
    setMode(MODE_SLEEP);

    const uint16_t deviation = 9900 / (32E06 / (1UL<<19));

    const Rfm69Config cfg[] = {
        {RegOpMode, MODE_STDBY<<2}, // standby
        {RegFdevMsb,        deviation >> 8},
        {RegFdevLsb,        deviation & 0xFF},
        {RegFrfMsb,         0xe4},
        {RegFrfMid,         0xc0},
        {RegFrfLsb,         0x00},
        {RegDataModul,      0}, // Packet mode, FSK
        {RegPreambleMsb,    0},
        {RegPreambleLsb,    10},
        {RegPacketConfig1,  0<<5}, // fixed length, no data whitening, crc off
        {RegPacketConfig2,  0},
        {RegFifoThresh,     0x8f},
        {RegRssiThresh,     220}, // *-0.5dBm
        {RegLna,            0x88},
        {RegRxBw,           2<<5 | RXBWFSK_125KHZ},
        {RegAfcBw,          4<<5 | RXBWFSK_250KHZ},
        {RegAfcFei,         0<<3 | 0<<2}, // AfcAutoclearOn, AfcAutoOn
        {RegAfcCtrl,        0<<5}, // Afc Low Beta On
        {RegTestDagc,       0x30}, // use 0x30 for low beta on = 0
        {RegTestAfc,        0}, //x 488 Hz offset
        {RegTestLna,        0x2D} // high sensitive mode
    };

    writeConfig(cfg, sizeof(cfg)/sizeof(cfg[0]));
    setMode(MODE_FS);
}

void Rfm69::writeConfig(const Rfm69Config cfg[], const uint8_t num) {
    for (uint8_t i=0; i<num; i++)
        writeReg(cfg[i].reg, cfg[i].val);
}

void Rfm69::loop() {
    if (mode > MODE_FS) {
        uint8_t f2 = readReg(RegIrqFlags2);
        switch (mode) {
        case MODE_TX:
            if ( (f2 & (1<<3)) != 0 ) // PacketSent
                setMode(MODE_FS);
            break;

        case MODE_RX: {
            if (!rssiflag) {
                auto iq1 = readReg(RegIrqFlags1);
                if (iq1 & (1<<0)) { // Sync address match
                    rssi = -readReg(RegRssiValue) / 2;
                    writeReg(RegAfcFei, 1<<5); //Fei start
                    rssiflag = true;
                }
            }

            //if ( (f2 & 1<<2) != 0 ) // PayloadReady
              //  setMode(FS);
            break;
        }
        default:
            break;
        }
    }
}

Rfm69::FifoLevel Rfm69::getFifoLevel() {
    uint8_t iq2 = readReg(RegIrqFlags2);

    if ( (iq2 & 0xE0) == 0 )
        return FIFO_EMPTY;

    if ( (iq2 & (1<<7)) != 0 )
        return FIFO_FULL;

    if ( (iq2 & (1<<5)) != 0 )
        return FIFO_THRESH;

    return FIFO_NOTEMPTY;
}

void Rfm69::setMode(const Mode mode) {
    if (highPower) {
        if ((mode == MODE_TX) && (txPwr >= 17)) {
            writeReg(RegOcp, 0x0F); // disable OCP
            writeReg(RegTestPa1, 0x5d); // +20 dBm mode
            writeReg(RegTestPa2, 0x7c); // +20 dBm mode
        }
        else {
            writeReg(RegOcp, 0x1a); // enable OCP 95 mA
            writeReg(RegTestPa1, 0x55); // normal mode
            writeReg(RegTestPa2, 0x70); // normal mode
        }
    }
    writeReg(RegOpMode, mode << 2);
    this->mode = mode;

    while (true) {
        auto if1 = readReg(RegIrqFlags1);
        if ((if1 & 1<<7) != 0)
            break;
    }
}

void Rfm69::stop() {
    setMode(MODE_STDBY);
}

void Rfm69::setSync(const uint8_t *sync, const int syncsize) {
    uint8_t sc = readReg(RegSyncConfig) & ~(1<<7 | 7<<3);
    if (syncsize > 0) {
        sc |= 1<<7 | (syncsize - 1) << 3;
        writeRegBuf(RegSyncValue1, sync, syncsize);
    }
    writeReg(RegSyncConfig, sc);
}

void Rfm69::setAesKey(const uint8_t aeskey[16]) {
    writeRegBuf(RegAesKey1, aeskey, 16);
}

void Rfm69::enableAes(const bool enable) {
    setReg(RegPacketConfig2, enable ? (1<<0) : 0, 1<<0);
}

void Rfm69::setFreq(const uint32_t freq_hz) {
    uint32_t fword = (freq_hz / FSTEP) + f_corr;
    writeReg16(RegFrfMsb, fword >> 8);
    writeReg(RegFrfLsb, fword);
}

void Rfm69::setFCorr(const int16_t fcorr) {
    this->f_corr = fcorr;
}

void Rfm69::setBitrate(const uint16_t bit_s) {
    uint16_t br_word = 32E06 / bit_s;
    writeReg16(RegBitrateMsb, br_word);
}

/**
 * @brief set output power
 * 
 * @param power -18..+13 dBm for RFM69xx, -2..+20 dBm for RFM69Hxx
 */
void Rfm69::setTxPower(const int8_t power) {
    txPwr = power;
    uint8_t pwrreg = 0x00;
    if (highPower) {
        if (power < 13)
            pwrreg = 0x40 | (power + 18); // <13 dBm: PA1
        else if (power < 17)
            pwrreg = 0x60 | (power + 14); // <17 dBm: PA1 + PA2
        else
            pwrreg = 0x60 | (power + 11); // >=17dBm: PA1 + PA2 (& high power registers)
    }
    else
        pwrreg = 0x80 | (power + 18); // PA0

    writeReg(RegPaLevel, pwrreg);
}

bool Rfm69::isIdle() {
    return (mode <= MODE_FS);
}

bool Rfm69::payloadReady() {
    auto iq2 = readReg(RegIrqFlags2);
    return ((iq2 & 1<<2) != 0);
}

void Rfm69::send(const uint8_t *data, const int size, const bool varSize) {
    setMode(MODE_FS);
    while (readReg(RegIrqFlags2) != 0) {
        volatile uint8_t f = readReg(RegFifo);
    }
    if (varSize) { //variable length
        setReg(RegPacketConfig1, 1<<7, 1<<7);  // set PacketFormat
        writeReg(RegFifo, (uint8_t) size);
    }
    else { //fixed length
        setReg(RegPacketConfig1, 0<<7, 1<<7); // unset PacketFormat
        writeReg(RegPayloadLength, size);
    }

    writeRegBuf(RegFifo, data, size);
    setMode(MODE_TX);
}

void Rfm69::writeFifo(const uint8_t *buf, uint8_t size) {
    writeRegBuf(RegFifo, buf, size);
}

void Rfm69::txTest(const uint32_t freq_hz, const int16_t f_corr, const int8_t pwr, const uint16_t baud) {
    setMode(MODE_FS);
    writeReg16(RegPreambleMsb, 15000);
    this->f_corr = f_corr;
    setFreq(freq_hz);
    setTxPower(pwr);
    setBitrate(baud);
    writeReg(RegDataModul, 1<<3); // Packet mode, OOK

    uint8_t data[] = {0x55};
    send(data, sizeof(data), false);
}

/**
 * @brief set RFM in receive mode
 * 
 * @param size -1: variable packet len
 */
void Rfm69::startReceive(const int size) {
    setMode(MODE_FS);
    rxSize = size;
    rssiflag = false;

    if (size == -1) {
        // variable length, packet engine will read 1st byte of payload as len
        setReg(RegPacketConfig1, 1<<7, 1<<7);
        writeReg(RegPayloadLength, 255);
    }
    else {
        // fixed or unlimieted length (size == 0)
        setReg(RegPacketConfig1, 0<<7, 1<<7);
        writeReg(RegPayloadLength, size);
    }

    setMode(MODE_RX);
}

uint8_t Rfm69::getPayload(uint8_t *buf) {
    uint16_t fei = readReg16(RegFeiMsb);
    uint8_t result;

    if (rxSize == -1) {
        // variable packet size
        result = readReg(RegFifo);
    }
    else
        result = rxSize;

    readFifo(buf, result);
    return result;
}

uint8_t Rfm69::getPayload(uint8_t *buf, const uint8_t maxlen) {
    uint8_t result = 0;

    auto iq2 = readReg(RegIrqFlags2);
    while ( (iq2 & (1<<6)) && (result < maxlen) )  { // FIFO not empty 
        buf[result++] = readReg(RegFifo);
        iq2 = readReg(RegIrqFlags2);
    }
    return result;
}

int8_t Rfm69::getRssi() {
    return rssi;
}
