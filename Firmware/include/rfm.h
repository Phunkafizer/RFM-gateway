#ifndef __rfm_h
#define __rfm_h
#include <Arduino.h>
#include <SPI.h>

/**
 * @brief baseclass for RFMxx
 * 
 * RFM69: 434 / 868 / 915 MHz
 * RFM95: 868 / 915 MHz, Spreading Factor 6 - 12
 * RFM96: 433 / 470 MHz, Spreading Factor 6 - 12
 * RFM97: 868 / 915 MHz, Spreading Factor 6 - 9
 * RFM98: 433 / 470 MHz, Spreading Factor 6 - 12
 * 
 * 
 */

class RfmBase {
private:
    uint8_t pinSS;
protected:
    uint8_t readReg(const uint8_t reg);
    uint16_t readReg16(const uint8_t reg);
    void readFifo(uint8_t *buf, const size_t size);
    void writeReg(const uint8_t reg, const uint8_t value);
    void writeReg16(const uint8_t reg, const uint16_t value);
    void writeRegBuf(const uint8_t reg, const uint8_t *buf, const size_t size);
    void setReg(const uint8_t reg, const uint8_t set, const uint8_t clear);
public:
    virtual void begin(const uint8_t pinSS);
    virtual ~RfmBase() {}
};

class Rfm69: public RfmBase {
private:
    enum Mode: uint8_t {
        MODE_SLEEP = 0,
        MODE_STDBY = 1,
        MODE_FS = 2,
        MODE_TX = 3,
        MODE_RX = 4
    } mode;
    void setMode(const Mode mode);
    bool highPower;
    int8_t txPwr;
    int rxSize;
    int16_t f_corr;
    bool rssiflag;
    int8_t rssi;
public:
    enum Registers: uint8_t {
        RegFifo = 0x00,
        RegOpMode = 0x01,
        RegDataModul = 0x02,
        RegBitrateMsb = 0x03,
        RegBitrateLsb = 0x04,
        RegFdevMsb = 0x05,
        RegFdevLsb = 0x06,
        RegFrfMsb = 0x07,
        RegFrfMid = 0x08,
        RegFrfLsb = 0x09,
        RegAfcCtrl = 0x0B,
        RegPaLevel = 0x11,
        RegOcp = 0x13,
        RegLna = 0x18,
        RegRxBw = 0x19,
        RegAfcBw = 0x1A,
        RegOokPeak = 0x1B,
        RegOokFix = 0x1D,
        RegAfcFei = 0x1E,
        RegAfcMsb = 0x1F,
        RegAfcLsb = 0x20,
        RegFeiMsb = 0x21,
        RegFeiLsb = 0x22,
        RegRssiConfig = 0x23,
        RegRssiValue = 0x24,
        RegIrqFlags1 = 0x27,
        RegIrqFlags2 = 0x28,
        RegRssiThresh = 0x29,
        RegPreambleMsb = 0x2c,
        RegPreambleLsb = 0x2d,
        RegSyncConfig = 0x2e,
        RegSyncValue1 = 0x2f,
        RegPacketConfig1 = 0x37,
        RegPayloadLength = 0x38,
        RegFifoThresh = 0x3c,
        RegPacketConfig2 = 0x3d,
        RegAesKey1 = 0x3e,
        RegTestLna = 0x58,
        RegTestPa1 = 0x5a,
        RegTestPa2 = 0x5c,
        RegTestDagc = 0x6f,
        RegTestAfc = 0x71
    };
    
    enum RxBandwidthFsk: uint8_t {
        RXBWFSK_500KHZ = 0<<3 | 0<<0,       // 500 kHz
        RXBWFSK_250KHZ = 0<<3 | 1<<0,       // 250 kHz
        RXBWFSK_125KHZ = 0<<3 | 2<<0,       // 125 kHz
        RXBWFSK_62_5KHZ = 0<<3 | 3<<0,      // 62.5 kHz
        RXBWFSK_31_25KHZ = 0<<3 | 4<<0,     // 31.25 kHz
        RXBWFSK_15_625KHZ = 0<<3 | 5<<0,    // 15.625 kHz
        RXBWFSK_7_82KHZ = 0<<3 | 6<<0,      // 7.8125 kHz
        RXBWFSK_3_9KHZ = 0<<3 | 7<<0,       // 3.90625 kHz
        RXBWFSK_400KHZ = 1<<3 | 0<<0,       // 400 kHz
        RXBWFSK_200KHZ = 1<<3 | 1<<0,       // 200 kHz
        RXBWFSK_100KHZ = 1<<3 | 2<<0,       // 100 kHz
        RXBWFSK_50KHZ = 1<<3 | 3<<0,        // 50 kHz
        RXBWFSK_25KHZ = 1<<3 | 4<<0,        // 25 kHz
        RXBWFSK_12_5KHZ = 1<<3 | 5<<0,      // 12.5 kHz
        RXBWFSK_6_25KHZ = 1<<3 | 6<<0,      // 6.25 kHz
        RXBWFSK_3_125KHZ = 1<<3 | 7<<0,     // 3.125 kHz
        RXBWFSK_333_3KHZ = 2<<3 | 0<<0,     // 333.33 kHz
        RXBWFSK_166_6KHZ = 2<<3 | 1<<0,     // 166.66 kHz
        RXBWFSK_83_3KHZ = 2<<3 | 2<<0,      // 83.33 kHz
        RXBWFSK_41_6KHZ = 2<<3 | 3<<0,      // 41.66 kHz
        RXBWFSK_20_8KHZ = 2<<3 | 4<<0,      // 20.833 kHz
        RXBWFSK_10_4KHZ = 2<<3 | 5<<0,      // 10.416 kHz
        RXBWFSK_5_2KHZ = 2<<3 | 6<<0,       // 5.208 kHz
        RXBWFSK_2_6KHZ = 2<<3 | 7<<0,       // 2.604 kHz

        RXBWASK_250KHZ = 0<<3 | 0<<0       // 250 kHz

    };

    enum FifoLevel {
        FIFO_EMPTY,
        FIFO_NOTEMPTY,
        FIFO_THRESH,
        FIFO_FULL
    };

    struct Rfm69Config {
        Registers reg;
        uint8_t val;
    };

    void begin(const uint8_t pinSS, const bool isHighPower);
    void loop();
    void stop();
    void setSync(const uint8_t *sync, const int syncsize);
    void setAesKey(const uint8_t aeskey[16]);
    void enableAes(const bool enable);
    void setTxPower(const int8_t power);
    void setFreq(const uint32_t freq_hz);
    void setBitrate(const uint16_t bit_s);
    void setFCorr(const int16_t fcorr);
    void writeConfig(const Rfm69Config cfg[], const uint8_t num);

    void send(const uint8_t *data, const int size, const bool varSize = true);
    void txTest(const uint32_t freq_hz, const int16_t f_corr, const int8_t pwr, const uint16_t baud);
    
    bool isIdle();
    void startReceive(const int size = -1);
    bool payloadReady();
    uint8_t getPayload(uint8_t *buf);
    uint8_t getPayload(uint8_t *buf, const uint8_t maxlen);
    int8_t getRssi();
    FifoLevel getFifoLevel();
    void writeFifo(const uint8_t *buf, uint8_t len);
};

#endif