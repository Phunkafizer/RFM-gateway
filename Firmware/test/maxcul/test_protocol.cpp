#include "../../src/applications/maxculprotocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

using namespace MaxCulProtocol;

static void testCrcAndWhitening() {
    const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    assert(crc16(check, sizeof(check)) == 0xAEE7);
    uint8_t zeros[8] {};
    const uint8_t pn9[] = {0xFF, 0xE1, 0x1D, 0x9A, 0xED, 0x85, 0x33, 0x24};
    whiten(zeros, sizeof(zeros));
    assert(!memcmp(zeros, pn9, sizeof(zeros)));
    whiten(zeros, sizeof(zeros));
    for (auto byte : zeros)
        assert(byte == 0);
}

static void testReceivedFrame() {
    const char sample[] = "0CEA04421927651A2A46002BD8";
    uint8_t wire[15];
    assert(fromHex(sample, strlen(sample), wire, sizeof(wire)));
    assert(validFrame(wire, 13));
    assert(crc16(wire, 13) == 0x827B); // Golden CRC from the working RaspyRFM gateway.
    wire[13] = 0x82;
    wire[14] = 0x7B;
    whiten(wire, sizeof(wire));
    assert((wire[0] ^ 0xFF) + 3 == sizeof(wire));
    whiten(wire, sizeof(wire));
    assert(crc16(wire, 13) == (uint16_t(wire[13]) << 8 | wire[14]));
    wire[11] ^= 1;
    assert(crc16(wire, 13) != (uint16_t(wire[13]) << 8 | wire[14]));
}

static void testFrameLimitsAndHexValidation() {
    uint8_t data[MAX_WIRE_SIZE] {};
    char text[MAX_FRAME_SIZE * 2 + 3];
    memset(text, '0', sizeof(text));
    assert(!fromHex("0G", 2, data, sizeof(data)));
    assert(!fromHex("123", 3, data, sizeof(data)));
    assert(!fromHex(text, sizeof(text) - 1, data, MAX_FRAME_SIZE));
    assert(fromHex("aBcD", 4, data, sizeof(data)));
    assert(data[0] == 0xAB && data[1] == 0xCD);
    for (size_t size : {size_t(11), size_t(254), MAX_FRAME_SIZE}) {
        memset(data, 0, sizeof(data));
        data[0] = size - 1;
        assert(validFrame(data, size));
        uint16_t crc = crc16(data, size);
        data[size] = crc >> 8;
        data[size + 1] = crc;
        whiten(data, size + 2);
        assert(size_t(data[0] ^ 0xFF) + 3 == size + 2);
        whiten(data, size + 2);
        assert(validFrame(data, size));
        assert(crc16(data, size) == crc);
        assert(!validFrame(data, size - 1));
    }
    assert(!validFrame(data, 0));
    assert(!validFrame(data, MAX_FRAME_SIZE + 1));
}

static void testAutomaticAck() {
    uint8_t frame[] = {11, 49, 5, 0x40, 0x19, 0x27, 0x65, 0x12, 0x34, 0x56, 0, 42};
    uint8_t ack[12];
    const uint8_t expected[] = {11, 49, 0, 2, 0x12, 0x34, 0x56, 0x19, 0x27, 0x65, 0, 0};
    for (uint8_t type : {uint8_t(0x30), uint8_t(0x40), uint8_t(0x50)}) {
        frame[3] = type;
        assert(makeAck(frame, sizeof(frame), 0x123456, 0, ack));
        assert(!memcmp(ack, expected, sizeof(ack)));
    }
    frame[3] = 0x42;
    assert(!makeAck(frame, sizeof(frame), 0x123456, 0, ack));
    frame[3] = 0x40;
    assert(!makeAck(frame, sizeof(frame), 0, 0, ack));
    assert(!makeAck(frame, sizeof(frame), 0x123457, 0, ack));
    assert(makeAck(frame, sizeof(frame), 0, 0x123456, ack));
    uint8_t extended[13];
    memcpy(extended, frame, sizeof(frame));
    extended[0] = 12;
    extended[12] = 0;
    assert(!makeAck(extended, sizeof(extended), 0, 0x123456, ack));
    frame[7] = frame[8] = frame[9] = 0;
    assert(!makeAck(frame, sizeof(frame), 0x123456, 0x123456, ack));
}

static void testAirtimeBudget() {
    uint32_t shortAck = airtimeUs(12, SHORT_PREAMBLE);
    assert(shortAck > 17000 && shortAck < 18000);
    uint32_t wakeup = airtimeUs(12, LONG_PREAMBLE);
    assert(wakeup >= 1000000 && wakeup < 1050000);
    uint32_t maximum = airtimeUs(MAX_FRAME_SIZE, LONG_PREAMBLE);
    assert(maximum > wakeup && maximum < 1250000);
    assert(uint32_t(LONG_PREAMBLE) * 8000000ULL >= uint32_t(BITRATE) * 1000000ULL);
}

int main() {
    testCrcAndWhitening();
    testReceivedFrame();
    testFrameLimitsAndHexValidation();
    testAutomaticAck();
    testAirtimeBudget();
    puts("MAX CUL protocol: all five test groups passed");
}
