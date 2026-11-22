#pragma once

#include <stddef.h>
#include <stdint.h>

namespace MaxCulProtocol {

constexpr size_t MAX_FRAME_SIZE = 256;
constexpr size_t MAX_WIRE_SIZE = MAX_FRAME_SIZE + 2;
constexpr size_t MAX_COMMAND_SIZE = 2 + MAX_FRAME_SIZE * 2;
constexpr uint16_t BITRATE = 9992;
constexpr uint16_t LONG_PREAMBLE = (BITRATE + 7) / 8;
constexpr uint16_t SHORT_PREAMBLE = 4;

inline int hexDigit(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

inline bool fromHex(const char *text, size_t length, uint8_t *data, size_t capacity) {
    if (length % 2 || length / 2 > capacity)
        return false;
    for (size_t i = 0; i < length; i += 2) {
        int high = hexDigit(text[i]);
        int low = hexDigit(text[i + 1]);
        if (high < 0 || low < 0)
            return false;
        data[i / 2] = (high << 4) | low;
    }
    return true;
}

inline bool validFrame(const uint8_t *frame, size_t size) {
    return size >= 11 && size <= MAX_FRAME_SIZE && frame[0] == size - 1;
}

inline uint16_t crc16(const uint8_t *data, size_t size) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < size; i++) {
        crc ^= uint16_t(data[i]) << 8;
        for (uint8_t bit = 0; bit < 8; bit++)
            crc = (crc << 1) ^ ((crc & 0x8000) ? 0x8005 : 0);
    }
    return crc;
}

inline void whiten(uint8_t *data, size_t size) {
    uint16_t lfsr = 0x1FF;
    for (size_t i = 0; i < size; i++) {
        data[i] ^= uint8_t(lfsr);
        for (uint8_t bit = 0; bit < 8; bit++) {
            if ((lfsr ^ (lfsr >> 5)) & 1)
                lfsr |= 1 << 9;
            lfsr >>= 1;
        }
    }
}

inline uint32_t address(const uint8_t *data) {
    return uint32_t(data[0]) << 16 | uint32_t(data[1]) << 8 | data[2];
}

inline bool makeAck(const uint8_t *frame, size_t size, uint32_t autoAck,
                    uint32_t fakeWall, uint8_t ack[12]) {
    if (!validFrame(frame, size))
        return false;
    uint32_t dst = address(frame + 7);
    bool automatic = autoAck && dst == autoAck &&
        (frame[3] == 0x30 || frame[3] == 0x40 || frame[3] == 0x50);
    bool wall = fakeWall && dst == fakeWall && frame[0] == 11 && frame[3] == 0x40;
    if (!automatic && !wall)
        return false;
    ack[0] = 11;
    ack[1] = frame[1];
    ack[2] = 2;
    ack[3] = 2;
    for (uint8_t i = 0; i < 3; i++) {
        ack[4 + i] = frame[7 + i];
        ack[7 + i] = frame[4 + i];
    }
    ack[10] = ack[11] = 0;
    return true;
}

inline uint32_t airtimeUs(size_t frameSize, uint16_t preamble) {
    return ((uint64_t(preamble) + 4 + frameSize + 2) * 8000000ULL + BITRATE - 1) / BITRATE;
}

} // namespace MaxCulProtocol
