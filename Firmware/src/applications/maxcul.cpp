#include "maxcul.h"
#include <string.h>

using namespace MaxCulProtocol;

MaxCul::MaxCul(const JsonObject &conf): server(TCP_PORT) {
    (void) conf;
    freq = 868300000UL;
    creditUpdated = millis();
    const Rfm69::Rfm69Config cfg[] = {
        {Rfm69::RegDataModul, 0},
        {Rfm69::RegFdevMsb, 0x01},
        {Rfm69::RegFdevLsb, 0x38},
        {Rfm69::RegRxBw, Rfm69::DCC_4 | Rfm69::RXBWFSK_166_6KHZ},
        {Rfm69::RegSyncConfig, 0},
        {Rfm69::RegPacketConfig1, 0},
        {Rfm69::RegPacketConfig2, 0},
        {Rfm69::RegFifoThresh, 0x8F},
    };
    rfm69->stop();
    rfm69->writeConfig(cfg, sizeof(cfg) / sizeof(cfg[0]));
    rfm69->setBitrate(BITRATE);
    rfm69->setRxThresh(-105);
    const uint8_t sync[] = {0xC6, 0x26, 0xC6, 0x26};
    rfm69->setSync(sync, sizeof(sync));
    restartReceive();
    server.begin();
    server.setNoDelay(true);
    if (server.status())
        log(F("TCP server listening on port 2323"));
    else
        log(F("ERROR: cannot open TCP port 2323"));
}

MaxCul::~MaxCul() {
    server.stop();
    for (auto &client : clients)
        closeClient(client);
    rfm69->stop();
    rfm69->clearFifo();
}

void MaxCul::log(const String &message) {
    String line = F("maxcul: ");
    line += message;
    Serial.println(line);
    ws.textAll(line);
}

void MaxCul::closeClient(Client &client) {
    client.socket.stop();
    client.id = 0;
    client.flags = 0;
    client.inputSize = 0;
    client.discarding = false;
    client.output = "";
}

void MaxCul::sendLine(Client &client, const String &line) {
    if (!client.id)
        return;
    size_t size = client.output.length() + line.length() + 2;
    if (size > MAX_OUTPUT_SIZE || !client.output.reserve(size)) {
        log(F("ERROR: TCP client output buffer full; disconnecting"));
        closeClient(client);
        return;
    }
    client.output += line;
    client.output += F("\r\n");
}

void MaxCul::sendError(uint32_t clientId, const String &message) {
    log(message);
    for (auto &client : clients)
        if (client.id && (!clientId || client.id == clientId))
            sendLine(client, message);
}

void MaxCul::pollClients() {
    if (server.hasClient()) {
        WiFiClient incoming = server.accept();
        bool accepted = false;
        for (auto &client : clients) {
            if (client.id && !client.socket.connected())
                closeClient(client);
            if (!client.id) {
                client.socket = incoming;
                client.id = ++nextClientId;
                if (!client.id)
                    client.id = ++nextClientId;
                accepted = true;
                log(F("TCP client connected"));
                break;
            }
        }
        if (!accepted) {
            log(F("ERROR: TCP client limit reached"));
            incoming.stop();
        }
    }
    for (auto &client : clients) {
        if (!client.id)
            continue;
        if (!client.socket.connected() && !client.socket.available()) {
            log(F("TCP client disconnected"));
            closeClient(client);
            continue;
        }
        for (size_t count = 0; count < 128 && client.id && client.socket.available(); count++) {
            int value = client.socket.read();
            if (value < 0)
                break;
            char c = char(value);
            if (c == '\r' || c == '\n') {
                if (!client.discarding && client.inputSize) {
                    client.input[client.inputSize] = '\0';
                    handleCommand(client);
                }
                client.inputSize = 0;
                client.discarding = false;
            } else if (!client.discarding) {
                if (c < 32 || c > 126 || client.inputSize == MAX_COMMAND_SIZE) {
                    client.discarding = true;
                    client.inputSize = 0;
                    sendError(client.id, F("ZERR invalid or oversized command"));
                } else {
                    client.input[client.inputSize++] = c;
                }
            }
        }
        int available = client.socket.availableForWrite();
        if (client.id && available > 0 && client.output.length()) {
            size_t size = client.output.length();
            if (size > size_t(available))
                size = available;
            size_t written = client.socket.write(
                reinterpret_cast<const uint8_t *>(client.output.c_str()), size);
            if (!written) {
                log(F("ERROR: TCP write failed"));
                closeClient(client);
            } else {
                client.output.remove(0, written);
            }
        }
    }
}

void MaxCul::handleCommand(Client &client) {
    const char *command = client.input;
    size_t size = client.inputSize;
    if (!strcmp(command, "V")) {
        sendLine(client, F("V 1.67 CUL868 (RFM Gateway maxcul)"));
    } else if (!strcmp(command, "X")) {
        char report[20];
        snprintf(report, sizeof(report), "%02X  %lu", client.flags,
                 static_cast<unsigned long>(creditUs / 10000));
        sendLine(client, String(report));
    } else if (size == 3 && command[0] == 'X') {
        uint8_t flags;
        if (!fromHex(command + 1, 2, &flags, 1))
            sendError(client.id, F("ZERR invalid reporting flags"));
        else
            client.flags = flags;
    } else if (size >= 2 && command[0] == 'Z' && (command[1] == 'a' || command[1] == 'w')) {
        uint8_t addr[3];
        if (size != 8 || !fromHex(command + 2, 6, addr, sizeof(addr))) {
            sendError(client.id, F("ZERR invalid ACK address"));
            return;
        }
        if (command[1] == 'a')
            autoAckAddress = address(addr);
        else
            fakeWallAddress = address(addr);
        log(String(F("Configured ")) + command);
    } else if (size >= 2 && command[0] == 'Z' && (command[1] == 's' || command[1] == 'f')) {
        if (queueSize == TX_QUEUE_SIZE) {
            sendError(client.id, F("ZERR TX queue full"));
            return;
        }
        Transmission &request = txQueue[(queueHead + queueSize) % TX_QUEUE_SIZE];
        if (!fromHex(command + 2, size - 2, request.frame, sizeof(request.frame))) {
            sendError(client.id, F("ZERR invalid TX hex"));
            return;
        }
        request.size = (size - 2) / 2;
        if (!validFrame(request.frame, request.size)) {
            sendError(client.id, F("LENERR"));
            return;
        }
        request.longPreamble = command[1] == 's';
        request.echo = false;
        request.clientId = client.id;
        queueSize++;
        log(String(F("TX queued: ")) + command);
    } else if (!strcmp(command, "Zr") || !strcmp(command, "T01")) {
        // Reception is already active; FHT mode is not implemented.
    } else {
        sendError(client.id, F("ZERR unsupported command"));
    }
}

void MaxCul::reportFrame(const uint8_t *frame, size_t size, int8_t rssi, bool echo, int32_t feiHz) {
    static const char hex[] = "0123456789ABCDEF";
    char line[MAX_FRAME_SIZE * 2 + 1];
    for (size_t i = 0; i < size; i++) {
        line[i * 2] = hex[frame[i] >> 4];
        line[i * 2 + 1] = hex[frame[i] & 0x0F];
    }
    line[size * 2] = '\0';
    String text = String('Z') + line;
    String info = String(echo ? F("ACK: ") : F("RX: ")) + text;
    if (!echo)
        info += String(F(" (")) + rssi + F(" dBm, FEI ") + feiHz + F(" Hz)");
    log(info);
    uint8_t encodedRssi = echo ? 0 : uint8_t(int(rssi) * 2 + 148);
    char suffix[3];
    snprintf(suffix, sizeof(suffix), "%02X", encodedRssi);
    for (auto &client : clients)
        if (client.id)
            sendLine(client, text + ((client.flags & 0x20) ? String(suffix) : String()));
}

void MaxCul::announceMdns(MDNSResponder::hMDNSService hHttpService) {
    (void) hHttpService;
    // Announce the CUL style TCP interface for the Home Assistant integration.
    MDNS.addService(hostname.c_str(), "maxcul", "tcp", TCP_PORT);
}

void MaxCul::restartReceive() {
    rfm69->stop();
    rfm69->clearFifo();
    rxSize = rxExpected = 0;
    state = RECEIVING;
    rfm69->setFreq(freq);
    rfm69->startReceive(0);
}

void MaxCul::receive() {
    if (rfm69->readReg(Rfm69::RegIrqFlags2) & 0x10) {
        log(F("ERROR: RX FIFO overrun"));
        restartReceive();
        return;
    }
    if (!rxSize && rfm69->getPayload(rx, 1)) {
        rxSize = 1;
        rxExpected = (rx[0] ^ 0xFF) + 3;
        rxStarted = millis();
        rxRssi = rfm69->getRssi();
        if (rxExpected < 13 || rxExpected > sizeof(rx)) {
            log(F("ERROR: invalid RX length"));
            restartReceive();
            return;
        }
    }
    if (!rxSize)
        return;
    while (rxSize < rxExpected) {
        size_t remaining = rxExpected - rxSize;
        uint8_t count = rfm69->getPayload(rx + rxSize, remaining > 64 ? 64 : remaining);
        if (!count)
            break;
        rxSize += count;
    }
    if (rxSize < rxExpected) {
        if (uint32_t(millis() - rxStarted) >= 500) {
            log(F("ERROR: RX frame timed out"));
            restartReceive();
        }
        return;
    }
    // FEI was captured at sync match (FeiStart in Rfm69::loop); read before leaving RX.
    int32_t feiHz = rfm69->getFeiHz();
    rfm69->stop();
    whiten(rx, rxSize);
    size_t size = rxSize - 2;
    uint16_t receivedCrc = uint16_t(rx[size]) << 8 | rx[size + 1];
    if (!validFrame(rx, size) || crc16(rx, size) != receivedCrc) {
        log(F("ERROR: RX CRC mismatch"));
        restartReceive();
        return;
    }
    reportFrame(rx, size, rxRssi, false, feiHz);
    if (makeAck(rx, size, autoAckAddress, fakeWallAddress, tx.frame)) {
        tx.size = 12;
        tx.longPreamble = false;
        tx.echo = true;
        tx.clientId = 0;
        prepareSend(20);
    } else {
        restartReceive();
    }
}

void MaxCul::prepareSend(uint32_t delayMs) {
    rfm69->stop();
    rfm69->clearFifo();
    rxSize = rxExpected = 0;
    state = TX_DELAY;
    uint32_t now = millis();
    uint32_t elapsed = now - lastTxEnd;
    if (elapsed < 20 && delayMs < 20 - elapsed)
        delayMs = 20 - elapsed;
    txAt = now + delayMs;
}

void MaxCul::startSend() {
    uint16_t preamble = tx.longPreamble ? LONG_PREAMBLE : SHORT_PREAMBLE;
    uint32_t cost = airtimeUs(tx.size, preamble);
    if (creditUs < cost) {
        sendError(tx.clientId, F("LOVF"));
        restartReceive();
        return;
    }
    creditUs -= cost;
    memcpy(wire, tx.frame, tx.size);
    uint16_t crc = crc16(wire, tx.size);
    wire[tx.size] = crc >> 8;
    wire[tx.size + 1] = crc & 0xFF;
    txSize = tx.size + 2;
    whiten(wire, txSize);
    const Rfm69::Rfm69Config cfg[] = {
        {Rfm69::RegPreambleMsb, uint8_t(preamble >> 8)},
        {Rfm69::RegPreambleLsb, uint8_t(preamble)},
    };
    rfm69->writeConfig(cfg, sizeof(cfg) / sizeof(cfg[0]));
    txPosition = rfm69->startSend(wire, txSize);
    txStarted = millis();
    txDraining = false;
    state = TRANSMITTING;
}

void MaxCul::pollSend() {
    if (rfm69->isIdle()) {
        if (txPosition != txSize)
            sendError(tx.clientId, F("ZERR TX ended before FIFO refill"));
        else if (tx.echo)
            reportFrame(tx.frame, tx.size, 0, true);
        lastTxEnd = millis();
        restartReceive();
        return;
    }
    if (uint32_t(millis() - txStarted) >= 3000) {
        sendError(tx.clientId, F("ZERR TX timeout"));
        lastTxEnd = millis();
        restartReceive();
        return;
    }
    if (txSize > 255 && txPosition == txSize && rfm69->getFifoLevel() == Rfm69::FIFO_EMPTY) {
        // Unlimited-length packets have no PacketSent interrupt. Allow the
        // final FIFO byte to leave the shift register before stopping TX.
        if (!txDraining) {
            txDraining = true;
            txDrainedAt = millis();
        } else if (uint32_t(millis() - txDrainedAt) >= 2) {
            rfm69->stop();
        }
        return;
    }
    if (txPosition < txSize && rfm69->getFifoLevel() <= Rfm69::FIFO_NOTEMPTY) {
        // FIFO threshold is 15; at most 15 bytes remain, leaving room for 49.
        size_t count = txSize - txPosition;
        if (count > 48)
            count = 48;
        rfm69->writeFifo(wire + txPosition, count);
        txPosition += count;
    }
}

void MaxCul::updateCredit() {
    uint32_t now = millis();
    uint32_t elapsed = now - creditUpdated;
    creditUpdated = now;
    if (elapsed > MAX_CREDIT_US / 10)
        elapsed = MAX_CREDIT_US / 10;
    creditUs += elapsed * 10; // One percent airtime: 10 us per elapsed ms.
    if (creditUs > MAX_CREDIT_US)
        creditUs = MAX_CREDIT_US;
}

void MaxCul::loop() {
    updateCredit();
    if (state == TRANSMITTING)
        pollSend();
    else if (state == RECEIVING)
        receive();
    pollClients();
    if (state == RECEIVING && !rxSize && queueSize) {
        tx = txQueue[queueHead];
        queueHead = (queueHead + 1) % TX_QUEUE_SIZE;
        queueSize--;
        prepareSend(0);
    }
    if (state == TX_DELAY && int32_t(millis() - txAt) >= 0)
        startSend();
}

bool MaxCul::sendDiscovery(JsonDocument &doc) {
    (void) doc;
    return false;
}
