#pragma once

#include <ESP8266WiFi.h>
#include "../radioapplication.h"
#include "maxculprotocol.h"

class MaxCul: public RadioApplication {
private:
    static constexpr uint16_t TCP_PORT = 2323;
    static constexpr size_t MAX_CLIENTS = 4;
    static constexpr size_t TX_QUEUE_SIZE = 4;
    static constexpr size_t MAX_OUTPUT_SIZE = 2048;
    static constexpr uint32_t MAX_CREDIT_US = 9000000;

    struct Client {
        WiFiClient socket;
        uint32_t id = 0;
        uint8_t flags = 0;
        char input[MaxCulProtocol::MAX_COMMAND_SIZE + 1] {};
        size_t inputSize = 0;
        bool discarding = false;
        String output;
    };

    struct Transmission {
        uint8_t frame[MaxCulProtocol::MAX_FRAME_SIZE] {};
        uint16_t size = 0;
        uint32_t clientId = 0;
        bool longPreamble = true;
        bool echo = false;
    };

    enum State { RECEIVING, TX_DELAY, TRANSMITTING } state = RECEIVING;
    WiFiServer server;
    Client clients[MAX_CLIENTS];
    uint32_t nextClientId = 0;
    uint32_t autoAckAddress = 0;
    uint32_t fakeWallAddress = 0;
    Transmission txQueue[TX_QUEUE_SIZE];
    size_t queueHead = 0;
    size_t queueSize = 0;
    Transmission tx;
    uint8_t wire[MaxCulProtocol::MAX_WIRE_SIZE] {};
    size_t txSize = 0;
    size_t txPosition = 0;
    uint32_t txAt = 0;
    uint32_t txStarted = 0;
    bool txDraining = false;
    uint32_t txDrainedAt = 0;
    uint32_t lastTxEnd = 0;
    uint8_t rx[MaxCulProtocol::MAX_WIRE_SIZE] {};
    size_t rxSize = 0;
    size_t rxExpected = 0;
    uint32_t rxStarted = 0;
    int8_t rxRssi = 0;
    uint32_t creditUs = MAX_CREDIT_US;
    uint32_t creditUpdated = 0;
    bool txLimitEnabled = true;

    void log(const String &message);
    void closeClient(Client &client);
    void sendLine(Client &client, const String &line);
    void sendError(uint32_t clientId, const String &message);
    void pollClients();
    void handleCommand(Client &client);
    void reportFrame(const uint8_t *frame, size_t size, int8_t rssi, bool echo = false, int32_t feiHz = 0);
    void receive();
    void prepareSend(uint32_t delayMs);
    void startSend();
    void pollSend();
    void updateCredit();

public:
    explicit MaxCul(const JsonObject &conf);
    ~MaxCul() override;
    void loop() override;
    void restartReceive() override;
    void announceMdns(MDNSResponder::hMDNSService hHttpService) override;
    bool sendDiscovery(JsonDocument &doc) override;
};
