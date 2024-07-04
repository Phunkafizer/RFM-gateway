#pragma once

#include <PubSubClient.h>
#include <ESPAsyncWebServer.h>
#include "rfm.h"

extern PubSubClient mqtt;
extern AsyncWebSocket ws;
extern String baseTopic;
extern Rfm69 *rfm69;