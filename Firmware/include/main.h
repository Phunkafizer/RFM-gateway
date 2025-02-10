#pragma once

#include <PubSubClient.h>
#include <ESPAsyncWebServer.h>
#include "rfm.h"

#ifdef DEBUG
#define SDBG(x) Serial.print(x)
#define SDBGLN(x) Serial.println(x)
#else
#define SDBG(x) void()
#define SDBGLN(x) void()
#endif

extern PubSubClient mqtt;
extern AsyncWebSocket ws;
extern String baseTopic;
extern Rfm69 *rfm69;
extern AsyncWebServer websrv;