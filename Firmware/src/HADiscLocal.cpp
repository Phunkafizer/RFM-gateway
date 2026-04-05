#include "HADiscLocal.h"
#include <ESP8266WiFi.h>
#include "main.h"

RfmHADiscovery haDisc;

const char *DEVNAME PROGMEM = "RFM-Gateway";
const char *MANUFACTURER PROGMEM = "Seegel Systeme";

RfmHADiscovery::RfmHADiscovery() {
    devName = FPSTR(DEVNAME);
    manufacturer = MANUFACTURER;
}

void RfmHADiscovery::begin() {
    String shortMac = WiFi.macAddress();
    shortMac.remove(0, 9);
    int idx;
    while ( (idx = shortMac.indexOf(':')) >= 0)
        shortMac.remove(idx, 1);
    devPrefix = F("rfmgw_");
    devPrefix += shortMac;
}

bool RfmHADiscovery::publish(const bool avail) {
    if (mqtt.connected()) {
        if (!avail)
            return (mqtt.publish(topic.c_str(), "", true) != 0);

        mqtt.beginPublish(topic.c_str(), measureJson(doc), true);
        serializeJson(doc, mqtt);
        return (mqtt.endPublish() != 0);
    }
    return false;
}