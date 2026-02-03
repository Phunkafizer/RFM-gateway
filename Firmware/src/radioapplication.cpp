#include <radioapplication.h>

RadioApplication::RadioApplication() {
    websrv.addHandler(this);
}

RadioApplication::~RadioApplication() {
    websrv.removeHandler(this);
}

void RadioApplication::publish(String topic, JsonDocument &doc) {
    String jsdata;
    serializeJson(doc, jsdata);
    jsdata.replace("\"", "&quot;");
    String btn = F("<button onclick=\"sendDiscovery(this)\" data-discovery='") + jsdata + F("'>send HA discovery</button>");
    ws.textAll(btn);

    mqtt.beginPublish(topic.c_str(), measureJson(doc), false);
    serializeJson(doc, mqtt);
    mqtt.endPublish();
}