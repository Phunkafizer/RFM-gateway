#include <radioapplication.h>

RadioApplication::RadioApplication() {
    websrv.addHandler(this);
}

RadioApplication::~RadioApplication() {
    websrv.removeHandler(this);
}

bool RadioApplication::onMqttMessage(String topic, String payload) {
    (void) topic;
    (void) payload;
    return false;
}

void RadioApplication::setFreq(uint32_t freq) {
    this->freq = freq;
    if (rfm69)
        rfm69->setFreq(freq);
}

void RadioApplication::publish(String topic, JsonDocument &doc) {
    String jsdata;
    serializeJson(doc, jsdata);
    ws.textAll(jsdata);
    jsdata.replace("\"", "&quot;");
    String btn = F("<button onclick=\"sendDiscovery(this)\" data-discovery='") + jsdata + F("'>send HA discovery</button></br>");
    ws.textAll(btn);

    if (!topic.isEmpty()) {
        mqtt.beginPublish(topic.c_str(), measureJson(doc), false);
        serializeJson(doc, mqtt);
        mqtt.endPublish();
    }
}