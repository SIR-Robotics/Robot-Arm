#include "favoriot.h"

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>

static WiFiClient network;
static PubSubClient mqtt(network);
static uint32_t lastConnectMs = 0;

static String streamTopic() {
    return String(DEVICE_ACCESS_TOKEN) + "/v2/streams";
}

static void connectMqtt() {
    if (WiFi.status() != WL_CONNECTED || mqtt.connected()) return;
    if (lastConnectMs && millis() - lastConnectMs < 3000) return;
    lastConnectMs = millis();
    String clientId = "arm-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (mqtt.connect(clientId.c_str(), DEVICE_ACCESS_TOKEN, DEVICE_ACCESS_TOKEN)) {
        Serial.println("[Favoriot] connected");
    } else {
        Serial.printf("[Favoriot] connection failed: %d\n", mqtt.state());
    }
}

static void publish(const char* key, const char* value) {
    if (!mqtt.connected()) return;
    char payload[256];
    snprintf(payload, sizeof(payload),
             "{\"device_developer_id\":\"%s\",\"data\":{\"%s\":\"%s\"}}",
             DEVICE_DEVELOPER_ID, key, value);
    mqtt.publish(streamTopic().c_str(), payload);
}

void favoriotSetup() {
    mqtt.setServer("mqtt.favoriot.com", 1883);
}

void favoriotLoop() {
    connectMqtt();
    mqtt.loop();
}

void favoriotAction(const char* action) {
    char message[160];
    snprintf(message, sizeof(message), "arm: %s", action);
    publish("action", message);
}
