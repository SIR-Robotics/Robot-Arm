#include "favoriot.h"
#include "vision.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>

static WiFiClient network;
static PubSubClient mqtt(network);
static uint32_t lastConnectMs = 0;
static const char* ONLINE_PRESENCE = "{\"to\":\"frontend\",\"type\":\"presence\",\"online\":true}";
static const char* OFFLINE_PRESENCE = "{\"to\":\"frontend\",\"type\":\"presence\",\"online\":false}";

static String streamTopic() {
    return String(DEVICE_ACCESS_TOKEN) + "/v2/streams";
}

static String rpcTopic() {
    return String(DEVICE_ACCESS_TOKEN) + "/v2/rpc";
}

static void handleMessage(char*, byte* payload, unsigned int length) {
    StaticJsonDocument<128> message;
    if (deserializeJson(message, payload, length)) return;
    if (strcmp(message["to"] | "", "arm") != 0) return;

    if (strcmp(message["command"] | "", "presence") == 0) {
        mqtt.publish(rpcTopic().c_str(), ONLINE_PRESENCE, false);
        return;
    }
    if (strcmp(message["command"] | "", "check_tagging") != 0) return;

    uint32_t id = message["id"] | 0;
    if (id > 0) visionRequestTagging(id);
}

static void connectMqtt() {
    if (WiFi.status() != WL_CONNECTED || mqtt.connected()) return;
    if (lastConnectMs && millis() - lastConnectMs < 3000) return;
    lastConnectMs = millis();
    String clientId = "arm-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    String topic = rpcTopic();
    if (mqtt.connect(clientId.c_str(), DEVICE_ACCESS_TOKEN, DEVICE_ACCESS_TOKEN,
                     topic.c_str(), 0, false, OFFLINE_PRESENCE)) {
        mqtt.subscribe(topic.c_str());
        mqtt.publish(topic.c_str(), ONLINE_PRESENCE, false);
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
    mqtt.setCallback(handleMessage);
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

void favoriotTaggingResult(uint32_t id, const char* status, int tag, const char* color) {
    if (!mqtt.connected()) return;
    char payload[160];
    if (tag >= 0 && color) {
        snprintf(payload, sizeof(payload),
                 "{\"to\":\"mecanum\",\"type\":\"tagging_result\","
                 "\"id\":%lu,\"status\":\"%s\",\"tag\":%d,\"color\":\"%s\"}",
                 (unsigned long)id, status, tag, color);
    } else {
        snprintf(payload, sizeof(payload),
                 "{\"to\":\"mecanum\",\"type\":\"tagging_result\","
                 "\"id\":%lu,\"status\":\"%s\"}",
                 (unsigned long)id, status);
    }
    mqtt.publish(rpcTopic().c_str(), payload);
}
