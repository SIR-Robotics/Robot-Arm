#include "favoriot.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>

static WiFiClient network;
static PubSubClient mqtt(network);
static uint32_t lastConnectMs = 0;
static const char* ONLINE_PRESENCE = "{\"to\":\"frontend\",\"type\":\"presence\",\"online\":true}";
static const char* OFFLINE_PRESENCE = "{\"to\":\"frontend\",\"type\":\"presence\",\"online\":false}";
static int pendingSortTag = -1;
static char pendingSortColor[8] = "";
static bool pendingSortComplete = false;

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

static void publishCompletedSort() {
    if (!pendingSortComplete || !mqtt.connected()) return;
    char payload[192];
    snprintf(payload, sizeof(payload),
             "{\"device_developer_id\":\"%s\",\"data\":{"
             "\"event\":\"book_sorted\",\"tag\":%d,\"color\":\"%s\"}}",
             DEVICE_DEVELOPER_ID, pendingSortTag, pendingSortColor);
    if (mqtt.publish(streamTopic().c_str(), payload)) favoriotSortCancelled();
}

void favoriotSetup() {
    mqtt.setServer("mqtt.favoriot.com", 1883);
    mqtt.setCallback(handleMessage);
}

void favoriotLoop() {
    connectMqtt();
    mqtt.loop();
    publishCompletedSort();
}

void favoriotAction(const char* action) {
    char message[160];
    snprintf(message, sizeof(message), "arm: %s", action);
    publish("action", message);
}

void favoriotSortStarted(int tag, const char* color) {
    pendingSortTag = tag;
    snprintf(pendingSortColor, sizeof(pendingSortColor), "%s", color);
    pendingSortComplete = false;
}

void favoriotSortCompleted() {
    if (pendingSortTag < 0) return;
    pendingSortComplete = true;
    publishCompletedSort();
}

void favoriotSortCancelled() {
    pendingSortTag = -1;
    pendingSortColor[0] = '\0';
    pendingSortComplete = false;
}
