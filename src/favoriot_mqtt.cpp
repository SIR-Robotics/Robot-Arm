#include "favoriot_mqtt.h"

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>

#include "arm.h"

const char* MQTT_HOST = "mqtt.favoriot.com";
const uint16_t MQTT_PORT = 1883;

WiFiClient favoriotWifiClient;
PubSubClient favoriotMqtt(favoriotWifiClient);

String rpcTopic() {
    return String(DEVICE_ACCESS_TOKEN) + "/v2/rpc";
}

String streamTopic() {
    return String(DEVICE_ACCESS_TOKEN) + "/v2/streams";
}

long readJsonLong(const String& message, const char* key) {
    int keyPos = message.indexOf(key);
    if (keyPos < 0) return -1;

    int colon = message.indexOf(':', keyPos);
    if (colon < 0) return -1;

    return message.substring(colon + 1).toInt();
}

void publishFavoriot(const char* key, const char* value) {
    if (!favoriotMqtt.connected()) return;

    char payload[180];
    snprintf(payload, sizeof(payload),
             "{\"device_developer_id\":\"%s\",\"data\":{\"%s\":\"%s\"}}",
             DEVICE_DEVELOPER_ID, key, value);
    favoriotMqtt.publish(streamTopic().c_str(), payload);
}

void publishArmResult(const char* color, long id) {
    char result[24];
    snprintf(result, sizeof(result), "%s_ok", color);
    publishFavoriot("arm", result);

    if (id < 0) return;
    char payload[80];
    snprintf(payload, sizeof(payload), "{\"to\":\"mecanum\",\"id\":%ld,\"arm\":\"%s\"}",
             id, result);
    favoriotMqtt.publish(rpcTopic().c_str(), payload);
}

void runColor(const char* color, long id) {
    if (strcmp(color, "red") == 0) runRed();
    else if (strcmp(color, "blue") == 0) runBlue();
    else if (strcmp(color, "yellow") == 0) runYellow();
    else return;

    publishArmResult(color, id);
}

void handleFavoriotMessage(char* topic, byte* payload, unsigned int length) {
    String message((const char*)payload, length);
    message.toLowerCase();
    Serial.printf("[Favoriot] %s: %s\n", topic, message.c_str());

    if (message.indexOf("\"to\":\"mecanum\"") >= 0) return;
    if (message.indexOf("\"to\":\"arm\"") < 0) return;

    long id = readJsonLong(message, "\"id\"");
    if (message.indexOf("red") >= 0) runColor("red", id);
    else if (message.indexOf("blue") >= 0) runColor("blue", id);
    else if (message.indexOf("yellow") >= 0) runColor("yellow", id);
}

void connectFavoriot() {
    static unsigned long lastAttemptMs = 0;

    if (WiFi.status() != WL_CONNECTED || favoriotMqtt.connected()) return;
    if (millis() - lastAttemptMs < 3000) return;
    lastAttemptMs = millis();

    String clientId = "arm-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    Serial.println("[Favoriot] Connecting MQTT...");
    if (favoriotMqtt.connect(clientId.c_str(), DEVICE_ACCESS_TOKEN, DEVICE_ACCESS_TOKEN)) {
        favoriotMqtt.subscribe(rpcTopic().c_str());
        publishFavoriot("arm", "online");
        Serial.println("[Favoriot] MQTT connected");
    } else {
        Serial.printf("[Favoriot] MQTT failed: %d\n", favoriotMqtt.state());
    }
}

void setupFavoriotMqtt() {
    favoriotMqtt.setServer(MQTT_HOST, MQTT_PORT);
    favoriotMqtt.setCallback(handleFavoriotMessage);
    connectFavoriot();
}

void loopFavoriotMqtt() {
    connectFavoriot();
    favoriotMqtt.loop();
}
