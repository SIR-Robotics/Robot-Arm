// ─── vision.cpp ─────────────────────────────────────────────────────────────
// Implementation for vision.h. Owns its own HUSKYLENS instance and state -
// nothing here is extern'd into globals.h on purpose, to keep this module
// removable without ripple effects on the rest of the codebase.

#include "vision.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <HUSKYLENS.h>
#include <ArduinoJson.h>

// ---------------- USER CONFIG ----------------
static const int  VISION_UART_RX = 16;   // ESP32 RX2 <- HuskyLens TX
static const int  VISION_UART_TX = 17;   // ESP32 TX2 -> HuskyLens RX
static const long VISION_UART_BAUD = 9600;

static const char* VISION_API_ENDPOINT = "http://your-server.example.com/api/tag"; // <-- change me

static const unsigned long VISION_POLL_INTERVAL_MS = 300;
static const bool VISION_RESEND_SAME_TAG_EVERY_POLL = false; // true = resend every detection
// ----------------------------------------------

static HUSKYLENS huskylens;
static bool visionReady = false;
static int lastSentTagID = -1;
static unsigned long lastPollTime = 0;

static bool sendTagID(int tagID) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[Vision] WiFi not connected, skipping send.");
    return false;
  }

  HTTPClient http;
  http.begin(VISION_API_ENDPOINT);
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<128> doc;
  doc["tag_id"] = tagID;
  doc["timestamp_ms"] = millis();

  String payload;
  serializeJson(doc, payload);

  int httpCode = http.POST(payload);
  if (httpCode > 0) {
    Serial.printf("[Vision] Sent tag %d -> HTTP %d\n", tagID, httpCode);
  } else {
    Serial.printf("[Vision] Send failed for tag %d: %s\n", tagID, http.errorToString(httpCode).c_str());
  }

  http.end();
  return httpCode > 0;
}

bool visionInit() {
  Serial2.begin(VISION_UART_BAUD, SERIAL_8N1, VISION_UART_RX, VISION_UART_TX);

  Serial.print("[Vision] Connecting to HuskyLens");
  uint32_t t0 = millis();
  while (!huskylens.begin(Serial2)) {
    Serial.print('.');
    if (millis() - t0 > 5000) {
      Serial.println("\n[Vision] HuskyLens not found - vision disabled, arm continues without it.");
      visionReady = false;
      return false;
    }
    delay(300);
  }
  Serial.println("\n[Vision] HuskyLens connected.");
  visionReady = true;
  return true;
}

void visionPoll() {
  if (!visionReady) return;
  if (millis() - lastPollTime < VISION_POLL_INTERVAL_MS) return;
  lastPollTime = millis();

  if (!huskylens.request()) {
    Serial.println("[Vision] Failed to request data from HuskyLens - check connection.");
    return;
  }
  if (!huskylens.isLearned() || !huskylens.available()) {
    return;
  }

  while (huskylens.available()) {
    HUSKYLENSResult result = huskylens.read();

    if (result.command == COMMAND_RETURN_BLOCK) {
      int tagID = result.ID;
      Serial.printf("[Vision] Detected tag ID: %d\n", tagID);

      if (VISION_RESEND_SAME_TAG_EVERY_POLL || tagID != lastSentTagID) {
        if (sendTagID(tagID)) {
          lastSentTagID = tagID;
        }
      }
    }
  }
}