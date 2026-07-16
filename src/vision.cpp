// ─── vision.cpp ─────────────────────────────────────────────────────────────
// Implementation for vision.h. Owns its own HUSKYLENS instance and state -
// nothing here is extern'd into globals.h on purpose, to keep this module
// removable without ripple effects on the rest of the codebase.

#include "vision.h"
#include "protocol.h"   // triggerColorRun — same path the /api/run/* routes use
#include "arm.h"        // isPlaying/isCycling/presetActive — don't preempt a move
#include "favoriot.h"
#include <HUSKYLENS.h>

// ---------------- USER CONFIG ----------------
static const int  VISION_UART_RX = 16;   // ESP32 RX2 <- HuskyLens TX
static const int  VISION_UART_TX = 17;   // ESP32 TX2 -> HuskyLens RX
static const long VISION_UART_BAUD = 9600;

// Tag ID → color sequence. Index = HuskyLens learned tag ID (1-based).
// Runs through triggerColorRun(), i.e. the exact code path behind the
// /api/run/red|yellow|blue endpoints — no HTTP loopback: those routes live
// on this same ESP32, and a blocking self-POST would stall the motion loop.
static const char* TAG_COLOR[] = { nullptr, "red", "yellow", "blue" };  // tags 1..3
static const uint8_t TAG_PRESET[] = { 0, PRESET_RED, PRESET_YELLOW, PRESET_BLUE };
static const int   TAG_COLOR_N = sizeof(TAG_COLOR) / sizeof(TAG_COLOR[0]);

static const unsigned long VISION_POLL_INTERVAL_MS = 300;
static const unsigned long VISION_TAG_HOLD_MS      = 2000;
static const unsigned long TAG_RUN_DELAY_MS        = 10000;
// ----------------------------------------------

static HUSKYLENS huskylens;
static bool visionReady = false;
static unsigned long lastPollTime = 0;
static int           lastSeenTagID = -1;   // most recent detection (for the UI)
static unsigned long lastSeenMs    = 0;
static int           pendingTagID = -1;
static const char*   pendingColor = nullptr;
static unsigned long pendingRunMs = 0;
static bool          tagTriggerArmed = true;

int visionCurrentTag() {
  if (lastSeenTagID < 0 || millis() - lastSeenMs > VISION_TAG_HOLD_MS) return -1;
  return lastSeenTagID;
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

  bool armBusy = isPlaying || isCycling || presetActive;
  if (pendingTagID >= 1 && !armBusy && millis() - pendingRunMs >= TAG_RUN_DELAY_MS) {
    int tagID = pendingTagID;
    const char* color = pendingColor;
    pendingTagID = -1;
    pendingColor = nullptr;
    favoriotSortStarted(tagID, color);
    triggerColorRun(color, "vision");
  }
  if (!tagTriggerArmed && pendingTagID < 0 && millis() - lastSeenMs > VISION_TAG_HOLD_MS) {
    tagTriggerArmed = true;
  }

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
      bool isNew = (tagID != lastSeenTagID) || (millis() - lastSeenMs > VISION_TAG_HOLD_MS);
      lastSeenTagID = tagID;
      lastSeenMs    = millis();
      if (isNew) Serial.printf("[Vision] Detected tag ID: %d\n", tagID);

      if (tagTriggerArmed && pendingTagID < 0 && !armBusy &&
          tagID >= 1 && tagID < TAG_COLOR_N && TAG_COLOR[tagID]) {
        const char* color = TAG_COLOR[tagID];
        if (presets[TAG_PRESET[tagID]].len <= 0) {
          Serial.printf("[Vision] %s preset missing\n", color);
        } else {
          pendingTagID = tagID;
          pendingColor = color;
          pendingRunMs = millis();
          Serial.printf("[Vision] Tag %d detected; running %s in 10 seconds\n",
                        tagID, color);
        }
        tagTriggerArmed = false;
      }
    }
  }
}
