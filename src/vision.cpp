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
static const unsigned long TAGGING_SCAN_MS         = 5000;
static const unsigned long TAGGING_QUEUE_MS        = 60000;
// ----------------------------------------------

static HUSKYLENS huskylens;
static bool visionReady = false;
static unsigned long lastPollTime = 0;
static int           lastSeenTagID = -1;   // most recent detection (for the UI)
static unsigned long lastSeenMs    = 0;
static uint32_t      taggingRequestId = 0;
static unsigned long taggingRequestedMs = 0;
static unsigned long taggingScanMs = 0;
static bool          taggingScanning = false;

int visionCurrentTag() {
  if (lastSeenTagID < 0 || millis() - lastSeenMs > VISION_TAG_HOLD_MS) return -1;
  return lastSeenTagID;
}

static void finishTagging(const char* status, int tag = -1, const char* color = nullptr) {
  favoriotTaggingResult(taggingRequestId, status, tag, color);
  taggingRequestId = 0;
  taggingScanning = false;
}

void visionRequestTagging(uint32_t id) {
  if (!visionReady) {
    favoriotTaggingResult(id, "vision_unavailable");
    return;
  }
  if (taggingRequestId != 0) {
    favoriotTaggingResult(id, "busy");
    return;
  }

  taggingRequestId = id;
  taggingRequestedMs = millis();
  taggingScanning = false;
  Serial.printf("[Vision] Tagging request %lu queued\n", (unsigned long)id);
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
  if (taggingRequestId != 0) {
    if (millis() - taggingRequestedMs >= TAGGING_QUEUE_MS) {
      finishTagging("queue_timeout");
    } else if (armBusy) {
      taggingScanning = false;
    } else if (!taggingScanning) {
      taggingScanning = true;
      taggingScanMs = millis();
      Serial.printf("[Vision] Tagging request %lu scanning\n",
                    (unsigned long)taggingRequestId);
    } else if (millis() - taggingScanMs >= TAGGING_SCAN_MS) {
      finishTagging("not_found");
    }
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

      if (taggingScanning && tagID >= 1 && tagID < TAG_COLOR_N && TAG_COLOR[tagID]) {
        const char* color = TAG_COLOR[tagID];
        if (presets[TAG_PRESET[tagID]].len <= 0) {
          finishTagging("preset_missing");
        } else {
          triggerColorRun(color, "mobile");
          finishTagging("started", tagID, color);
        }
      }
    }
  }
}
