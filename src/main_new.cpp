// ─── RoboArm 6-DOF Controller — main_new.cpp ────────────────────────────────
// Thin orchestrator: defines singleton handles, wires up setup() and loop().
// Real work lives in the per-feature modules:
//
//   config.h     ─ pins, timing, types (Joint, Pose, WsMsg)
//   globals.h    ─ singleton externs (this file owns the defs)
//   arm.h/cpp    ─ joints[], servo math, recording, playback
//   input.h/cpp  ─ HW joystick, buttons, web jog
//   protocol.h/.cpp ─ JSON builders, WS handler, serial commands
//   web_ui.h/cpp ─ HTML/CSS/JS payload
//
// To switch back to the single-file build, see git tag `pre-segmentation`.
#include "vision.h"
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>

#include "config.h"
#include "globals.h"
#include "arm.h"
#include "input.h"
#include "protocol.h"
#include "web_ui.h"

// ─── WiFi (STA) ─────────────────────────────────────────────────────────────
const char* WIFI_SSID = "ASEM Training";
const char* WIFI_PASS = "Class@Asem";

// ─── Singleton definitions (extern'd in globals.h) ──────────────────────────
BootState               bootState  = STATE_INIT;
Adafruit_PWMServoDriver pca9685(PCA9685_ADDR);
AsyncWebServer          server(80);
AsyncWebSocket          ws("/ws");
Preferences             prefs;
SemaphoreHandle_t       servoMutex = nullptr;
QueueHandle_t           wsQueue    = nullptr;
bool                    pendingBroadcast = false;

static uint32_t lastBroadMs = 0;
static uint32_t lastFkMs    = 0;

// ─── Setup ──────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== RoboArm 6-DOF (main_new) ===");

    servoMutex = xSemaphoreCreateMutex();
    wsQueue    = xQueueCreate(32, sizeof(WsMsg));

    // ── STATE_INIT: full self-audit ─────────────────────────────────────────
    bootState = STATE_INIT;
    Serial.println("[INIT] STATE_INIT — running self-audit");
    Wire.begin(I2C_SDA, I2C_SCL);

    // 1. I²C ACK probe: detects missing/unpowered PCA9685
    Wire.beginTransmission(PCA9685_ADDR);
    uint8_t ack = Wire.endTransmission();
    bool i2cOk = (ack == 0);
    Serial.printf("[INIT] (1/3) I2C ACK on 0x%02X: %s",
                  PCA9685_ADDR, i2cOk ? "OK\n" : "");
    if (!i2cOk) {
        Serial.printf("FAIL (err=%u)\n", ack);
        bootState = STATE_FAULT;
    }

    // 2. PCA9685 per-channel write/readback: detects dead channels / wrong chip
    bool channelsOk = false;
    if (bootState != STATE_FAULT) {
        pca9685.begin();
        pca9685.setOscillatorFrequency(OSC_FREQ);
        pca9685.setPWMFreq(PWM_FREQ_HZ);
        delay(10);
        Serial.println("[INIT] (2/3) PCA9685 configured");
        channelsOk = selfTestServoChannels();
        if (!channelsOk) {
            Serial.println("[INIT] FAULT: one or more PCA9685 channels misbehaving");
            bootState = STATE_FAULT;
        }
    }

    pinMode(JOY_SW_PIN,    INPUT_PULLUP);
    pinMode(BTN_REC_PIN,   INPUT_PULLUP);
    pinMode(BTN_PLAY_PIN,  INPUT_PULLUP);
    pinMode(BTN_CLR_PIN,   INPUT_PULLUP);
    pinMode(BTN_CYCLE_PIN, INPUT_PULLUP);

    // 3. Joystick sanity: pinned-to-rail center means broken pot or shorted wire
    bool joyOk = calibrateJoystick();
    Serial.printf("[INIT] (3/3) Joystick centers: %s\n", joyOk ? "OK" : "OUT OF RANGE");
    if (!joyOk && bootState != STATE_FAULT) {
        Serial.println("[INIT] WARN: joystick degraded — continuing (web UI still works)");
        // Non-fatal: web UI can drive the arm without the HW stick.
    }

    // 4. HuskyLens (vision): non-fatal, arm runs fine even if this fails.    // <-- ADDED
    // NOTE: uses Serial2 on GPIO16/17 - confirm these don't collide with    // <-- ADDED
    // I2C_SDA/I2C_SCL or any BTN_*_PIN/JOY_SW_PIN in config.h.              // <-- ADDED
    visionInit();                                                            // <-- ADDED


    loadPresetsFromFlash();
    loadFromFlash();

    // ── STATE_HOMING: Wrist → Elbow → Shoulder → Base to Home ──────────────
    if (bootState != STATE_FAULT) {
        bootState = STATE_HOMING;
        Serial.println("[INIT] STATE_HOMING -> Home pose");
        moveToHomePose();
        bootState = STATE_IDLE;
        Serial.println("[INIT] STATE_IDLE — type ? for serial help");
    } else {
        Serial.println("[INIT] Skipping homing — audit failed. WiFi/UI still up.");
    }

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("[WiFi] Joining \"%s\"", WIFI_SSID);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
        delay(400); Serial.print('.');
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        WiFi.setSleep(false);     // keep radio hot — phones already power-save aggressively
        Serial.printf("[WiFi] http://%s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.printf("[WiFi] FAILED (status=%d); retrying in background\n",
                      (int)WiFi.status());
    }

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req){
        req->send(200, "text/html", HTML_PAGE);
    });
    registerHttpRoutes(server);                  // /poses.json GET + POST
    server.begin();
    Serial.println("Send HELP for serial commands.");
}

// ─── Loop ───────────────────────────────────────────────────────────────────
void loop() {
    static uint32_t lastWifiRetryMs = 0;
    static bool wifiWasConnected = WiFi.status() == WL_CONNECTED;
    bool wifiConnected = WiFi.status() == WL_CONNECTED;
    if (wifiConnected && !wifiWasConnected) {
        Serial.printf("[WiFi] http://%s\n", WiFi.localIP().toString().c_str());
    } else if (!wifiConnected && millis() - lastWifiRetryMs >= 10000) {
        lastWifiRetryMs = millis();
        Serial.println("[WiFi] Retrying...");
        WiFi.reconnect();
    }
    wifiWasConnected = wifiConnected;

    WsMsg m;
    while (xQueueReceive(wsQueue, &m, 0)) processWsCmd(m.buf);

    // Vision runs independent of bootState/arm motion - HuskyLens detection    // <-- ADDED
    // and its HTTP POSTs should keep working even if the arm faulted.         // <-- ADDED
    visionPoll();

    // Push a status frame whenever the visible tag changes (including the
    // -1 "cleared" transition) so the UI indicator updates without waiting
    // for other activity.
    static int lastTagShown = -2;
    int curTag = visionCurrentTag();
    if (curTag != lastTagShown) {
        lastTagShown = curTag;
        pendingBroadcast = true;
    }

    // STATE_FAULT: skip every motion path. WS + serial stay alive so the user
    // sees the fault on the UI and can issue a STATUS / RAW probe over serial.
    if (bootState != STATE_FAULT) {
        processJoystick();
        processWebJog();
        processButtons();
        processPlayback();
        processPresetMove();  // advances staged preset moves between phases
        processMotion();      // ramps servoCur -> servoTarget at motionSpeed

        // Recompute IDLE ↔ BUSY each tick. Only consider transitions once
        // homing is finished — INIT/HOMING/FAULT stay put.
        if (bootState == STATE_IDLE || bootState == STATE_BUSY) {
            BootState target = (presetActive || isPlaying || isCycling)
                             ? STATE_BUSY : STATE_IDLE;
            if (target != bootState) {
                bootState = target;
                pendingBroadcast = true;
            }
        }
    }
    processSerial();

    static uint32_t lastCleanupMs = 0;
    if (millis() - lastCleanupMs >= 1000) {
        ws.cleanupClients();   // uses lib default; never evict an active client
        lastCleanupMs = millis();
    }

    if ((pendingBroadcast || joyActive || webJogActive)
        && millis() - lastBroadMs >= 50) {
        broadcastStatus();
        lastBroadMs      = millis();
        pendingBroadcast = false;
    }

    if (millis() - lastFkMs >= 500) {
        printFK();
        lastFkMs = millis();
    }
}
