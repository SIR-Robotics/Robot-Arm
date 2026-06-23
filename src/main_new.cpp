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
Adafruit_PWMServoDriver pca9685(PCA9685_ADDR);
AsyncWebServer          server(80);
AsyncWebSocket          ws("/ws");
Preferences             prefs;
SemaphoreHandle_t       servoMutex = nullptr;
QueueHandle_t           wsQueue    = nullptr;
bool                    pendingBroadcast = false;

static uint32_t lastBroadMs = 0;

// ─── Setup ──────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== RoboArm 6-DOF (main_new) ===");

    servoMutex = xSemaphoreCreateMutex();
    wsQueue    = xQueueCreate(32, sizeof(WsMsg));

    Wire.begin(I2C_SDA, I2C_SCL);
    pca9685.begin();
    pca9685.setOscillatorFrequency(OSC_FREQ);
    pca9685.setPWMFreq(PWM_FREQ_HZ);
    delay(10);
    Serial.println("[PCA9685] OK");

    pinMode(JOY_SW_PIN,    INPUT_PULLUP);
    pinMode(BTN_REC_PIN,   INPUT_PULLUP);
    pinMode(BTN_PLAY_PIN,  INPUT_PULLUP);
    pinMode(BTN_CLR_PIN,   INPUT_PULLUP);
    pinMode(BTN_CYCLE_PIN, INPUT_PULLUP);

    calibrateJoystick();
    loadPresetsFromFlash();
    moveToHome();
    Serial.println("[Servos] Home");

    loadFromFlash();

    WiFi.mode(WIFI_STA);
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
        Serial.println("[WiFi] FAILED");
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
    WsMsg m;
    while (xQueueReceive(wsQueue, &m, 0)) processWsCmd(m.buf);

    processJoystick();
    processWebJog();
    processButtons();
    processPlayback();
    processMotion();          // ramps servoCur -> servoTarget at motionSpeed
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
}
