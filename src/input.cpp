// ─── input.cpp — HW joystick, buttons, and web jog ──────────────────────────
// All motion paths just push into servoTarget[] — the motion engine in
// arm.cpp ramps servoCur[] toward target at motionSpeed.
#include "input.h"
#include "arm.h"
#include "globals.h"

// ── HW joystick state ───────────────────────────────────────────────────────
uint8_t  joyMode    = 0;
int      joyXCenter = 2048;
int      joyYCenter = 2048;
bool     joyActive  = false;
uint32_t joyLastMs  = 0;

const uint8_t PAIR[3][2]   = { {0,1}, {2,3}, {4,5} };
const char*   PAIR_NAME[3] = { "Base+Shoulder", "Elbow+Wrist Pitch", "Wrist Roll+Gripper" };

// ── Web jog state ───────────────────────────────────────────────────────────
int      webJog[6]    = {0,0,0,0,0,0};
uint32_t lastWebJogMs = 0;
bool     webJogActive = false;

// ── Button debounce timers
static uint32_t lastSW = 0, lastRec = 0, lastPlay = 0, lastClr = 0, lastCycle = 0;

// ── HW joystick ─────────────────────────────────────────────────────────────
bool calibrateJoystick() {
    long xs = 0, ys = 0;
    const int N = 64;
    for (int i = 0; i < N; i++) {
        xs += analogRead(JOY_X_PIN);
        ys += analogRead(JOY_Y_PIN);
        delay(5);
    }
    joyXCenter = xs / N;
    joyYCenter = ys / N;

    // Sanity: 12-bit ADC centered on a 3.3V/2 divider should land near 2048.
    // Pinned-to-rail (0 or 4095) means a broken pot or shorted wire.
    bool ok = (joyXCenter >= 1600 && joyXCenter <= 2500 &&
               joyYCenter >= 1600 && joyYCenter <= 2500);
    Serial.printf("[JOY] Center X=%d Y=%d  %s\n",
                  joyXCenter, joyYCenter, ok ? "OK" : "OUT OF RANGE");
    return ok;
}

void processJoystick() {
    if (presetActive) { joyActive = false; return; }
    if (millis() - joyLastMs < JOY_INTERVAL) return;
    joyLastMs = millis();

    int xRaw = analogRead(JOY_X_PIN);
    int yRaw = analogRead(JOY_Y_PIN);

    float xD = 0, yD = 0;
    if (abs(xRaw - joyXCenter) > JOY_DEADZONE)
        xD = (xRaw - joyXCenter) / (float)joyXCenter * JOY_SPEED;
    if (abs(yRaw - joyYCenter) > JOY_DEADZONE)
        yD = (yRaw - joyYCenter) / (float)joyYCenter * JOY_SPEED;

    uint8_t jA = PAIR[joyMode][0];
    uint8_t jB = PAIR[joyMode][1];
    joyActive = (xD != 0.0f || yD != 0.0f);

    if (xD != 0.0f)
        servoTarget[jA] = constrain(servoTarget[jA] + xD,
                                    (float)joints[jA].lo, (float)joints[jA].hi);
    if (yD != 0.0f)
        servoTarget[jB] = constrain(servoTarget[jB] + yD,
                                    (float)joints[jB].lo, (float)joints[jB].hi);
}

// ── Buttons ─────────────────────────────────────────────────────────────────
static bool btnPressed(uint8_t pin, uint32_t& last) {
    if (digitalRead(pin) == LOW && millis() - last > DEBOUNCE_MS) {
        last = millis(); return true;
    }
    return false;
}

void processButtons() {
    if (btnPressed(JOY_SW_PIN,    lastSW))    { joyMode = (joyMode + 1) % 3; Serial.printf("[JOY] %s\n", PAIR_NAME[joyMode]); pendingBroadcast = true; }
    if (btnPressed(BTN_REC_PIN,   lastRec))   recordPose();
    if (btnPressed(BTN_PLAY_PIN,  lastPlay))  isPlaying ? stopPlayback() : startPlayback();
    if (btnPressed(BTN_CLR_PIN,   lastClr))   clearRecording();
    if (btnPressed(BTN_CYCLE_PIN, lastCycle)) isCycling ? stopCycle()    : startCycle();
}

// ── Web jog ─────────────────────────────────────────────────────────────────
// webJog[j] is -100..100 from sticks/keyboard/gamepad. Per tick we push the
// target float by (webJog/100)*WEB_JOG_SPEED. Motion engine catches up.
void processWebJog() {
    if (isPlaying || isCycling || presetActive) { webJogActive = false; return; }
    if (millis() - lastWebJogMs < WEB_JOG_INTERVAL) return;
    lastWebJogMs = millis();

    bool anyActive = false;
    for (int j = 0; j < 6; j++) {
        if (webJog[j] == 0) continue;
        anyActive = true;
        float d = (webJog[j] / 100.0f) * WEB_JOG_SPEED;
        servoTarget[j] = constrain(servoTarget[j] + d,
                                   (float)joints[j].lo, (float)joints[j].hi);
    }
    webJogActive = anyActive;
}
