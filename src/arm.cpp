// ─── arm.cpp — Joint table, smooth motion engine, recording, playback ───────
#include "arm.h"
#include "globals.h"
#include "startup_pwm.h"
#include <math.h>

// ── Joint table ─────────────────────────────────────────────────────────────
// minUs/maxUs are physical pulse-width calibration; counts derived per-tick.
Joint joints[6] = {
    //  name           ch   lo   hi  home  cur   inv   minUs maxUs
    { "Base",           5,   0, 270, 135, 135,  false,  500, 2500 },
    { "Shoulder",       0,  30, 150,  90,  90,  true,   500, 2500 },
    { "Elbow",          1,   0, 180,  90,  90,  false,  500, 2500 },
    { "Wrist Pitch",    2,   0, 180,  90,  90,  false,  500, 2500 },
    { "Wrist Roll",     3,   0, 180,  90,  90,  false,  500, 2500 },
    { "Gripper",        4,   0,  90,  45,  45,  false,  500, 2500 },
};

// ── Smooth motion state
float servoCur[6]    = {135, 90, 90, 90, 90, 45};
float servoTarget[6] = {135, 90, 90, 90, 90, 45};
float motionSpeed    = 120.0f;        // deg/sec — ~0.75s for a 90° move

// ── Recording state
Pose     seq[MAX_POSES];
int      seqLen     = 0;
bool     isPlaying  = false;
bool     isCycling  = false;
int      playIdx    = 0;
uint32_t playNextMs = 0;

int POSE_HOME [6] = { 90, 90, 90, 90, 90, 45 };
int POSE_READY[6] = { 90, 60, 90, 90, 90, 45 };
int POSE_PICK [6] = { 90, 45, 45, 90, 90,  0 };
int POSE_PLACE[6] = { 90, 45, 45, 90, 90, 90 };

// ── Servo math ──────────────────────────────────────────────────────────────
uint16_t toCounts(const Joint& j, int angle) {
    long us = (long)j.minUs +
              ((long)(angle - j.lo) * ((long)j.maxUs - j.minUs)) / (j.hi - j.lo);
    long lo = min((long)j.minUs, (long)j.maxUs);
    long hi = max((long)j.minUs, (long)j.maxUs);
    return (uint16_t)(constrain(us, lo, hi) * 4096L / 20000L);
}

// Smooth path — UI/serial/playback/preset use this. processMotion picks it up.
void setServo(uint8_t i, int angle) {
    servoTarget[i] = (float)constrain(angle, joints[i].lo, joints[i].hi);
}

// Immediate path — startup, RAW, debug TEST. Bypasses motion engine entirely.
void setServoNow(uint8_t i, int angle) {
    angle = constrain(angle, joints[i].lo, joints[i].hi);
    joints[i].cur  = angle;
    servoCur[i]    = (float)angle;
    servoTarget[i] = (float)angle;
    int phys = joints[i].invert ? (joints[i].lo + joints[i].hi - angle) : angle;
    if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        pca9685.setPWM(joints[i].ch, 0, toCounts(joints[i], phys));
        xSemaphoreGive(servoMutex);
    }
}

// Raw fast path — no mutex, no state sync. Only safe when caller owns the bus.
void sendPWM(uint8_t i, int angle) {
    joints[i].cur = angle;
    int phys = joints[i].invert ? (joints[i].lo + joints[i].hi - angle) : angle;
    pca9685.setPWM(joints[i].ch, 0, toCounts(joints[i], phys));
}

void moveToHome() {
    for (int i = 0; i < 6; i++) setServoNow(i, joints[i].home);
}

static void writePwmCounts(const uint16_t counts[6]) {
    xSemaphoreTake(servoMutex, portMAX_DELAY);
    for (uint8_t channel = 0; channel < 6; ++channel) {
        pca9685.setPWM(channel, 0, counts[channel]);
    }
    xSemaphoreGive(servoMutex);
}

static void rampPwmChannels(uint16_t current[6], const uint16_t target[6],
                            const uint8_t channels[], uint8_t channelCount) {
    bool moved;
    do {
        moved = false;
        xSemaphoreTake(servoMutex, portMAX_DELAY);
        for (uint8_t i = 0; i < channelCount; ++i) {
            uint8_t channel = channels[i];
            uint16_t next = stepPwmToward(current[channel], target[channel]);
            if (next == current[channel]) continue;
            current[channel] = next;
            pca9685.setPWM(channel, 0, next);
            moved = true;
        }
        xSemaphoreGive(servoMutex);
        if (moved) delay(30);  // one PWM count per channel, about 15 deg/sec
    } while (moved);
}

void moveToSafePosition() {
    static const uint16_t startPwmByChannel[6] = {300, 400, 300, 300, 300, 300};
    static const uint16_t safePwmByChannel[6]  = {400, 300, 300, 300, 300, 300};
    static const uint8_t clearanceChannels[] = {2, 3, 4};
    static const uint8_t elevationChannels[] = {0, 1};
    static const uint8_t alignmentChannels[] = {5};
    uint16_t current[6];

    for (uint8_t channel = 0; channel < 6; ++channel) {
        current[channel] = startPwmByChannel[channel];
    }
    writePwmCounts(current);

    rampPwmChannels(current, safePwmByChannel, clearanceChannels, 3);
    rampPwmChannels(current, safePwmByChannel, elevationChannels, 2);
    rampPwmChannels(current, safePwmByChannel, alignmentChannels, 1);
    writePwmCounts(safePwmByChannel);

    for (uint8_t i = 0; i < 6; ++i) {
        const Joint& joint = joints[i];
        int angle = pwmCountToLogicalAngle(safePwmByChannel[joint.ch],
                                            joint.lo, joint.hi, joint.invert);
        joints[i].cur = angle;
        servoCur[i] = (float)angle;
        servoTarget[i] = (float)angle;
    }
}

void applyPreset(const int p[6]) {
    for (int i = 0; i < 6; i++) setServo(i, p[i]);   // smooth
}

// ── Motion engine — ramp servoCur toward servoTarget at motionSpeed ─────────
void processMotion() {
    static uint32_t lastMs = 0;
    uint32_t now = millis();
    if (lastMs == 0) { lastMs = now; return; }
    uint32_t dt = now - lastMs;
    if (dt < 5) return;                              // ≤200 Hz tick
    lastMs = now;

    float maxStep = motionSpeed * (dt * 0.001f);     // deg this tick
    bool moved = false;

    for (int i = 0; i < 6; i++) {
        float diff = servoTarget[i] - servoCur[i];
        if (fabsf(diff) < 0.05f) {
            servoCur[i] = servoTarget[i];
            continue;
        }
        float step = (fabsf(diff) <= maxStep) ? diff : (diff > 0 ? maxStep : -maxStep);
        servoCur[i] += step;

        int newAngle = (int)lroundf(servoCur[i]);
        if (newAngle == joints[i].cur) continue;

        int phys = joints[i].invert
                 ? (joints[i].lo + joints[i].hi - newAngle)
                 : newAngle;
        if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            joints[i].cur = newAngle;
            pca9685.setPWM(joints[i].ch, 0, toCounts(joints[i], phys));
            xSemaphoreGive(servoMutex);
            moved = true;
        }
    }
    if (moved) pendingBroadcast = true;
}

// ── Recording ───────────────────────────────────────────────────────────────
void recordPose() {
    if (seqLen >= MAX_POSES) { Serial.println("[REC] Full"); return; }
    for (int i = 0; i < 6; i++) seq[seqLen].a[i] = joints[i].cur;
    snprintf(seq[seqLen].label, sizeof(seq[seqLen].label), "Pose %d", seqLen + 1);
    seqLen++;
    Serial.printf("[REC] %d total\n", seqLen);
    pendingBroadcast = true;
}

void renamePose(int idx, const char* name) {
    if (idx < 0 || idx >= seqLen) return;
    strncpy(seq[idx].label, name, sizeof(seq[idx].label) - 1);
    seq[idx].label[sizeof(seq[idx].label) - 1] = '\0';
}

void startPlayback() {
    if (!seqLen) { Serial.println("[PLAY] Empty"); return; }
    isCycling = false; isPlaying = true;
    playIdx = 0; playNextMs = millis();
    pendingBroadcast = true;
}

void stopPlayback() { isPlaying = false; isCycling = false; pendingBroadcast = true; }

void startCycle() {
    if (!seqLen) { Serial.println("[CYCLE] Empty"); return; }
    isPlaying = false; isCycling = true;
    playIdx = 0; playNextMs = millis();
    pendingBroadcast = true;
}

void stopCycle() { isCycling = false; pendingBroadcast = true; }

void clearRecording() {
    seqLen = 0; isPlaying = false; isCycling = false; pendingBroadcast = true;
    Serial.println("[REC] Cleared");
}

void saveToFlash() {
    prefs.begin("roboarm", false);
    prefs.putInt("len", seqLen);
    if (seqLen > 0) prefs.putBytes("seq", seq, seqLen * sizeof(Pose));
    prefs.end();
    Serial.printf("[FLASH] Saved %d\n", seqLen);
}

void loadFromFlash() {
    prefs.begin("roboarm", true);
    seqLen = prefs.getInt("len", 0);
    if (seqLen > 0) prefs.getBytes("seq", seq, seqLen * sizeof(Pose));
    prefs.end();
    Serial.printf("[FLASH] Loaded %d\n", seqLen);
    pendingBroadcast = true;
}

// ── Teachable presets ──────────────────────────────────────────────────────
void setPresetFromCurrent(uint8_t idx) {
    int* p = nullptr;
    const char* name = "";
    switch (idx) {
        case 0: p = POSE_HOME;  name = "home";  break;
        case 1: p = POSE_READY; name = "ready"; break;
        case 2: p = POSE_PICK;  name = "pick";  break;
        case 3: p = POSE_PLACE; name = "place"; break;
        default: return;
    }
    for (int i = 0; i < 6; i++) p[i] = joints[i].cur;
    savePresetsToFlash();
    Serial.printf("[PRESET] %s = [%d %d %d %d %d %d]\n", name,
        p[0], p[1], p[2], p[3], p[4], p[5]);
}

void savePresetsToFlash() {
    prefs.begin("roboarm", false);
    prefs.putBytes("ph", POSE_HOME,  sizeof(POSE_HOME));
    prefs.putBytes("pr", POSE_READY, sizeof(POSE_READY));
    prefs.putBytes("pk", POSE_PICK,  sizeof(POSE_PICK));
    prefs.putBytes("pl", POSE_PLACE, sizeof(POSE_PLACE));
    prefs.end();
}

void loadPresetsFromFlash() {
    prefs.begin("roboarm", true);
    if (prefs.isKey("ph")) prefs.getBytes("ph", POSE_HOME,  sizeof(POSE_HOME));
    if (prefs.isKey("pr")) prefs.getBytes("pr", POSE_READY, sizeof(POSE_READY));
    if (prefs.isKey("pk")) prefs.getBytes("pk", POSE_PICK,  sizeof(POSE_PICK));
    if (prefs.isKey("pl")) prefs.getBytes("pl", POSE_PLACE, sizeof(POSE_PLACE));
    prefs.end();
}

// ── Playback ────────────────────────────────────────────────────────────────
void processPlayback() {
    if (!isPlaying && !isCycling) return;
    if (millis() < playNextMs)    return;

    if (playIdx >= seqLen) {
        if (isCycling) { playIdx = 0; }
        else {
            isPlaying = false; playIdx = 0;
            Serial.println("[PLAY] Done");
            pendingBroadcast = true;
            return;
        }
    }

    for (int i = 0; i < 6; i++) setServo(i, seq[playIdx].a[i]);   // smooth ramp
    Serial.printf("[%s] %d/%d \"%s\"\n",
        isCycling ? "CYC" : "PLAY", playIdx + 1, seqLen, seq[playIdx].label);
    playIdx++;
    playNextMs = millis() + PLAY_STEP_MS;
    pendingBroadcast = true;
}
