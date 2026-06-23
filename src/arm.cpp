// ─── arm.cpp — Joint table, smooth motion engine, recording, playback ───────
#include "arm.h"
#include "globals.h"
#include <math.h>

// ── Joint table ─────────────────────────────────────────────────────────────
// minUs/maxUs are physical pulse-width calibration; counts derived per-tick.
Joint joints[6] = {
    //  name           ch   lo   hi  home  cur   inv   minUs maxUs
    { "Base",           5,   0, 270, 129, 129,  false,  342, 2686 },
    { "Shoulder",       0,  30, 150,  62,  62,  true,   342, 2539 },
    { "Elbow",          1,   0, 180,  86,  86,  false,  342, 2686 },
    { "Wrist Pitch",    2,   0, 180,  86,  86,  false,  342, 2686 },
    { "Wrist Roll",     3,   0, 180,  86,  86,  false,  342, 2686 },
    { "Gripper",        4,   0,  90,   0,   0,  false,  342, 1514 },
};

// ── Smooth motion state
float servoCur[6]    = {129, 62, 86, 86, 86, 0};
float servoTarget[6] = {129, 62, 86, 86, 86, 0};
float motionSpeed    = 40.0f;         // deg/sec — ~2.25s for a 90° move (matches preset speed)

// ── Recording state
Pose     seq[MAX_POSES];
int      seqLen     = 0;
bool     isPlaying  = false;
bool     isCycling  = false;
int      playIdx    = 0;
uint32_t playNextMs = 0;

Pose* playSrc    = nullptr;
int   playSrcLen = 0;

Preset presets[MAX_PRESETS] = {
    { "Home",  1, { { { 129, 62, 86, 86, 86,  0 }, "" } } },
    { "Ready", 1, { { {  90, 60, 90, 90, 90, 45 }, "" } } },
    { "Pick",  1, { { {  90, 45, 45, 90, 90,  0 }, "" } } },
    { "Place", 1, { { {  90, 45, 45, 90, 90, 90 }, "" } } },
};

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

// ── Soft-home: staged 1°/step ramp run from setup() before the server opens ──
// Phase order (end-effector → base) for collision avoidance:
//   1. Clearance: gripper, wrist roll, wrist pitch  — retract the tool first.
//   2. Elevation: elbow, shoulder                   — lift the arm body.
//   3. Alignment: base                              — rotate to center last.
// Per joint we ramp from joints[].cur (best-known position; on cold boot this
// equals .home so the inner loop is a no-op and we just emit one settling
// pulse per joint, staggered). When the firmware reboots mid-session the cur
// value may differ from home; this ramp then walks 1° at a time at ~67°/s.
//
// Limitation: there are no encoders. If the arm was physically moved while
// powered off, the first pulse for each joint will still command home and the
// servo will slew to it at its own max speed. The phase staging at least
// guarantees the slews don't happen simultaneously (power-supply spike, and
// uncoordinated arc collisions). Use sendPWM() because setup() is the only
// thread running on this core right now — no async tasks, no motion engine.
static void sendJointTo(uint8_t j, int target) {
    target = constrain(target, joints[j].lo, joints[j].hi);
    int start = joints[j].cur;

    Serial.printf("  %-12s ch%-2d  %3d -> %3d deg (%d deg)\n",
                  joints[j].name, joints[j].ch, start, target, abs(target - start));

    // Walk 1°/step at SOFT_HOME_STEP_MS per degree. Servo follows the staged
    // pulses instead of slewing at its own max rate, so the boot move is
    // visible and matches motionSpeed (40°/s). Motion engine isn't running
    // yet (we're still in setup()), so the bus is ours — sendPWM is safe.
    int step = (target > start) ? 1 : (target < start) ? -1 : 0;
    for (int a = start; a != target; a += step) {
        sendPWM(j, a + step);
        delay(SOFT_HOME_STEP_MS);
    }
    delay(SOFT_HOME_SETTLE_MS);

    joints[j].cur  = target;
    servoCur[j]    = (float)target;
    servoTarget[j] = (float)target;
}

// Phase tables shared by soft-home (blocking, setup-time) and applyPreset
// (async, loop-time). Same order: clearance → elevation → alignment.
static const uint8_t  PHASE_CLEARANCE[3] = { 5, 4, 3 }; // Gripper, Wrist Roll, Wrist Pitch
static const uint8_t  PHASE_ELEVATION[2] = { 2, 1 };    // Elbow, Shoulder
static const uint8_t  PHASE_ALIGNMENT[1] = { 0 };       // Base
static const uint8_t  PHASE_SIZES[3]     = { 3, 2, 1 };
static const uint8_t* PHASE_JOINTS[3]    = {
    PHASE_CLEARANCE, PHASE_ELEVATION, PHASE_ALIGNMENT
};
static const char*    PHASE_NAMES[3]     = { "Clearance", "Elevation", "Alignment" };

void moveToPose(const int p[6]) {
    // Stage commands one joint at a time, in clearance → elevation →
    // alignment order. Each joint gets one pulse to its target angle in p[];
    // SOFT_HOME_SETTLE_MS separates them so the servos don't all slew at once
    // (power-supply spike) and the user sees a deliberate boot sequence. The
    // servos themselves slew at their own max rate from wherever they are.
    for (uint8_t ph = 0; ph < 3; ph++) {
        Serial.printf("[BOOT] Phase: %s\n", PHASE_NAMES[ph]);
        for (uint8_t k = 0; k < PHASE_SIZES[ph]; k++) {
            uint8_t j = PHASE_JOINTS[ph][k];
            sendJointTo(j, p[j]);
        }
    }
    Serial.println("[BOOT] Pose reached");
}

void moveToSafePosition() {
    int homePose[6];
    for (int i = 0; i < 6; i++) homePose[i] = joints[i].home;
    moveToPose(homePose);
}

void moveToHomePose() {
    // Boot target = presets[0] ("Home"). User-renamable, so re-teaching the
    // Home preset from the UI changes the boot landing without a reflash.
    moveToPose(presets[0].seq[0].a);
}

// ── Per-channel PCA9685 write/readback self-test ────────────────────────────
// Catches: dead PCA9685, wrong chip at 0x40, I²C corruption on a specific
// channel, mid-boot I²C glitches. DOES NOT catch a pulled servo wire — the
// chip will happily store a duty cycle for a channel whose connector is
// dangling. For real per-servo detection you need rail current sensing.
// We probe just the 6 channels we actually drive; reads use the Adafruit
// lib's getPWM(), writes use setPWM(). Restore each channel to 0 (off) after
// the probe so we don't leave stale duty cycles before the homing ramp.
bool selfTestServoChannels() {
    const uint16_t TEST_OFF = 0x0AAA;     // arbitrary pattern, distinct from 0/4095
    bool allOk = true;
    Serial.println("[INIT] PCA9685 channel readback:");
    for (uint8_t i = 0; i < 6; i++) {
        uint8_t ch = joints[i].ch;
        pca9685.setPWM(ch, 0, TEST_OFF);
        delayMicroseconds(200);                    // let the I²C write settle
        uint16_t off = pca9685.getPWM(ch, true);   // true = read OFF register
        bool ok = (off == TEST_OFF);
        Serial.printf("  ch%-2d (%-12s) wrote=0x%04X read=0x%04X  %s\n",
                      ch, joints[i].name, TEST_OFF, off, ok ? "PASS" : "FAIL");
        if (!ok) allOk = false;
        pca9685.setPWM(ch, 0, 0);                  // park channel off before homing
    }
    Serial.printf("[INIT] Channel readback: %s\n", allOk ? "ALL PASS" : "FAILURES");
    return allOk;
}

// ── Staged preset move (async, runs through the motion engine) ──────────────
// Drips the preset's target angles into servoTarget one phase at a time,
// waiting for each phase to settle before starting the next. Lowers
// motionSpeed to PRESET_SPEED_DEG_S while active; restores on completion.
// processPresetMove() drives the state machine from loop().
bool       presetActive   = false;
static int presetTarget[6];
static uint8_t presetPhase    = 0;
static float   presetSavedSpd = 0;

static void presetStartPhase(uint8_t phase) {
    Serial.printf("[PRESET] Phase %u: %s\n", (unsigned)phase + 1, PHASE_NAMES[phase]);
    for (uint8_t k = 0; k < PHASE_SIZES[phase]; k++) {
        uint8_t j = PHASE_JOINTS[phase][k];
        setServo(j, presetTarget[j]);   // motion engine ramps at PRESET_SPEED_DEG_S
    }
}

void applyPreset(const int p[6]) {
    for (int i = 0; i < 6; i++) presetTarget[i] = p[i];
    if (!presetActive) presetSavedSpd = motionSpeed;
    motionSpeed   = PRESET_SPEED_DEG_S;
    presetPhase   = 0;
    presetActive  = true;
    presetStartPhase(0);
    pendingBroadcast = true;
}

void processPresetMove() {
    if (!presetActive) return;

    // Settling check: every joint in the phase just dispatched must be within
    // PRESET_SETTLE_DEG of its target before we advance.
    uint8_t phaseIdx = (presetPhase >= 3) ? 2 : presetPhase;
    bool settled = true;
    for (uint8_t k = 0; k < PHASE_SIZES[phaseIdx]; k++) {
        uint8_t j = PHASE_JOINTS[phaseIdx][k];
        if (fabsf(servoTarget[j] - servoCur[j]) > PRESET_SETTLE_DEG) {
            settled = false; break;
        }
    }
    if (!settled) return;

    presetPhase++;
    if (presetPhase >= 3) {
        presetActive  = false;
        motionSpeed   = presetSavedSpd;
        Serial.println("[PRESET] Complete");
        pendingBroadcast = true;
        return;
    }
    presetStartPhase(presetPhase);
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
    playSrc = seq; playSrcLen = seqLen;
    isCycling = false; isPlaying = true;
    playIdx = 0; playNextMs = millis();
    pendingBroadcast = true;
}

void stopPlayback() { isPlaying = false; isCycling = false; pendingBroadcast = true; }

void startCycle() {
    if (!seqLen) { Serial.println("[CYCLE] Empty"); return; }
    playSrc = seq; playSrcLen = seqLen;
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
    if (idx >= MAX_PRESETS) return;
    int* p = presets[idx].seq[0].a;
    for (int i = 0; i < 6; i++) p[i] = joints[i].cur;
    presets[idx].len = 1;          // teaching a single pose collapses the slot
    savePresetsToFlash();
    Serial.printf("[PRESET %u \"%s\"] taught: [%d %d %d %d %d %d]\n",
        (unsigned)idx, presets[idx].name,
        p[0], p[1], p[2], p[3], p[4], p[5]);
}

void renamePreset(uint8_t idx, const char* name) {
    if (idx >= MAX_PRESETS || !name) return;
    strncpy(presets[idx].name, name, sizeof(presets[idx].name) - 1);
    presets[idx].name[sizeof(presets[idx].name) - 1] = '\0';
    savePresetsToFlash();
    Serial.printf("[PRESET %u] renamed to \"%s\"\n", (unsigned)idx, presets[idx].name);
}

void playPreset(uint8_t idx) {
    if (idx >= MAX_PRESETS) return;
    if (presets[idx].len <= 0) {
        Serial.printf("[PRESET %u \"%s\"] empty\n", (unsigned)idx, presets[idx].name);
        return;
    }
    if (presets[idx].len == 1) {
        // Single-pose preset → staged 3-phase move (clearance → elev → align)
        applyPreset(presets[idx].seq[0].a);
        Serial.printf("[PRESET %u \"%s\"] staged 1-pose move\n",
                      (unsigned)idx, presets[idx].name);
    } else {
        // Multi-pose preset → playback through motion engine
        playSrc = presets[idx].seq;
        playSrcLen = presets[idx].len;
        isCycling = false; isPlaying = true;
        playIdx = 0; playNextMs = millis();
        pendingBroadcast = true;
        Serial.printf("[PRESET %u \"%s\"] playing %d poses\n",
                      (unsigned)idx, presets[idx].name, presets[idx].len);
    }
}

void savePresetsToFlash() {
    prefs.begin("roboarm", false);
    for (uint8_t i = 0; i < MAX_PRESETS; i++) {
        char key[4]; snprintf(key, sizeof(key), "ps%u", (unsigned)i);
        prefs.putBytes(key, &presets[i], sizeof(Preset));
    }
    prefs.end();
    Serial.printf("[FLASH] Presets saved (%u slots)\n", (unsigned)MAX_PRESETS);
}

void loadPresetsFromFlash() {
    // Migration: convert old per-pose keys (ph/pr/pk/pl) into the new preset
    // format on first boot after the upgrade. Old keys are removed afterwards
    // so this only fires once.
    prefs.begin("roboarm", false);
    bool migrated = false;
    if (prefs.isKey("ph") && !prefs.isKey("ps0")) {
        int oldHome[6], oldReady[6], oldPick[6], oldPlace[6];
        prefs.getBytes("ph", oldHome,  sizeof(oldHome));
        prefs.getBytes("pr", oldReady, sizeof(oldReady));
        prefs.getBytes("pk", oldPick,  sizeof(oldPick));
        prefs.getBytes("pl", oldPlace, sizeof(oldPlace));
        for (int j = 0; j < 6; j++) {
            presets[0].seq[0].a[j] = oldHome[j];
            presets[1].seq[0].a[j] = oldReady[j];
            presets[2].seq[0].a[j] = oldPick[j];
            presets[3].seq[0].a[j] = oldPlace[j];
        }
        for (uint8_t i = 0; i < MAX_PRESETS; i++) presets[i].len = 1;
        for (uint8_t i = 0; i < MAX_PRESETS; i++) {
            char key[4]; snprintf(key, sizeof(key), "ps%u", (unsigned)i);
            prefs.putBytes(key, &presets[i], sizeof(Preset));
        }
        prefs.remove("ph"); prefs.remove("pr");
        prefs.remove("pk"); prefs.remove("pl");
        migrated = true;
    }
    prefs.end();

    prefs.begin("roboarm", true);
    for (uint8_t i = 0; i < MAX_PRESETS; i++) {
        char key[4]; snprintf(key, sizeof(key), "ps%u", (unsigned)i);
        if (prefs.isKey(key)) prefs.getBytes(key, &presets[i], sizeof(Preset));
    }
    prefs.end();

    Serial.printf("[FLASH] Presets loaded%s\n",
                  migrated ? " (migrated from old format)" : "");
}

// ── Playback ────────────────────────────────────────────────────────────────
void processPlayback() {
    if (!isPlaying && !isCycling) return;
    if (millis() < playNextMs)    return;
    if (!playSrc || playSrcLen == 0) {
        isPlaying = false; isCycling = false; return;
    }

    if (playIdx >= playSrcLen) {
        if (isCycling) { playIdx = 0; }
        else {
            isPlaying = false; playIdx = 0;
            Serial.println("[PLAY] Done");
            pendingBroadcast = true;
            return;
        }
    }

    for (int i = 0; i < 6; i++) setServo(i, playSrc[playIdx].a[i]);   // smooth ramp
    Serial.printf("[%s] %d/%d \"%s\"\n",
        isCycling ? "CYC" : "PLAY", playIdx + 1, playSrcLen, playSrc[playIdx].label);
    playIdx++;
    playNextMs = millis() + PLAY_STEP_MS;
    pendingBroadcast = true;
}
