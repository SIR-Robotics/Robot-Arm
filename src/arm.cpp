// ─── arm.cpp — Joint table, smooth motion engine, recording, playback ───────
#include "arm.h"
#include "input.h"
#include "globals.h"
#include <math.h>

// ── Link geometry (mm) — measured from the physical arm.
// FK origin = shoulder pivot (NOT base servo plate). The 43 mm base column
// is intentionally excluded so Z=0 sits at shoulder height — at full
// horizontal reach the wrist roll center reads Z=0, which is the natural
// reference plane for IK. L4 ends at the wrist roll motor center; the
// 111 mm gripper extension beyond it is a tool offset, applied later by
// pick-and-place code, not part of FK.
constexpr float L2 = 83.0f;   // upper arm: shoulder → elbow
constexpr float L3 = 77.0f;   // forearm:   elbow → wrist pitch axis
constexpr float L4 = 68.0f;   // wrist:     wrist pitch axis → wrist roll center

// ── Joint zero offsets (logical degrees that correspond to FK "zero").
// Reference pose (all zeros) = arm pointing straight along +Y forward,
// wrist roll centered. Calibrated against the operator's "max Y" preset.
constexpr float Z_SHOULDER =  65.0f;
constexpr float Z_ELBOW    =  81.0f;
constexpr float Z_W_PITCH  =  90.0f;
constexpr float Z_W_ROLL   =  90.0f;

// ── Base yaw — piecewise linear, asymmetric.
// The base servo's physical mounting is NOT symmetric about forward: the
// arm hits the mechanical limit at a different distance on each side of
// the operator's +Y reference. A single Z_BASE offset (the old approach)
// reads cleanly on one side and drifts on the other. Piecewise scale fixes
// that: ±90° operator yaw lands exactly on the joint's lo/hi stops,
// regardless of how off-center forward is.
//
//   BASE_FWD       = logical angle where arm physically points +Y forward.
//   BASE_RIGHT_MAX = logical angle of the right-side mechanical stop (high end).
//   BASE_LEFT_MAX  = logical angle of the  left-side mechanical stop (low  end).
//
// BASE_FWD was measured from the operator-saved "max Y" preset (Base=134°).
// The two MAX constants currently mirror the joint-table software limits
// (joints[0].lo / .hi); update them here if the physical stops sit inside
// those values.
constexpr float BASE_FWD       = 134.0f;
constexpr float BASE_RIGHT_MAX = 270.0f;
constexpr float BASE_LEFT_MAX  =   0.0f;

// Slopes: degrees of operator yaw per degree of base servo motion.
// 90° operator yaw ↔ (MAX − FWD) degrees of physical base rotation,
// so SCALE = 90 / range. Pre-computed at compile time.
constexpr float BASE_SCALE_RIGHT = 90.0f / (BASE_RIGHT_MAX - BASE_FWD);  // ≈ 0.662
constexpr float BASE_SCALE_LEFT  = 90.0f / (BASE_FWD - BASE_LEFT_MAX);   // ≈ 0.672

// Logical-base (servo deg) ↔ operator-yaw (deg) conversion. Branch on
// which side of forward we're on; each side uses its own slope. These two
// are inverses of each other across the full physical range.
static inline float baseLogicalToYaw(float logical) {
    return (logical >= BASE_FWD)
        ? (logical - BASE_FWD) * BASE_SCALE_RIGHT
        : (logical - BASE_FWD) * BASE_SCALE_LEFT;
}
static inline float baseYawToLogical(float yaw_deg) {
    return (yaw_deg >= 0.0f)
        ? BASE_FWD + yaw_deg / BASE_SCALE_RIGHT
        : BASE_FWD + yaw_deg / BASE_SCALE_LEFT;
}

// ── Workspace bias — shifts FK output so the operator's preferred reference
// pose reads as (X=0, Y=Y_max, Z=0, Rx=0, Ry=0, Rz=0). Decouples kinematic
// frame from operator frame; tune by snapshotting current FK output at the
// desired reference pose. Applied as a post-FK subtraction, so kinematic
// math (sin/cos chain) stays geometrically correct.
//
// Rx and Rz offsets are currently zero — at the operator's max-Y reference
// the wrist roll sits at Z_W_ROLL (Rx=0 by construction) and base sits at
// BASE_FWD (Rz=0 by piecewise mapping). They're parameterised so future
// reference-pose changes don't require touching the FK math.
constexpr float Z_OUT_OFFSET  = 150.0f;   // mm — subtracted from FK z
constexpr float RX_OUT_OFFSET =   0.0f;   // deg — subtracted from FK rx
constexpr float RY_OUT_OFFSET =  49.0f;   // deg — subtracted from FK ry
constexpr float RZ_OUT_OFFSET =   0.0f;   // deg — subtracted from FK rz

// ── Joint table ─────────────────────────────────────────────────────────────
// minUs/maxUs are physical pulse-width calibration; counts derived per-tick.
Joint joints[6] = {
    //  name           ch   lo   hi  home  cur   inv   minUs maxUs
    { "Base",           5,   0, 270, 129, 129,  false,  342, 2686 },
    { "Shoulder",       0,  30, 150,  62,  62,  true,   342, 2539 },
    { "Elbow",          1,   0, 180,  86,  86,  false,  342, 2686 },
    { "Wrist Pitch",    2,   0, 180,  86,  86,  false,  342, 2686 },
    { "Wrist Roll",     3,   0, 180,  86,  86,  false,  342, 2686 },
    { "Gripper",        4,   32,  90,   0,   0,  false,  342, 1514 },
};

// ── Smooth motion state
float servoCur[6]    = {129, 62, 86, 86, 86, 0};
float servoTarget[6] = {129, 62, 86, 86, 86, 0};
float motionSpeed    = 40.0f;         // deg/sec — ~2.25s for a 90° move (matches preset speed)

// Trapezoidal profile state: per-joint velocity, slewed at MOTION_ACCEL so
// moves ramp in AND out instead of starting/stopping at full speed (jerk =
// visible wobble on a desktop arm). 300°/s² reaches the 40°/s cruise in ~0.13s.
static float    servoVel[6]  = {0, 0, 0, 0, 0, 0};
constexpr float MOTION_ACCEL = 300.0f;   // deg/sec²

// Last PWM count written per joint. The engine writes whenever the *count*
// changes (~0.3–0.6°/count), not the whole degree — recovers the PWM's native
// resolution. 0xFFFF = unknown/stale, forces a write on the next tick.
static uint16_t lastCnt[6] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};

// ── Tool-tip extension (mm beyond wrist roll center, along tool approach axis).
// 111 mm = physical gripper length from the wrist roll motor face to the
// closed-jaw tip. Used by toolMode IK to back-project the input target onto
// the wrist roll center before the 2-link solver runs.
constexpr float TOOL_OFFSET = 111.0f;
bool toolMode = false;

// ── Straight-line motion (LINE)
constexpr float    LINE_STEP_MM = 10.0f;   // mm of Cartesian path per IK waypoint
constexpr uint32_t LINE_STEP_MS = 120;     // ms per waypoint — a FLOOR; settle gate paces
constexpr int      LINE_MAX_STEPS = 200;   // safety cap → 2000 mm path
constexpr float    LINE_SETTLE_DEG = 3.0f; // joints this close to target → next waypoint

struct LineState {
    float x0, y0, z0;        // start (mm) at startLine()
    float dx, dy, dz;        // mm per step
    float ry_deg;
    bool  fixed_ry;
    int   curStep;
    int   totalSteps;
    uint32_t nextMs;
};
static LineState line = {};
bool lineActive = false;

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
    { "Home",   1, { { { 129, 62, 86, 86, 86,  0 }, "" } } },
    { "Ready",  1, { { {  90, 60, 90, 90, 90, 45 }, "" } } },
    { "Pick",   1, { { {  90, 45, 45, 90, 90,  0 }, "" } } },
    { "Place",  1, { { {  90, 45, 45, 90, 90, 90 }, "" } } },
    { "Red",    0, {} },      // color sequences — taught at runtime (SQ:idx),
    { "Yellow", 0, {} },      // triggered by Favoriot MQTT / AMR HTTP routes
    { "Blue",   0, {} },
};

// ── Servo math ──────────────────────────────────────────────────────────────
uint16_t toCounts(const Joint& j, int angle) {
    long us = (long)j.minUs +
              ((long)(angle - j.lo) * ((long)j.maxUs - j.minUs)) / (j.hi - j.lo);
    long lo = min((long)j.minUs, (long)j.maxUs);
    long hi = max((long)j.minUs, (long)j.maxUs);
    return (uint16_t)(constrain(us, lo, hi) * 4096L / 20000L);
}

// Float-angle variant for the motion engine / IK paths. The PWM grid is
// ~4.88 µs/count ≈ 0.3–0.6°/count depending on the joint's µs span — rounding
// the angle to int degrees first (the old path) quantized every move to 1°.
static uint16_t toCountsF(const Joint& j, float angle) {
    float us = j.minUs + (angle - j.lo) * (float)(j.maxUs - j.minUs) / (float)(j.hi - j.lo);
    float lo = fminf(j.minUs, j.maxUs);
    float hi = fmaxf(j.minUs, j.maxUs);
    us = constrain(us, lo, hi);
    return (uint16_t)(us * 4096.0f / 20000.0f);
}

// Smooth path — UI/serial/playback/preset use this. processMotion picks it up.
// Float variant is the primary: IK/jog produce fractional degrees, and rounding
// them to int here re-introduced the 1° dead-zone the float pipeline removes.
void setServoF(uint8_t i, float angle) {
    servoTarget[i] = constrain(angle, (float)joints[i].lo, (float)joints[i].hi);
}
void setServo(uint8_t i, int angle) { setServoF(i, (float)angle); }

// Immediate path — startup, RAW, debug TEST. Bypasses motion engine entirely.
void setServoNow(uint8_t i, int angle) {
    angle = constrain(angle, joints[i].lo, joints[i].hi);
    joints[i].cur  = angle;
    servoCur[i]    = (float)angle;
    servoTarget[i] = (float)angle;
    servoVel[i]    = 0.0f;       // snap — discard any in-flight ramp velocity
    lastCnt[i]     = 0xFFFF;     // engine's count cache is stale after a direct write
    int phys = joints[i].invert ? (joints[i].lo + joints[i].hi - angle) : angle;
    if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        pca9685.setPWM(joints[i].ch, 0, toCounts(joints[i], phys));
        xSemaphoreGive(servoMutex);
    }
}

// Raw fast path — no mutex, no state sync. Only safe when caller owns the bus.
void sendPWM(uint8_t i, int angle) {
    joints[i].cur = angle;
    lastCnt[i]    = 0xFFFF;      // engine's count cache is stale after a raw write
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

    float dts   = dt * 0.001f;
    bool  moved = false;

    for (int i = 0; i < 6; i++) {
        float diff = servoTarget[i] - servoCur[i];
        float dist = fabsf(diff);
        if (dist < 0.02f && fabsf(servoVel[i]) < 1.0f) {
            servoCur[i] = servoTarget[i];
            servoVel[i] = 0.0f;
            continue;
        }

        // Trapezoidal profile: desired velocity is the cruise speed, capped by
        // sqrt(2·a·d) so it ramps DOWN as the target nears (decel starts at
        // v²/2a ≈ 2.7° out at 40°/s), and servoVel slews toward it at most
        // a·dt per tick so it ramps UP from standstill too. Direction flips
        // (jog reversal mid-move) decelerate through zero naturally.
        float dir   = (diff >= 0.0f) ? 1.0f : -1.0f;
        float vDes  = dir * fminf(motionSpeed, sqrtf(2.0f * MOTION_ACCEL * dist));
        float dvMax = MOTION_ACCEL * dts;
        servoVel[i] += constrain(vDes - servoVel[i], -dvMax, dvMax);
        servoCur[i] += servoVel[i] * dts;
        if ((servoTarget[i] - servoCur[i]) * dir <= 0.0f) {   // reached / crossed
            servoCur[i] = servoTarget[i];
            servoVel[i] = 0.0f;
        }

        joints[i].cur = (int)lroundf(servoCur[i]);   // logical readout (UI, recording)

        // Write on PWM-count granularity (~0.3–0.6°), not whole degrees.
        float phys = joints[i].invert
                   ? (float)(joints[i].lo + joints[i].hi) - servoCur[i]
                   : servoCur[i];
        uint16_t cnt = toCountsF(joints[i], phys);
        if (cnt == lastCnt[i]) continue;
        if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            pca9685.setPWM(joints[i].ch, 0, cnt);
            xSemaphoreGive(servoMutex);
            lastCnt[i] = cnt;
            moved = true;
        }
    }
    if (moved) pendingBroadcast = true;
}

// ── Forward kinematics ──────────────────────────────────────────────────────
// Planar 4-link chain (shoulder/elbow/wrist-pitch + wrist offset) in the arm's
// vertical plane, then rotated about Z by the base angle. Each pitch joint
// is measured from +Z, so total tool tilt = θ2 + θ3 + θ4. Wrist roll is on
// the tool axis — it changes orientation but not the wrist-center position.
Pose3D computeFK() {
    const float D2R = (float)M_PI / 180.0f;
    float t1 = baseLogicalToYaw(servoCur[0]) * D2R;
    float t2 = (servoCur[1] - Z_SHOULDER) * D2R;
    float t3 = (servoCur[2] - Z_ELBOW)    * D2R;
    float t4 = (servoCur[3] - Z_W_PITCH)  * D2R;
    float t5 = (servoCur[4] - Z_W_ROLL)   * D2R;

    float a23  = t2 + t3;
    float a234 = a23 + t4;
    float r = L2*sinf(t2) + L3*sinf(a23) + L4*sinf(a234);
    float z = L2*cosf(t2) + L3*cosf(a23) + L4*cosf(a234);

    Pose3D p;
    p.x  = r * sinf(t1);              // +X = right (base CW from above)
    p.y  = r * cosf(t1);              // +Y = forward (base θ1=0 → arm along +Y)
    p.z  = z  - Z_OUT_OFFSET;         // shifted: operator reference plane → Z=0
    p.rx = (t5 / D2R) - RX_OUT_OFFSET;                     // tool roll
    p.ry = (a234 / D2R) - RY_OUT_OFFSET;                   // tool pitch
    p.rz = baseLogicalToYaw(servoCur[0]) - RZ_OUT_OFFSET;  // base yaw (piecewise)
    return p;
}

// FK in the frame the operator is *commanding*: wrist roll center normally,
// gripper TIP when toolMode is on. solveIK interprets its input in exactly
// this frame, so every FK→IK round-trip (cart jog, LINE start point, status
// display) must use this — mixing frames commanded a point TOOL_OFFSET mm
// inside the real pose the instant tool mode went live.
Pose3D computeFKEffective() {
    Pose3D p = computeFK();
    if (!toolMode) return p;
    const float D2R = (float)M_PI / 180.0f;
    float a234 = (p.ry + RY_OUT_OFFSET) * D2R;   // tool tilt from +Z (internal frame)
    float t1   = atan2f(p.x, p.y);               // base azimuth from position
    float rExt = TOOL_OFFSET * sinf(a234);       // radial component of the tool vector
    p.x += rExt * sinf(t1);
    p.y += rExt * cosf(t1);
    p.z += TOOL_OFFSET * cosf(a234);
    return p;
}

void printFK() {
    Pose3D p = computeFKEffective();
    long x  = lroundf(p.x),  y  = lroundf(p.y),  z  = lroundf(p.z);
    long rx = lroundf(p.rx), ry = lroundf(p.ry), rz = lroundf(p.rz);

    static bool first = true;
    static long pX, pY, pZ, pRx, pRy, pRz;
    if (!first && x==pX && y==pY && z==pZ && rx==pRx && ry==pRy && rz==pRz) return;
    first = false;
    pX=x; pY=y; pZ=z; pRx=rx; pRy=ry; pRz=rz;

    Serial.printf("X: %ld mm, Y: %ld mm, Z: %ld mm | Rx: %ld, Ry: %ld, Rz: %ld\n",
        x, y, z, rx, ry, rz);
}

// ── Inverse kinematics (operator frame) ─────────────────────────────────────
// Inverts computeFK(): given a wrist-roll-center target (x, y, z) and tool
// pitch ry (all in operator coords, i.e. after the OUT_OFFSET biases), solve
// for joint angles. Strategy: peel off base yaw + L4 to get a 2D wrist-pitch
// target (r2, z2), then standard 2-link IK on L2/L3, then back out wrist
// pitch from the orientation constraint. Elbow-up only — second solution
// (elbow-down) omitted intentionally for predictability.
//
// Extended params:
//   rx_deg — if finite, wrist roll = Z_W_ROLL + rx_deg + RX_OUT_OFFSET (mod invert).
//            NaN = neutral wrist roll (default).
//   rz_deg — if finite, validates atan2(x,y) matches it within IK_RZ_TOL_DEG.
//            NaN = unconstrained (default). On this arm base yaw is fully
//            determined by (x,y) — Rz is redundant, so this is a consistency
//            check, not a degree of freedom.
//   toolMode (file-scope) — if true, (x,y,z) is the gripper TIP, not the
//            wrist roll center. Back-projects TOOL_OFFSET along the tool
//            approach axis. Requires fixed_ry to know that axis.
static constexpr float IK_RZ_TOL_DEG = 1.5f;

bool solveIK(float x, float y, float z, float ry_deg, bool fixed_ry,
             float rx_deg, float rz_deg, float out_logical[5]) {
    const float D2R = (float)M_PI / 180.0f;

    // Tool-tip → wrist-center back-projection. Tool approach axis lies in the
    // arm's vertical plane at azimuth atan2(x,y); its tilt from +Z is a234
    // (=ry_internal). Subtract TOOL_OFFSET projected onto the radial and
    // z directions. Requires fixed_ry — without it, a234 is determined by
    // the solver, creating a chicken-and-egg.
    if (toolMode) {
        if (!fixed_ry) {
            Serial.println("[IK] tool mode requires fixed Ry");
            return false;
        }
        float ryi_rad = (ry_deg + RY_OUT_OFFSET) * D2R;
        float r_tip   = sqrtf(x*x + y*y);
        if (r_tip < 0.01f) { Serial.println("[IK] tool tip on base axis"); return false; }
        float r_wrist = r_tip - TOOL_OFFSET * sinf(ryi_rad);
        float z_wrist = z     - TOOL_OFFSET * cosf(ryi_rad);
        float scale = r_wrist / r_tip;     // same azimuth, smaller radial
        x = x * scale;
        y = y * scale;
        z = z_wrist;
    }

    // Operator → internal frame (undo the workspace biases)
    float zi = z + Z_OUT_OFFSET;

    // Base yaw + radial; +X right, +Y forward → t1 = atan2(x, y)
    float t1 = atan2f(x, y);
    float r  = sqrtf(x*x + y*y);

    // Optional Rz consistency check — operator-supplied Rz must agree with
    // the implied base yaw (mechanical coupling on this arm).
    if (!isnan(rz_deg)) {
        float t1_deg = t1 / D2R;
        float diff   = rz_deg + RZ_OUT_OFFSET - t1_deg;
        while (diff >  180.0f) diff -= 360.0f;
        while (diff < -180.0f) diff += 360.0f;
        if (fabsf(diff) > IK_RZ_TOL_DEG) {
            Serial.printf("[IK] Rz=%.1f inconsistent with atan2(x,y)=%.1f (diff=%.1f > %.1f)\n",
                          rz_deg, t1_deg, diff, IK_RZ_TOL_DEG);
            return false;
        }
    }

    float t2, t3, t4;

    if (fixed_ry) {
        // 4-DOF: tool orientation locked to ry_deg. Step back L4 along the
        // tool axis to find the wrist-pitch joint position, then 2-link IK
        // on L2/L3 reaches that.
        float ryi_rad = (ry_deg + RY_OUT_OFFSET) * D2R;
        float r2 = r  - L4 * sinf(ryi_rad);
        float z2 = zi - L4 * cosf(ryi_rad);
        float d2    = r2*r2 + z2*z2;
        float cosT3 = (d2 - L2*L2 - L3*L3) / (2.0f * L2 * L3);
        // Swallow float dust at the workspace boundary (full extension /
        // full fold) — without this, LINE waypoints along max reach fail
        // spuriously. Genuine overreach still rejects.
        if (cosT3 >  1.0f) { if (cosT3 >  1.001f) return false; cosT3 =  1.0f; }
        if (cosT3 < -1.0f) { if (cosT3 < -1.001f) return false; cosT3 = -1.0f; }
        t3 = acosf(cosT3);                              // elbow-up
        float A = L2 + L3 * cosf(t3);
        float B = L3 * sinf(t3);
        t2 = atan2f(A*r2 - B*z2, B*r2 + A*z2);
        t4 = ryi_rad - t2 - t3;
    } else {
        // 3-DOF: wrist pitch pinned neutral (t4=0), so L4 collapses inline
        // with the forearm. Effective 2-link arm: L2 + (L3+L4). The free
        // DOF goes into reach instead of orientation — tool ends up wherever
        // the forearm points (Ry = t2 + t3).
        float Le    = L3 + L4;
        float d2    = r*r + zi*zi;
        float cosT3 = (d2 - L2*L2 - Le*Le) / (2.0f * L2 * Le);
        if (cosT3 >  1.0f) { if (cosT3 >  1.001f) return false; cosT3 =  1.0f; }
        if (cosT3 < -1.0f) { if (cosT3 < -1.001f) return false; cosT3 = -1.0f; }
        t3 = acosf(cosT3);                              // elbow-up
        float A = L2 + Le * cosf(t3);
        float B = Le * sinf(t3);
        t2 = atan2f(A*r - B*zi, B*r + A*zi);
        t4 = 0.0f;
    }

    // Wrist roll: caller-supplied Rx, or neutral. 1:1 mapping (no nonlinearity
    // expected on the WR joint).
    float wr_deg = isnan(rx_deg)
        ? Z_W_ROLL
        : (Z_W_ROLL + rx_deg + RX_OUT_OFFSET);

    // Float output — no rounding. The old int quantization made every solve
    // ±0.5°/joint, which turned small Cartesian jogs (<0.5° per joint) into
    // zero motion followed by 1° snaps.
    out_logical[0] = baseYawToLogical(t1 / D2R);
    out_logical[1] = t2 / D2R + Z_SHOULDER;
    out_logical[2] = t3 / D2R + Z_ELBOW;
    out_logical[3] = t4 / D2R + Z_W_PITCH;
    out_logical[4] = wr_deg;

    // 0.5° grace before rejecting, then clamp — parity with the old behavior
    // where lroundf() pulled boundary solutions like 180.4° back in range.
    for (int i = 0; i < 5; i++) {
        if (out_logical[i] < joints[i].lo - 0.5f || out_logical[i] > joints[i].hi + 0.5f) {
            Serial.printf("[IK] joint %d (%s) out of range: %.1f ∉ [%d, %d]\n",
                          i, joints[i].name, out_logical[i], joints[i].lo, joints[i].hi);
            return false;
        }
        out_logical[i] = constrain(out_logical[i], (float)joints[i].lo, (float)joints[i].hi);
    }
    return true;
}

void moveToXYZ(float x, float y, float z, float ry_deg, bool fixed_ry,
               float rx_deg, float rz_deg) {
    float s[5];
    if (!solveIK(x, y, z, ry_deg, fixed_ry, rx_deg, rz_deg, s)) {
        Serial.printf("[IK] unreachable: (%.0f,%.0f,%.0f)%s%s\n",
                      x, y, z,
                      fixed_ry ? " Ry=fixed" : "",
                      toolMode ? " tool=ON" : "");
        return;
    }
    Serial.printf("[IK] (%.0f,%.0f,%.0f%s%s%s%s) -> base=%.1f sh=%.1f el=%.1f wp=%.1f wr=%.1f\n",
                  x, y, z,
                  fixed_ry        ? " Ry" : "",
                  !isnan(rx_deg)  ? " Rx" : "",
                  !isnan(rz_deg)  ? " Rz" : "",
                  toolMode        ? " tool" : "",
                  s[0], s[1], s[2], s[3], s[4]);
    for (int i = 0; i < 5; i++) setServoF(i, s[i]);
    pendingBroadcast = true;
}

// ── Straight-line interpolator ──────────────────────────────────────────────
// Computes the current wrist position via FK, splits the segment to (xt,yt,zt)
// into LINE_STEP_MM chunks, and pushes one IK solution per LINE_STEP_MS tick
// into the motion engine. The engine ramps between consecutive waypoints, so
// the tool traces a polyline that approximates a straight line in Cartesian
// space.
//
// Cancellation: presets / playback / cycle stop the line implicitly via the
// guard in processLine(); call stopLine() to cancel from a handler.
bool startLine(float xt, float yt, float zt, float ry_deg, bool fixed_ry) {
    // Effective frame: tip when toolMode — must match what solveIK expects,
    // or the very first waypoint jumps TOOL_OFFSET mm off the real position.
    Pose3D p = computeFKEffective();
    float dx_t = xt - p.x, dy_t = yt - p.y, dz_t = zt - p.z;
    float dist = sqrtf(dx_t*dx_t + dy_t*dy_t + dz_t*dz_t);
    if (dist < 1.0f) { Serial.println("[LINE] already at target"); return false; }

    int steps = (int)ceilf(dist / LINE_STEP_MM);
    if (steps > LINE_MAX_STEPS) {
        Serial.printf("[LINE] path too long (%d steps > %d)\n", steps, LINE_MAX_STEPS);
        return false;
    }

    // Sanity-check the endpoint before moving — cheaper than failing mid-path.
    float s[5];
    if (!solveIK(xt, yt, zt, ry_deg, fixed_ry, NAN, NAN, s)) {
        Serial.println("[LINE] endpoint unreachable — aborting");
        return false;
    }

    line.x0 = p.x; line.y0 = p.y; line.z0 = p.z;
    line.dx = dx_t / steps;
    line.dy = dy_t / steps;
    line.dz = dz_t / steps;
    line.ry_deg     = ry_deg;
    line.fixed_ry   = fixed_ry;
    line.curStep    = 1;
    line.totalSteps = steps;
    line.nextMs     = millis();
    lineActive      = true;
    Serial.printf("[LINE] %d steps × %.1f mm -> (%.0f,%.0f,%.0f)\n",
                  steps, dist / steps, xt, yt, zt);
    return true;
}

void stopLine() {
    if (lineActive) Serial.println("[LINE] stopped");
    lineActive = false;
}

void processLine() {
    if (!lineActive) return;
    if (presetActive || isPlaying || isCycling) { stopLine(); return; }
    if (millis() < line.nextMs) return;

    // Don't outrun the motion engine: hand out the next waypoint only when
    // the joints are nearly at the previous one. 3° sits just above the
    // trapezoid's decel-onset distance (v²/2a ≈ 2.7° at 40°/s), so waypoints
    // blend at cruise speed instead of piling up and corner-cutting when a
    // step needs more joint motion than LINE_STEP_MS allows.
    for (int i = 0; i < 5; i++)
        if (fabsf(servoTarget[i] - servoCur[i]) > LINE_SETTLE_DEG) return;

    float x = line.x0 + line.dx * line.curStep;
    float y = line.y0 + line.dy * line.curStep;
    float z = line.z0 + line.dz * line.curStep;

    float s[5];
    if (!solveIK(x, y, z, line.ry_deg, line.fixed_ry, NAN, NAN, s)) {
        Serial.printf("[LINE] waypoint %d unreachable, stopping\n", line.curStep);
        lineActive = false;
        return;
    }
    for (int i = 0; i < 5; i++) setServoF(i, s[i]);
    pendingBroadcast = true;

    if (line.curStep >= line.totalSteps) {
        Serial.println("[LINE] Done");
        lineActive = false;
    } else {
        line.curStep++;
        line.nextMs = millis() + LINE_STEP_MS;
    }
}

// ── Cartesian jog engine ────────────────────────────────────────────────────
// Each tick: FK → offset by cartJog[] → IK → servoTarget[]. Rate-limited to
// ~50 Hz. If IK fails the arm simply stays put (no error flood). Ry is
// managed as a delta too, so the operator can tilt the tool while jogging.
// The jog speed is in mm/tick at full deflection (100); at 50 Hz, 2 mm/tick
// ≈ 100 mm/s Cartesian velocity — comfortable for a desktop arm.
constexpr float CART_JOG_SPEED_MM  = 2.0f;   // mm per tick at full deflection
constexpr float CART_JOG_SPEED_DEG = 1.5f;   // deg per tick at full Ry deflection
constexpr uint32_t CART_JOG_INTERVAL = 20;   // ms — 50 Hz

void processCartJog() {
    if (!cartMode) { cartJogActive = false; return; }
    if (isPlaying || isCycling || presetActive || lineActive) {
        cartJogActive = false; return;
    }

    static uint32_t lastMs = 0;
    if (millis() - lastMs < CART_JOG_INTERVAL) return;
    lastMs = millis();

    bool any = false;
    for (int i = 0; i < 5; i++) if (cartJog[i] != 0) { any = true; break; }
    cartJogActive = any;
    if (!any) return;

    // Current Cartesian position from FK — effective frame (tip in tool mode),
    // matching solveIK's interpretation of the target below.
    Pose3D p = computeFKEffective();

    // Apply deltas (normalised -100..100 → -1..1 × speed)
    float nx = p.x  + (cartJog[0] / 100.0f) * CART_JOG_SPEED_MM;
    float ny = p.y  + (cartJog[1] / 100.0f) * CART_JOG_SPEED_MM;
    float nz = p.z  + (cartJog[2] / 100.0f) * CART_JOG_SPEED_MM;
    float nry = p.ry + (cartJog[3] / 100.0f) * CART_JOG_SPEED_DEG;
    float nrx = p.rx + (cartJog[4] / 100.0f) * CART_JOG_SPEED_DEG;

    // Solve IK — fixed Ry so orientation tracks the jog. Rx is passed as
    // current+delta (not NAN): NAN means "neutral wrist roll" in solveIK,
    // which was silently re-centering the roll on every jog tick. This way
    // zero drx input HOLDS the current roll, nonzero steers it.
    float s[5];
    if (!solveIK(nx, ny, nz, nry, /*fixed_ry=*/true, nrx, NAN, s)) return;

    // Push to motion engine — float, so sub-degree jog increments aren't
    // rounded to zero motion
    for (int i = 0; i < 5; i++) setServoF(i, s[i]);
    pendingBroadcast = true;
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

// Color-preset wrappers — the Favoriot MQTT handler and the /api/run/* AMR
// routes trigger these by name.
void runRed()    { playPreset(PRESET_RED); }
void runYellow() { playPreset(PRESET_YELLOW); }
void runBlue()   { playPreset(PRESET_BLUE); }

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
