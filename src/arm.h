// ─── arm.h — Joint table, servo control, recording, playback ────────────────
#pragma once
#include "config.h"

// ── State (defined in arm.cpp)
extern Joint  joints[6];

// Smooth motion state
extern float  servoCur[6];              // physical position, advanced by processMotion
extern float  servoTarget[6];           // desired position — set by joystick/web/preset/playback
extern float  motionSpeed;              // deg/sec, tunable via SPEED serial command

extern Pose   seq[MAX_POSES];
extern int    seqLen;
extern bool   isPlaying;
extern bool   isCycling;
extern int    playIdx;
extern uint32_t playNextMs;

// Playback source — points at seq[] (live recording) or presets[i].seq.
// Set by startPlayback / startCycle / playPreset before kicking the engine.
extern Pose* playSrc;
extern int   playSrcLen;

// Sequence-capable preset slots. Each can hold up to MAX_POSES_PER_PRESET poses.
extern Preset presets[MAX_PRESETS];

// ── Servo
uint16_t toCounts(const Joint& j, int angle);
void     setServo   (uint8_t i, int angle);   // smooth — sets servoTarget[i]
void     setServoF  (uint8_t i, float angle); // float variant — IK/jog paths (no 1° rounding)
void     setServoNow(uint8_t i, int angle);   // immediate I2C write (startup, RAW, TEST)
void     sendPWM    (uint8_t i, int angle);   // raw fast path (no mutex — caller owns bus)
void     moveToPose(const int p[6]);          // blocking, staged: Clearance → Elev → Align
void     moveToSafePosition();                // wrapper: targets joints[].home
void     moveToHomePose();                    // wrapper: targets presets[0] (Home)
bool     selfTestServoChannels();             // PCA9685 per-channel write/readback (true=all pass)
void     applyPreset(const int p[6]);         // staged + slow; uses motion engine
void     processMotion();                     // advances servoCur -> servoTarget every tick
void     processPresetMove();                 // drives the async preset state machine
extern bool presetActive;                     // true while a staged preset is in flight

// ── Teachable presets
void setPresetFromCurrent(uint8_t idx);       // 0=home 1=ready 2=pick 3=place
void renamePreset(uint8_t idx, const char* name);  // writes through to flash
void savePresetsToFlash();
void loadPresetsFromFlash();

// ── Recording / playback
void recordPose();
void renamePose(int idx, const char* name);
void startPlayback();
void stopPlayback();
void startCycle();
void stopCycle();
void clearRecording();
void saveToFlash();
void loadFromFlash();
void processPlayback();
void playPreset(uint8_t idx);                 // 1-pose → staged move; multi-pose → playback
void runRed();                                // color wrappers — Favoriot MQTT + /api/run/*
void runYellow();
void runBlue();

// ── Forward kinematics
// Position = wrist roll motor center (NOT gripper tip — tool offset is applied
// separately by callers that need the tip, e.g. pick-and-place targeting).
struct Pose3D {
    float x, y, z;     // mm — origin = shoulder pivot, Z up, +Y = forward, +X = right
    float rx, ry, rz;  // deg — RPY: roll about tool, pitch from vertical, yaw about Z
};

Pose3D computeFK();    // reads servoCur[]; always wrist roll center
Pose3D computeFKEffective(); // = computeFK(), but gripper TIP when toolMode is on —
                             // the frame solveIK expects, so FK→IK round-trips
                             // (cart jog, LINE, status) stay consistent
void   printFK();      // emits the spec'd serial line (only when value changed)

// ── Tool-tip mode ───────────────────────────────────────────────────────────
// When true, solveIK / moveToXYZ interpret the (x,y,z) input as the gripper
// TIP position; the IK back-projects TOOL_OFFSET mm along the tool approach
// axis to get the wrist roll center, then solves normally. Requires fixed_ry
// (the approach axis is determined by ry); free-Ry calls are rejected.
// Default OFF — backwards-compatible with all existing IK callers.
extern bool toolMode;

// IK: solve joints for an operator-frame target. Returns false if unreachable
// or if any joint would exceed its [lo, hi]. out_logical[0..4] = base, shoulder,
// elbow, wrist pitch, wrist roll (logical degrees, ready for setServo).
//   fixed_ry=true  → constrain tool pitch to ry_deg (uses up the 4th DOF)
//   fixed_ry=false → wrist pitch held neutral (t4=0); ry_deg is ignored;
//                    reach is wider because L4 collapses into a 2-link arm.
// rx_deg/rz_deg: if NaN, wrist roll defaults to neutral and Rz is unconstrained.
// If finite, wrist roll = Z_W_ROLL + rx_deg (after RX_OUT_OFFSET), and Rz must
// match the implied atan2(x,y) within tolerance — else the solve fails.
bool solveIK(float x, float y, float z, float ry_deg, bool fixed_ry,
             float rx_deg, float rz_deg, float out_logical[5]);
void moveToXYZ(float x, float y, float z, float ry_deg, bool fixed_ry,
               float rx_deg = NAN, float rz_deg = NAN);

// ── Straight-line motion (LINE) ─────────────────────────────────────────────
// Drives a Cartesian-linear path from current wrist position to (xt,yt,zt) by
// IK-solving waypoints at ~LINE_STEP_MM spacing and feeding the motion engine
// one waypoint at a time. processLine() advances when the previous waypoint
// settles. Cancelled by any preset/playback start.
extern bool lineActive;
bool startLine(float xt, float yt, float zt, float ry_deg, bool fixed_ry);
void processLine();
void stopLine();

// ── Cartesian jog ───────────────────────────────────────────────────────────
// Each tick: read current FK → offset by cartJog[] scaled by CART_JOG_SPEED ×
// dt → solve IK → push to servoTarget[]. No-op if IK fails (arm stays put).
void processCartJog();
