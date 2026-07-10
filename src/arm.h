// ─── arm.h — Joint table, servo control, recording, playback ────────────────
#pragma once
#include "config.h"

// ── State (defined in arm.cpp)
extern Joint  joints[6];

// Smooth motion state
extern float  servoCur[6];              // physical position, advanced by processMotion
extern float  servoTarget[6];           // desired position — set by joystick/web/preset/playback
extern float  motionSpeed;              // deg/sec, defaults to MOTION_SPEED_DEG_S

extern Pose   seq[MAX_POSES];
extern int    seqLen;
extern bool   isPlaying;
extern bool   isCycling;
extern int    playIdx;

// Playback source — points at seq[] (live recording) or presets[i].seq.
// Set by startPlayback / startCycle / playPreset before kicking the engine.
extern Pose* playSrc;
extern int   playSrcLen;

// Sequence-capable preset slots. Each can hold up to MAX_POSES_PER_PRESET poses.
extern Preset presets[MAX_PRESETS];

// ── Servo
uint16_t toCounts(const Joint& j, int angle);
void     setServo   (uint8_t i, int angle);   // smooth — sets servoTarget[i]
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
void runRed();
void runYellow();
void runBlue();

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

// ── Forward kinematics
// Position = wrist roll motor center (NOT gripper tip — tool offset is applied
// separately by callers that need the tip, e.g. pick-and-place targeting).
struct Pose3D {
    float x, y, z;     // mm — origin = shoulder pivot, Z up, +Y = forward, +X = right
    float rx, ry, rz;  // deg — RPY: roll about tool, pitch from vertical, yaw about Z
};

Pose3D computeFK();    // reads servoCur[]
void   printFK();      // emits the spec'd serial line (only when value changed)
