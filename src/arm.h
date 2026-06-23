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

// Mutable so user can re-teach them from the current arm position.
extern int POSE_HOME [6];
extern int POSE_READY[6];
extern int POSE_PICK [6];
extern int POSE_PLACE[6];

// ── Servo
uint16_t toCounts(const Joint& j, int angle);
void     setServo   (uint8_t i, int angle);   // smooth — sets servoTarget[i]
void     setServoNow(uint8_t i, int angle);   // immediate I2C write (startup, RAW, TEST)
void     sendPWM    (uint8_t i, int angle);   // raw fast path (no mutex — caller owns bus)
void     moveToHome();
void     applyPreset(const int p[6]);
void     processMotion();                     // advances servoCur -> servoTarget every tick

// ── Teachable presets
void setPresetFromCurrent(uint8_t idx);       // 0=home 1=ready 2=pick 3=place
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
