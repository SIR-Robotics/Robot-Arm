// ─── config.h — Hardware pins, timing, types ────────────────────────────────
// Pure constants and types shared by every module. No mutable state here.
#pragma once

#include <Arduino.h>

// ── PCA9685 / I²C
#define PCA9685_ADDR 0x40
#define PWM_FREQ_HZ  50
#define OSC_FREQ     27000000
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define I2C_SDA      21
#define I2C_SCL      22

// ── HW joystick + buttons (ADC1-safe pins only — see CLAUDE.md)
#define JOY_X_PIN     34
#define JOY_Y_PIN     35
#define JOY_SW_PIN    32
#define BTN_REC_PIN   33
#define BTN_PLAY_PIN  25
#define BTN_CLR_PIN   26
#define BTN_CYCLE_PIN 27

#define JOY_DEADZONE 300
#define JOY_SPEED    0.8f       // 0.8°/20ms = 40°/s, matches motionSpeed cap
#define JOY_INTERVAL 20
#define DEBOUNCE_MS  220

// ── Web jog smoothing
#define WEB_JOG_INTERVAL 25
#define WEB_JOG_SPEED    1.0f   // 1.0°/25ms = 40°/s, matches motionSpeed cap

// ── Recording
#define MAX_POSES    50
#define PLAY_STEP_MS 3000       // 3s/pose, enough for 120° move at 40°/s

// ── Staged motion (soft-home + preset moves)
#define SOFT_HOME_SETTLE_MS  1000   // pause between joints in setup() boot sequence
#define PRESET_SPEED_DEG_S   40.0f  // motionSpeed override while a preset is staging
#define PRESET_SETTLE_DEG    0.5f   // |target-cur| threshold for "phase done"

// ── Types
struct Joint {
    const char* name;
    uint8_t     ch;
    int         lo, hi;
    int         home;
    int         cur;
    bool        invert;
    uint16_t    minUs;   // µs at lo° (physical min pulse)
    uint16_t    maxUs;   // µs at hi° (physical max pulse)
};

struct Pose { int a[6]; char label[20]; };

#define MAX_PRESETS          4
#define MAX_POSES_PER_PRESET 30

struct Preset {
    char name[20];
    int  len;
    Pose seq[MAX_POSES_PER_PRESET];
};

struct WsMsg { char buf[96]; };
