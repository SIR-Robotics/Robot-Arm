// ─── input.h — HW joystick, buttons, and web-driven jog ─────────────────────
#pragma once
#include "config.h"

extern uint8_t joyMode;          // 0..2 — which joint pair the HW stick drives
extern bool    joyActive;
extern bool    webJogActive;
extern int     webJog[6];        // -100..+100 per joint, set by web/gamepad

// IK-mode web jog: world-frame deltas sent by browser sticks/keyboard
extern int     ikWebJog[5];      // dx, dy, dz, dry, drx  (-100..100)
extern bool    ikWebJogActive;

extern const uint8_t  PAIR[3][2];
extern const char*    PAIR_NAME[3];

bool calibrateJoystick();        // true = centers landed in sane range [1600,2500]
void processJoystick();
void processButtons();
void processWebJog();
void processIkWebJog();          // IK-mode web jog handler
