// ─── input.h — HW joystick, buttons, and web-driven jog ─────────────────────
#pragma once
#include "config.h"

extern uint8_t joyMode;          // 0..2 — which joint pair the HW stick drives
extern bool    joyActive;
extern bool    webJogActive;
extern int     webJog[6];        // -100..+100 per joint, set by web/gamepad

// ── Cartesian (IK) jog — sticks drive XYZ + Ry instead of joint angles
extern bool    cartMode;          // false=joint jog, true=Cartesian jog
extern int     cartJog[4];        // -100..+100: [0]=dx, [1]=dy, [2]=dz, [3]=dry
extern bool    cartJogActive;

extern const uint8_t  PAIR[3][2];
extern const char*    PAIR_NAME[3];

bool calibrateJoystick();        // true = centers landed in sane range [1600,2500]
void processJoystick();
void processButtons();
void processWebJog();
