// ─── input.h — HW joystick, buttons, and web-driven jog ─────────────────────
#pragma once
#include "config.h"

extern uint8_t joyMode;          // 0..2 — which joint pair the HW stick drives
extern bool    joyActive;
extern bool    webJogActive;
extern int     webJog[6];        // -100..+100 per joint, set by web/gamepad

extern const uint8_t  PAIR[3][2];
extern const char*    PAIR_NAME[3];

void calibrateJoystick();
void processJoystick();
void processButtons();
void processWebJog();
