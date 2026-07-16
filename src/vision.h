#pragma once
// ─── vision.h ───────────────────────────────────────────────────────────────
// Autonomous HuskyLens tag detection. Self-contained: doesn't touch
// servoMutex/wsQueue or any arm state, so it can be added/removed without
// affecting arm.cpp / input.cpp / protocol.cpp.
//
// Wiring (HuskyLens dial set to UART, default baud 9600):
//   HuskyLens TX -> ESP32 GPIO16 (RX2)
//   HuskyLens RX -> ESP32 GPIO17 (TX2)
//
// IMPORTANT: this project already uses I2C (Wire.begin(I2C_SDA, I2C_SCL))
// for the PCA9685, plus GPIOs for JOY_SW_PIN / BTN_*_PIN. Check config.h
// before wiring HuskyLens up and change VISION_UART_RX / VISION_UART_TX
// below if GPIO16/17 are already claimed.
 
#include <Arduino.h>
 
// Call once from setup(). Non-fatal: if HuskyLens isn't found within a
// short timeout, vision stays disabled and returns false, but the rest of
// the arm keeps running normally.
bool visionInit();

// Call every loop() tick. A learned tag waits 10 seconds, then triggers its
// matching preset without WiFi/MQTT. Internally rate-limited, so it's cheap to
// call unconditionally even when bootState == STATE_FAULT.
void visionPoll();

// Last tag the camera saw: its ID (>= 0), or -1 when nothing has been seen
// for VISION_TAG_HOLD_MS — so the UI indicator self-clears instead of
// showing a stale detection forever.
int visionCurrentTag();
