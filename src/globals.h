// ─── globals.h — Singleton hardware/network handles ─────────────────────────
// Defined in main_new.cpp; declared here so every module can reach them
// without re-creating the buses.
#pragma once

#include <Adafruit_PWMServoDriver.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

extern Adafruit_PWMServoDriver pca9685;
extern AsyncWebServer          server;
extern AsyncWebSocket          ws;
extern Preferences             prefs;
extern SemaphoreHandle_t       servoMutex;
extern QueueHandle_t           wsQueue;

// Coalesced WS broadcast flag — set by anything that mutates state.
extern bool pendingBroadcast;
