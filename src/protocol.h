// ─── protocol.h — JSON builders, WebSocket, HTTP, and serial commands ──────
#pragma once
#include <ESPAsyncWebServer.h>

const char* buildStatus();
const char* buildPoses();
const char* buildPresets();
void        broadcastStatus();
void        broadcastPoses();
void        broadcastPresets();

void processWsCmd(char* msg);
void onWsEvent(AsyncWebSocket*, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len);

void registerHttpRoutes(AsyncWebServer& srv);   // GET/POST /poses.json
int  importPosesFromJson(const char* buf, size_t len);

void processSerial();
