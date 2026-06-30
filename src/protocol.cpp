// ─── protocol.cpp — JSON, WebSocket, Serial ─────────────────────────────────
#include "protocol.h"
#include "config.h"
#include "arm.h"
#include "input.h"
#include "globals.h"
#include <WiFi.h>

// ── No-heap JSON buffers
static char statusBuf[300];
static char posesBuf[3000];

// ── JSON builders ───────────────────────────────────────────────────────────
const char* buildStatus() {
    Pose3D fk = computeFK();
    snprintf(statusBuf, sizeof(statusBuf),
        "{\"t\":\"s\",\"j\":[%d,%d,%d,%d,%d,%d],"
        "\"m\":%u,\"l\":%d,\"p\":%d,\"c\":%d,\"i\":%d,\"r\":%d,\"b\":%u,"
        "\"x\":%ld,\"y\":%ld,\"z\":%ld,\"rx\":%ld,\"ry\":%ld,\"rz\":%ld,"
        "\"ik\":%d}",
        joints[0].cur, joints[1].cur, joints[2].cur,
        joints[3].cur, joints[4].cur, joints[5].cur,
        (unsigned)joyMode, seqLen,
        isPlaying ? 1 : 0, isCycling ? 1 : 0, playIdx,
        (int)WiFi.RSSI(),
        (unsigned)bootState,
        lroundf(fk.x),  lroundf(fk.y),  lroundf(fk.z),
        lroundf(fk.rx), lroundf(fk.ry), lroundf(fk.rz),
        ikControlMode ? 1 : 0);
    return statusBuf;
}

const char* buildPoses() {
    char* p   = posesBuf;
    char* end = posesBuf + sizeof(posesBuf);
    p += snprintf(p, end - p, "{\"t\":\"p\",\"c\":%d,\"i\":[", seqLen);
    for (int i = 0; i < seqLen && (end - p) > 80; i++) {
        if (i) *p++ = ',';
        p += snprintf(p, end - p,
            "{\"a\":[%d,%d,%d,%d,%d,%d],\"n\":\"%s\"}",
            seq[i].a[0], seq[i].a[1], seq[i].a[2],
            seq[i].a[3], seq[i].a[4], seq[i].a[5],
            seq[i].label);
    }
    if ((end - p) >= 3) { *p++ = ']'; *p++ = '}'; *p = '\0'; }
    return posesBuf;
}

const char* buildPresets() {
    char* p   = posesBuf;
    char* end = posesBuf + sizeof(posesBuf);
    p += snprintf(p, end - p, "{\"t\":\"pl\",\"i\":[");
    for (int i = 0; i < MAX_PRESETS && (end - p) > 60; i++) {
        if (i) *p++ = ',';
        p += snprintf(p, end - p, "{\"n\":\"%s\",\"l\":%d}",
                      presets[i].name, presets[i].len);
    }
    if ((end - p) >= 3) { *p++ = ']'; *p++ = '}'; *p = '\0'; }
    return posesBuf;
}

void broadcastStatus() {
    if (ws.count() == 0) return;
    ws.textAll(buildStatus());
}
void broadcastPoses() {
    if (ws.count() == 0) return;
    ws.textAll(buildPoses());
}
void broadcastPresets() {
    if (ws.count() == 0) return;
    ws.textAll(buildPresets());
}

// ── WS protocol ─────────────────────────────────────────────────────────────
// Tags:
//   JG:j:v             single-axis jog  (-100..100)
//   JX:j1:v1:j2:v2     dual-axis jog (one XY stick → 2 joints in one frame)
//   SV:j:a             absolute servo set
//   MD:m               HW joystick pair (0..2)
//   PR:idx             play preset (0..MAX_PRESETS-1; 1-pose=staged, multi=playback)
//   SP:idx             save current pose into preset (collapses to 1-pose)
//   SQ:idx             save current recording into preset as a sequence
//   PN:idx:name        rename preset
//   RC                 record current pose into the live sequence
//   RN:p:name          rename pose in the live sequence
//   GT:p               goto pose (smooth ramp; no playback)
//   PY / ST / CY       play / stop / toggle cycle of live sequence
//   CL / SA / LD       clear / save / load live sequence
//   ID:dx:dy:dz:dry:drx IK delta jog (-100..100 per axis)
//   IK:[0|1]            toggle IK control mode (0=joint, 1=IK)
#define TAG(a,b) ((uint16_t)(((a)<<8) | (b)))

void processWsCmd(char* msg) {
    if (msg[0] == 0 || msg[1] == 0) return;
    uint16_t tag = TAG((uint8_t)msg[0], (uint8_t)msg[1]);

    char* args[5] = { nullptr, nullptr, nullptr, nullptr, nullptr };
    if (msg[2] == ':') {
        char* s = msg + 3;
        args[0] = s;
        for (int i = 1; i < 5 && (s = strchr(s, ':')) != nullptr; i++) {
            *s++ = '\0';
            args[i] = s;
        }
    }

    switch (tag) {
        case TAG('J','G'): {
            if (!args[0] || !args[1]) break;
            int j = atoi(args[0]), v = atoi(args[1]);
            if (j >= 0 && j < 6) webJog[j] = constrain(v, -100, 100);
            break;
        }
        case TAG('J','X'): {
            if (!args[0] || !args[1] || !args[2] || !args[3]) break;
            int j1 = atoi(args[0]), v1 = atoi(args[1]);
            int j2 = atoi(args[2]), v2 = atoi(args[3]);
            if (j1 >= 0 && j1 < 6) webJog[j1] = constrain(v1, -100, 100);
            if (j2 >= 0 && j2 < 6) webJog[j2] = constrain(v2, -100, 100);
            break;
        }
        case TAG('S','V'): {
            if (!args[0] || !args[1]) break;
            int j = atoi(args[0]), a = atoi(args[1]);
            if (j >= 0 && j < 6 && a >= 0) { setServo(j, a); pendingBroadcast = true; }
            break;
        }
        case TAG('P','R'): {                             // Play preset (1-pose staged or sequence)
            if (!args[0]) break;
            int idx = atoi(args[0]);
            if (idx >= 0 && idx < MAX_PRESETS) playPreset((uint8_t)idx);
            break;
        }
        case TAG('S','P'): {                             // Save current pose to preset (1-pose)
            if (!args[0]) break;
            int idx = atoi(args[0]);
            if (idx >= 0 && idx < MAX_PRESETS) {
                setPresetFromCurrent((uint8_t)idx);
                broadcastPresets();
            }
            break;
        }
        case TAG('S','Q'): {                             // Save current recording to preset (sequence)
            if (!args[0]) break;
            int idx = atoi(args[0]);
            if (idx < 0 || idx >= MAX_PRESETS || seqLen <= 0) break;
            int n = (seqLen < MAX_POSES_PER_PRESET) ? seqLen : MAX_POSES_PER_PRESET;
            for (int i = 0; i < n; i++) presets[idx].seq[i] = seq[i];
            presets[idx].len = n;
            savePresetsToFlash();
            broadcastPresets();
            Serial.printf("[PRESET %d] saved %d poses from recording\n", idx, n);
            break;
        }
        case TAG('P','N'): {                             // rename Preset Name
            if (!args[0] || !args[1] || !*args[1]) break;
            int idx = atoi(args[0]);
            if (idx >= 0 && idx < MAX_PRESETS) {
                renamePreset((uint8_t)idx, args[1]);
                broadcastPresets();
            }
            break;
        }
        case TAG('M','D'): {
            if (!args[0]) break;
            int m = atoi(args[0]);
            if (m >= 0 && m < 3) { joyMode = (uint8_t)m; pendingBroadcast = true; }
            break;
        }
        case TAG('R','C'): recordPose(); broadcastPoses(); break;
        case TAG('R','N'): {
            if (!args[0] || !args[1] || !*args[1]) break;
            int p = atoi(args[0]);
            if (p >= 0 && p < seqLen) {
                renamePose(p, args[1]);
                broadcastPoses();
                saveToFlash();
            }
            break;
        }
        case TAG('G','T'): {
            if (!args[0]) break;
            int p = atoi(args[0]);
            if (p >= 0 && p < seqLen) {
                int s[5];
                int* a = seq[p].a;
                int usedRy = 0;
                if (solveRecordedWaypoint(seq[p], s, &usedRy)) {
                    for (int i = 0; i < 5; i++) setServo(i, s[i]);
                    setServo(5, a[5]);
                    if (usedRy != a[3]) {
                        if (usedRy == 999) {
                            Serial.printf("[GT] Pose %d using position-only IK\n", p + 1);
                        } else {
                            Serial.printf("[GT] Pose %d relaxed Ry %d -> %d\n",
                                          p + 1, a[3], usedRy);
                        }
                    }
                } else {
                    Serial.printf("[GT] Pose %d unreachable FK=(%d,%d,%d Ry=%d Rx=%d)\n",
                                  p + 1, a[0], a[1], a[2], a[3], a[4]);
                }
                pendingBroadcast = true;
            }
            break;
        }
        case TAG('P','Y'): startPlayback(); break;
        case TAG('S','T'): stopPlayback();  break;
        case TAG('C','Y'): isCycling ? stopCycle() : startCycle(); break;
        case TAG('C','L'): clearRecording(); broadcastPoses(); break;
        case TAG('S','A'): saveToFlash(); break;
        case TAG('L','D'): loadFromFlash(); broadcastPoses(); break;
        case TAG('M','V'): {                             // MV:x:y:z[:ry]  IK move
            if (!args[0] || !args[1] || !args[2]) break;
            float x = atof(args[0]);
            float y = atof(args[1]);
            float z = atof(args[2]);
            bool fixed = (args[3] != nullptr);
            float ry = fixed ? atof(args[3]) : 0.0f;
            moveToXYZ(x, y, z, ry, fixed);
            break;
        }
        case TAG('I','D'): {                             // ID:dx:dy:dz:dry:drx  IK delta jog
            for (int i = 0; i < 5; i++) {
                ikWebJog[i] = (args[i] != nullptr) ? constrain(atoi(args[i]), -100, 100) : 0;
            }
            break;
        }
        case TAG('I','K'): {                             // IK:[0|1]  toggle IK control mode
            if (args[0]) {
                ikControlMode = (atoi(args[0]) != 0);
                if (!ikControlMode) {
                    for (int i = 0; i < 5; i++) ikWebJog[i] = 0;
                    ikWebJogActive = false;
                }
                Serial.printf("[IK] Control mode: %s\n", ikControlMode ? "IK (XYZ)" : "Joint");
                pendingBroadcast = true;
            }
            break;
        }
    }
}

// ── JSON poses import (forgiving parser, fixed-shape) ───────────────────────
int importPosesFromJson(const char* buf, size_t len) {
    const char* end = buf + len;
    const char* p   = buf;
    int newLen = 0;

    while (p < end && newLen < MAX_POSES) {
        const char* a = strstr(p, "\"a\"");
        if (!a || a >= end) break;
        const char* lb = strchr(a, '[');
        if (!lb || lb >= end) break;
        p = lb + 1;

        for (int k = 0; k < 6; k++) {
            while (p < end && (*p == ' ' || *p == ',' || *p == '\t' || *p == '\n' || *p == '\r')) p++;
            char* np = nullptr;
            long v = strtol(p, &np, 10);
            if (np == p) { v = 0; }
            else         { p = np; }
            seq[newLen].a[k] = (int)v;
        }

        seq[newLen].label[0] = '\0';
        const char* nField = strstr(p, "\"n\"");
        const char* nextA  = strstr(p, "\"a\"");
        if (nField && (!nextA || nField < nextA)) {
            const char* q = strchr(nField, ':');
            if (q) {
                q++;
                while (q < end && *q != '"') q++;
                if (q < end) {
                    q++;
                    int li = 0;
                    while (q < end && *q != '"' && li < (int)sizeof(seq[newLen].label) - 1)
                        seq[newLen].label[li++] = *q++;
                    seq[newLen].label[li] = '\0';
                    p = (q < end) ? q + 1 : end;
                }
            }
        }
        if (seq[newLen].label[0] == '\0')
            snprintf(seq[newLen].label, sizeof(seq[newLen].label), "Pose %d", newLen + 1);
        newLen++;
    }

    seqLen = newLen;
    isPlaying = false; isCycling = false; playIdx = 0;
    saveToFlash();
    pendingBroadcast = true;
    return newLen;
}

// ── HTTP routes: download/upload poses as JSON, run color presets ───────────
// GET  /poses.json — current sequence, downloaded as poses.json
// POST /poses.json — body is JSON, replaces sequence and persists to flash
// GET/POST /api/run/red|yellow|blue — execute recorded color preset sequence
static char    importBuf[3200];
static size_t  importLen = 0;

void registerHttpRoutes(AsyncWebServer& srv) {
    auto addRunRoute = [&](const char* path, const char* name, uint8_t idx, void (*fn)()) {
        auto handler = [name, idx, fn](AsyncWebServerRequest* req) {
            fn();
            char resp[80];
            snprintf(resp, sizeof(resp), "{\"ok\":true,\"preset\":\"%s\",\"len\":%d}",
                     name, presets[idx].len);
            req->send(200, "application/json", resp);
        };
        srv.on(path, HTTP_GET, handler);
        srv.on(path, HTTP_POST, handler);
    };

    addRunRoute("/api/run/red",    "red",    PRESET_RED,    runRed);
    addRunRoute("/api/run/yellow", "yellow", PRESET_YELLOW, runYellow);
    addRunRoute("/api/run/blue",   "blue",   PRESET_BLUE,   runBlue);

    srv.on("/poses.json", HTTP_GET, [](AsyncWebServerRequest* req){
        AsyncWebServerResponse* r = req->beginResponse(200, "application/json", buildPoses());
        r->addHeader("Content-Disposition", "attachment; filename=poses.json");
        req->send(r);
    });

    srv.on("/poses.json", HTTP_POST,
        [](AsyncWebServerRequest* req){
            importBuf[importLen] = '\0';
            int n = importPosesFromJson(importBuf, importLen);
            importLen = 0;
            char resp[64];
            snprintf(resp, sizeof(resp), "{\"ok\":true,\"n\":%d}", n);
            req->send(200, "application/json", resp);
            broadcastPoses();
        },
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total){
            if (index == 0) importLen = 0;
            if (importLen + len < sizeof(importBuf)) {
                memcpy(importBuf + importLen, data, len);
                importLen += len;
            }
        }
    );
}

void onWsEvent(AsyncWebSocket*, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        if (client->canSend()) client->text(buildStatus());
        if (client->canSend()) client->text(buildPoses());
        if (client->canSend()) client->text(buildPresets());
    } else if (type == WS_EVT_DISCONNECT) {
        for (int j = 0; j < 6; j++) webJog[j] = 0;
        for (int i = 0; i < 5; i++) ikWebJog[i] = 0;
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            WsMsg m; size_t n = min(len, sizeof(m.buf) - 1);
            memcpy(m.buf, data, n); m.buf[n] = '\0';
            xQueueSend(wsQueue, &m, 0);
        }
    }
}

// ── Serial command processor ────────────────────────────────────────────────
static const char* BOOT_NAMES[] = { "INIT", "HOMING", "IDLE", "BUSY", "FAULT" };

static void printHelp() {
    Serial.println();
    Serial.println(F("─── RoboArm serial — quick commands ───────────────────"));
    Serial.println(F("  0..6     play preset (4=Red 5=Yellow 6=Blue)"));
    Serial.println(F("  H        home (= preset 0)"));
    Serial.println(F("  R        record current pose"));
    Serial.println(F("  P        play / S stop / C cycle toggle"));
    Serial.println(F("  K        clear recording"));
    Serial.println(F("  V        save to flash / L load from flash"));
    Serial.println(F("  ?        this help"));
    Serial.println(F("─── Long form ─────────────────────────────────────────"));
    Serial.println(F("  STATUS              FSM state + joints + WiFi"));
    Serial.println(F("  PRESET <0-6>        same as the digit shortcut"));
    Serial.println(F("  RED/YELLOW/BLUE     run the matching color preset"));
    Serial.println(F("  RENAME <0-6> <nm>   rename a preset (UPPERCASE)"));
    Serial.println(F("  S <j> <a>           set joint j to angle a (smooth)"));
    Serial.println(F("  MOVE                show current XYZ + Rxyz"));
    Serial.println(F("  MOVE x y z          IK move (free Ry — wider reach)"));
    Serial.println(F("  MOVE x y z ry       IK move with tool pitch pinned"));
    Serial.println(F("  INVERT <j>          toggle joint direction"));
    Serial.println(F("  SPEED [deg/sec]     get/set motion engine speed"));
    Serial.println(F("  IKMODE [ON|OFF]     toggle IK control mode (XYZ vs joint jog)"));
    Serial.println(F("  TEST                lo→home→hi sweep per joint"));
    Serial.println(F("─── Calibration ───────────────────────────────────────"));
    Serial.println(F("  RAW <j> <counts>           direct PCA9685 write"));
    Serial.println(F("  SWEEP <j> <s> <e> [step]   1.5 s/step counts ramp"));
    Serial.println(F("  CALSHOW                    show calibration table"));
    Serial.println();
}

void processSerial() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); cmd.toUpperCase();
    if (cmd.length() == 0) return;

    // ── Single-char shortcuts ────────────────────────────────────────────
    if (cmd.length() == 1) {
        char c = cmd[0];
        switch (c) {
            case '?': printHelp();                                       return;
            case 'H': playPreset(0);                                     return;
            case 'R': recordPose();                                      return;
            case 'P': startPlayback();                                   return;
            case 'S': stopPlayback();                                    return;
            case 'C': isCycling ? stopCycle() : startCycle();            return;
            case 'K': clearRecording();                                  return;
            case 'V': saveToFlash();                                     return;
            case 'L': loadFromFlash();                                   return;
            case '0': case '1': case '2': case '3':
            case '4': case '5': case '6':
                playPreset((uint8_t)(c - '0'));                          return;
        }
        // Single letter that didn't match — fall through to unknown handler
    }

    if (cmd.startsWith("S ")) {
        int s1 = cmd.indexOf(' '), s2 = cmd.indexOf(' ', s1 + 1);
        if (s2 < 0) { Serial.println("Usage: S <joint 0-5> <angle>"); return; }
        int i = cmd.substring(s1 + 1, s2).toInt();
        int a = cmd.substring(s2 + 1).toInt();
        if (i < 0 || i >= 6) { Serial.println("Joint 0-5"); return; }
        setServo(i, a);
        Serial.printf("[%s] -> %d deg\n", joints[i].name, joints[i].cur);
        pendingBroadcast = true;
    }
    else if (cmd.startsWith("INVERT ")) {
        int j = cmd.substring(7).toInt();
        if (j >= 0 && j < 6) {
            joints[j].invert = !joints[j].invert;
            Serial.printf("[%s] invert = %s\n", joints[j].name, joints[j].invert ? "ON" : "OFF");
        }
    }
    else if (cmd == "TEST") {
        Serial.println("-- Servo channel test --");
        for (int i = 0; i < 6; i++) {
            Serial.printf("  %d %s ch%d  lo=%d hi=%d\n",
                i, joints[i].name, joints[i].ch, joints[i].lo, joints[i].hi);
            setServoNow(i, joints[i].lo);   delay(600);
            setServoNow(i, joints[i].home); delay(600);
            setServoNow(i, joints[i].hi);   delay(600);
            setServoNow(i, joints[i].home); delay(600);
        }
        pendingBroadcast = true;
    }
    else if (cmd.startsWith("IKMODE")) {
        if (cmd.length() <= 6) {
            Serial.printf("[IK] Control mode: %s\n", ikControlMode ? "IK (XYZ)" : "Joint");
        } else {
            String val = cmd.substring(7);
            val.trim(); val.toUpperCase();
            ikControlMode = (val == "ON" || val == "1");
            if (!ikControlMode) {
                for (int i = 0; i < 5; i++) ikWebJog[i] = 0;
                ikWebJogActive = false;
            }
            Serial.printf("[IK] Control mode: %s\n", ikControlMode ? "IK (XYZ)" : "Joint");
            pendingBroadcast = true;
        }
    }
    else if (cmd.startsWith("SPEED")) {
        if (cmd.length() <= 5) {
            Serial.printf("[MOTION] speed = %.1f deg/sec\n", motionSpeed);
        } else {
            float v = cmd.substring(6).toFloat();
            if (v >= 5.0f && v <= 1000.0f) {
                motionSpeed = v;
                Serial.printf("[MOTION] speed = %.1f deg/sec\n", motionSpeed);
            } else {
                Serial.println("Usage: SPEED <5..1000 deg/sec>");
            }
        }
    }
    else if (cmd.startsWith("PRESET")) {
        if (cmd.length() <= 6) {
            Serial.println("Usage: PRESET <0-6>");
        } else {
            int idx = cmd.substring(7).toInt();
            if (idx >= 0 && idx < MAX_PRESETS) playPreset((uint8_t)idx);
            else Serial.println("Preset 0-6");
        }
    }
    else if (cmd.startsWith("RENAME")) {
        // RENAME <idx> <name>  — note: cmd is already uppercased by processSerial
        int s1 = cmd.indexOf(' ');
        int s2 = cmd.indexOf(' ', s1 + 1);
        if (s1 < 0 || s2 < 0) {
            Serial.println("Usage: RENAME <0-6> <name>  (uppercase only via serial)");
        } else {
            int idx = cmd.substring(s1 + 1, s2).toInt();
            String nm = cmd.substring(s2 + 1);
            nm.trim();
            if (idx >= 0 && idx < MAX_PRESETS) {
                char buf[20]; nm.toCharArray(buf, sizeof(buf));
                renamePreset((uint8_t)idx, buf);
            } else {
                Serial.println("Preset 0-6");
            }
        }
    }
    else if (cmd == "STATUS") {
        Serial.printf("== FSM == %s", BOOT_NAMES[bootState]);
        if (bootState == STATE_FAULT) Serial.print("  (motion gated off)");
        else if (bootState == STATE_BUSY) Serial.printf("  (%s)",
            presetActive ? "preset" : isCycling ? "cycle" : "playback");
        Serial.println();
        Serial.println("-- Joints --");
        for (int i = 0; i < 6; i++)
            Serial.printf("  %d. %-14s ch%-2d  %3d deg  invert=%s\n",
                i, joints[i].name, joints[i].ch, joints[i].cur, joints[i].invert ? "Y" : "N");
        Serial.printf("  HW joy pair : %s\n", PAIR_NAME[joyMode]);
        Serial.printf("  Recording   : %d / %d\n", seqLen, MAX_POSES);
        Serial.printf("  Playing/Cyc : %s / %s\n", isPlaying ? "yes" : "no", isCycling ? "yes" : "no");
        Serial.printf("  WiFi RSSI   : %d dBm\n", (int)WiFi.RSSI());
    }
    else if (cmd.startsWith("RAW ")) {
        int s1 = cmd.indexOf(' '), s2 = cmd.indexOf(' ', s1 + 1);
        if (s2 < 0) { Serial.println("Usage: RAW <joint 0-5> <counts 0-4095>"); return; }
        int j = cmd.substring(s1 + 1, s2).toInt();
        int c = cmd.substring(s2 + 1).toInt();
        if (j < 0 || j >= 6) { Serial.println("Joint 0-5"); return; }
        c = constrain(c, 0, 4095);
        if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            pca9685.setPWM(joints[j].ch, 0, c);
            xSemaphoreGive(servoMutex);
        }
        Serial.printf("[RAW] %-12s ch%d  counts=%d  ~%lu us\n",
            joints[j].name, joints[j].ch, c, (unsigned long)c * 20000UL / 4096UL);
    }
    else if (cmd == "CALSHOW") {
        Serial.println("-- Calibration (50 Hz, 4096 counts/period) --");
        Serial.println("  #  Name           ch   lo   hi  minUs  maxUs  minCnt maxCnt");
        for (int i = 0; i < 6; i++) {
            uint16_t cLo = toCounts(joints[i], joints[i].lo);
            uint16_t cHi = toCounts(joints[i], joints[i].hi);
            Serial.printf("  %d  %-12s  %2d  %3d  %3d   %4u   %4u   %4u   %4u\n",
                i, joints[i].name, joints[i].ch,
                joints[i].lo, joints[i].hi,
                joints[i].minUs, joints[i].maxUs, cLo, cHi);
        }
        Serial.println("-- Live servo state --");
        Serial.println("  #  Name           ch  angle    us  counts");
        for (int i = 0; i < 6; i++) {
            int phys = joints[i].invert
                ? (joints[i].lo + joints[i].hi - joints[i].cur)
                : joints[i].cur;
            uint16_t c = toCounts(joints[i], phys);
            unsigned long us = (unsigned long)c * 20000UL / 4096UL;
            Serial.printf("  %d  %-12s  %2d  %3d deg  %4lu   %4u\n",
                i, joints[i].name, joints[i].ch, joints[i].cur, us, c);
        }
    }
    else if (cmd.startsWith("SWEEP ")) {
        // SWEEP <joint> <start_counts> <end_counts> [step]  — blocks 1.5 s per step
        int sp[4]; int ns = 0, p = 6;
        while (ns < 4) {
            int q = cmd.indexOf(' ', p);
            sp[ns++] = (q < 0 ? cmd.substring(p) : cmd.substring(p, q)).toInt();
            if (q < 0) break;
            p = q + 1;
        }
        if (ns < 3) { Serial.println("Usage: SWEEP <joint> <start> <end> [step=10]"); return; }
        int j = sp[0], st = sp[1], en = sp[2];
        int step = (ns >= 4 ? constrain(abs(sp[3]), 1, 512) : 10);
        if (j < 0 || j >= 6) { Serial.println("Joint 0-5"); return; }
        if (en < st) step = -step;
        st = constrain(st, 0, 4095); en = constrain(en, 0, 4095);
        Serial.printf("[SWEEP] %s ch%d  %d->%d  step=%d  1.5s/step\n",
            joints[j].name, joints[j].ch, st, en, step);
        for (int c = st; step > 0 ? c <= en : c >= en; c += step) {
            if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                pca9685.setPWM(joints[j].ch, 0, c);
                xSemaphoreGive(servoMutex);
            }
            Serial.printf("  counts=%4d  ~%4lu us  <- measure angle\n",
                c, (unsigned long)c * 20000UL / 4096UL);
            delay(1500);
        }
        Serial.println("[SWEEP] Done");
    }
    else if (cmd == "MOVE") {
        // No args — one-shot current FK readout (force-prints even if
        // printFK's change-detector is sitting on the same values).
        Pose3D p = computeFK();
        Serial.printf("X: %ld mm, Y: %ld mm, Z: %ld mm | Rx: %ld, Ry: %ld, Rz: %ld\n",
            lroundf(p.x),  lroundf(p.y),  lroundf(p.z),
            lroundf(p.rx), lroundf(p.ry), lroundf(p.rz));
    }
    else if (cmd.startsWith("MOVE ")) {
        // 3 args → free-Ry (wider reach); 4 args → fixed-Ry (tool orientation pinned)
        float v[4] = {0, 0, 0, 0};
        int n = 0, p = 5;
        while (n < 4) {
            int q = cmd.indexOf(' ', p);
            v[n++] = (q < 0 ? cmd.substring(p) : cmd.substring(p, q)).toFloat();
            if (q < 0) break;
            p = q + 1;
        }
        if (n < 3) {
            Serial.println("Usage: MOVE <x> <y> <z> [ry]   (4th arg = pin tool pitch)");
        } else {
            moveToXYZ(v[0], v[1], v[2], v[3], /*fixed_ry=*/ n >= 4);
        }
    }
    else if (cmd == "HELP") printHelp();
    else if (cmd == "RED")    runRed();
    else if (cmd == "YELLOW") runYellow();
    else if (cmd == "BLUE")   runBlue();
    // ── Long-form aliases for the legacy verbs (REC/PLAY/STOP/...) ─────────
    else if (cmd == "REC")    recordPose();
    else if (cmd == "PLAY")   startPlayback();
    else if (cmd == "STOP")   stopPlayback();
    else if (cmd == "CYCLE")  isCycling ? stopCycle() : startCycle();
    else if (cmd == "CLEAR")  clearRecording();
    else if (cmd == "SAVE")   saveToFlash();
    else if (cmd == "LOAD")   loadFromFlash();
    else {
        Serial.printf("Unknown: '%s'\n", cmd.c_str());
        printHelp();
    }
}
