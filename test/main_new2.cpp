// ─── RoboArm 6-DOF Controller ───────────────────────────────────────────────
// Optimized build. Notable choices:
//   - WS incoming uses a tiny "TAG:arg:arg" text protocol; no JSON, no String
//   - WS outgoing JSON built with snprintf into fixed buffers (no heap churn)
//   - Joint angle → PCA9685 counts via a 181-entry lookup table
//   - Servo writes protected by an I2C mutex
//   - All ISR-style work (network) decoupled from servo via FreeRTOS queue
//   - HTML/CSS/JS minified to cut response size and RAM pressure during send
//   - Playback/idle modes governed by a single-variable FSM (ArmState)

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>

// ─── PCA9685 ────────────────────────────────────────────────────────────────
#define PCA9685_ADDR 0x40
#define PWM_FREQ_HZ  50
#define OSC_FREQ     27000000
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define I2C_SDA      21
#define I2C_SCL      22

// ─── WiFi (STA mode) ────────────────────────────────────────────────────────
const char* WIFI_SSID = "ASEM Training";
const char* WIFI_PASS = "Class@Asem";

// ─── Hardware joystick & buttons (ADC1 only — ADC2 conflicts with WiFi) ─────
#define JOY_X_PIN     34
#define JOY_Y_PIN     35
#define JOY_SW_PIN    32   // cycle joint pair
#define BTN_REC_PIN   33
#define BTN_PLAY_PIN  25
#define BTN_CLR_PIN   26
#define BTN_CYCLE_PIN 27

#define JOY_DEADZONE  300
#define JOY_SPEED     1.2f
#define JOY_INTERVAL  20
#define DEBOUNCE_MS   220

const uint8_t PAIR[3][2]   = { {0,1}, {2,3}, {4,5} };
const char*   PAIR_NAME[3] = { "Base+Shoulder", "Elbow+Wrist Pitch", "Wrist Roll+Gripper" };

// ─── Web touch joystick ─────────────────────────────────────────────────────
#define WEB_JOG_INTERVAL 25
#define WEB_JOG_SPEED    2.0f

// ─── Recording ──────────────────────────────────────────────────────────────
#define MAX_POSES    50
#define PLAY_STEP_MS 1200

struct Pose { int a[6]; char label[20]; };

Pose     seq[MAX_POSES];
int      seqLen     = 0;
int      playIdx    = 0;
uint32_t playNextMs = 0;

// ─── FSM ────────────────────────────────────────────────────────────────────
// The arm is always in exactly one of these three states.
//
//         startPlayback()          startCycle()
//   IDLE ─────────────────► PLAYING ─────────────► CYCLING
//     ▲   stopPlayback()      │  startPlayback()      │
//     │◄──────────────────────┘◄──────────────────────┘
//     │        stopCycle() / stopPlayback()
//     └── clearRecording() forces IDLE from any state
//
enum ArmState { ARM_IDLE, ARM_PLAYING, ARM_CYCLING };
ArmState armState = ARM_IDLE;

// ─── Joint table ────────────────────────────────────────────────────────────
struct Joint {
    const char* name;
    uint8_t     ch;
    int         lo, hi;
    int         home;
    int         cur;
    bool        invert;
};

Joint joints[6] = {
    { "Base",        5,   0, 180,  90,  90, false },
    { "Shoulder",    0,  30, 150,  90,  90, true  },
    { "Elbow",       1,   0, 135,  90,  90, false },
    { "Wrist Pitch", 2,   0, 180,  90,  90, false },
    { "Wrist Roll",  3,   0, 180,  90,  90, false },
    { "Gripper",     4,   0,  90,  45,  45, false },
};

const int POSE_HOME [6] = { 90, 90, 90, 90, 90, 45 };
const int POSE_READY[6] = { 90, 60, 90, 90, 90, 45 };
const int POSE_PICK [6] = { 90, 45, 45, 90, 90,  0 };
const int POSE_PLACE[6] = { 90, 45, 45, 90, 90, 90 };

// ─── Globals ────────────────────────────────────────────────────────────────
Adafruit_PWMServoDriver pca9685(PCA9685_ADDR);
AsyncWebServer  server(80);
AsyncWebSocket  ws("/ws");
Preferences     prefs;
SemaphoreHandle_t servoMutex;

float    joyF[6];
uint8_t  joyMode     = 0;
uint8_t  moveSpeed   = 5;   // 1 = slowest, 10 = fastest
int      joyXCenter = 2048;
int      joyYCenter = 2048;
bool     joyActive  = false;
uint32_t joyLastMs  = 0;
uint32_t lastBroadMs = 0;

int      webJog[6]    = {0,0,0,0,0,0};
float    webJogAcc[6] = {0,0,0,0,0,0};
uint32_t lastWebJogMs = 0;
bool     webJogActive = false;

uint32_t lastSW=0, lastRec=0, lastPlay=0, lastClr=0, lastCycle=0;

struct WsMsg { char buf[96]; };
QueueHandle_t wsQueue;
bool pendingBroadcast = false;

// PWM lookup: angle (0..180) → PCA9685 counts. Initialized in setup().
uint16_t pwmTable[181];

// Static buffers for outgoing JSON — never alloc on the broadcast hot path
static char statusBuf[200];
static char posesBuf[3000];   // ~50 poses * ~50 chars + headers; fits comfortably

// ─── FSM transition ──────────────────────────────────────────────────────────
static const char* stateName(ArmState s) {
    switch (s) {
        case ARM_IDLE:    return "IDLE";
        case ARM_PLAYING: return "PLAYING";
        case ARM_CYCLING: return "CYCLING";
        default:          return "?";
    }
}

void transitionTo(ArmState next) {
    if (next == armState) return;
    Serial.printf("[FSM] %s → %s\n", stateName(armState), stateName(next));
    armState = next;
    pendingBroadcast = true;
}

// ─── Servo helpers ──────────────────────────────────────────────────────────
inline uint16_t toCounts(int a) {
    return pwmTable[a < 0 ? 0 : (a > 180 ? 180 : a)];
}

void setServo(uint8_t i, int angle) {
    angle = constrain(angle, joints[i].lo, joints[i].hi);
    joints[i].cur = angle;
    joyF[i]       = (float)angle;
    int phys = joints[i].invert ? (joints[i].lo + joints[i].hi - angle) : angle;
    if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        pca9685.setPWM(joints[i].ch, 0, toCounts(phys));
        xSemaphoreGive(servoMutex);
    }
}

// Loop-only fast path: no mutex (caller already in loop task), no joyF sync.
inline void sendPWM(uint8_t i, int angle) {
    joints[i].cur = angle;
    int phys = joints[i].invert ? (joints[i].lo + joints[i].hi - angle) : angle;
    pca9685.setPWM(joints[i].ch, 0, toCounts(phys));
}

// ─── Speed helpers ───────────────────────────────────────────────────────────
// moveSpeed 1–10 maps to interpolation parameters.
// steps×ms gives total move duration: ~1080 ms (slow) → ~60 ms (fast).
static inline uint8_t speedSteps() { return (uint8_t)map(moveSpeed, 1, 10, 60, 15); }
static inline uint8_t speedMs()    { return (uint8_t)map(moveSpeed, 1, 10, 18,  4); }

// Single-joint smooth move driven by moveSpeed.
void smoothMove(uint8_t i, int target) {
    int start = joints[i].cur;
    target = constrain(target, joints[i].lo, joints[i].hi);
    uint8_t steps = speedSteps(), ms = speedMs();
    for (uint8_t s = 1; s <= steps; s++) {
        setServo(i, start + (target - start) * s / steps);
        delay(ms);
    }
}

// All-joints simultaneous smooth move — all joints step in lock-step so total
// duration is steps×ms, not 6× that.
void smoothMoveAll(const int targets[6]) {
    int starts[6];
    for (int i = 0; i < 6; i++) starts[i] = joints[i].cur;
    uint8_t steps = speedSteps(), ms = speedMs();
    for (uint8_t s = 1; s <= steps; s++) {
        for (int i = 0; i < 6; i++)
            setServo(i, starts[i] + (targets[i] - starts[i]) * s / steps);
        delay(ms);
    }
}

void moveToHome()                { for (int i=0;i<6;i++) setServo(i, joints[i].home); }
void applyPreset(const int p[6]) { smoothMoveAll(p); }

// ─── Outgoing JSON (no heap) ────────────────────────────────────────────────
const char* buildStatus() {
    snprintf(statusBuf, sizeof(statusBuf),
        "{\"t\":\"s\",\"j\":[%d,%d,%d,%d,%d,%d],\"m\":%u,\"l\":%d,\"p\":%d,\"c\":%d,\"i\":%d,\"sp\":%d}",
        joints[0].cur, joints[1].cur, joints[2].cur,
        joints[3].cur, joints[4].cur, joints[5].cur,
        (unsigned)joyMode, seqLen,
        (armState == ARM_PLAYING) ? 1 : 0,
        (armState == ARM_CYCLING) ? 1 : 0,
        playIdx, (int)moveSpeed);
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

inline void broadcastStatus() { ws.textAll(buildStatus()); }
inline void broadcastPoses()  { ws.textAll(buildPoses());  }

// ─── Recording ──────────────────────────────────────────────────────────────
void recordPose() {
    if (seqLen >= MAX_POSES) { Serial.println("[REC] Full"); return; }
    for (int i = 0; i < 6; i++) seq[seqLen].a[i] = joints[i].cur;
    snprintf(seq[seqLen].label, sizeof(seq[seqLen].label), "Pose %d", seqLen + 1);
    seqLen++;
    Serial.printf("[REC] %d total\n", seqLen);
    pendingBroadcast = true;
}

void renamePose(int idx, const char* name) {
    if (idx < 0 || idx >= seqLen) return;
    strncpy(seq[idx].label, name, sizeof(seq[idx].label) - 1);
    seq[idx].label[sizeof(seq[idx].label) - 1] = '\0';
}

void startPlayback() {
    if (!seqLen) { Serial.println("[PLAY] Empty"); return; }
    transitionTo(ARM_PLAYING);
    playIdx = 0; playNextMs = millis();
}

void stopPlayback() {
    transitionTo(ARM_IDLE);
}

void startCycle() {
    if (!seqLen) { Serial.println("[CYCLE] Empty"); return; }
    transitionTo(ARM_CYCLING);
    playIdx = 0; playNextMs = millis();
}

void stopCycle() {
    transitionTo(ARM_IDLE);
}

void clearRecording() {
    seqLen = 0;
    transitionTo(ARM_IDLE);
    pendingBroadcast = true;  // seqLen changed regardless of prior state
    Serial.println("[REC] Cleared");
}

void saveToFlash() {
    prefs.begin("roboarm", false);
    prefs.putInt("len", seqLen);
    if (seqLen > 0) prefs.putBytes("seq", seq, seqLen * sizeof(Pose));
    prefs.end();
    Serial.printf("[FLASH] Saved %d\n", seqLen);
}

void loadFromFlash() {
    prefs.begin("roboarm", true);
    seqLen = prefs.getInt("len", 0);
    if (seqLen > 0) prefs.getBytes("seq", seq, seqLen * sizeof(Pose));
    prefs.end();
    Serial.printf("[FLASH] Loaded %d\n", seqLen);
    pendingBroadcast = true;
}

// ─── Hardware joystick ──────────────────────────────────────────────────────
void calibrateJoystick() {
    long xs = 0, ys = 0;
    const int N = 64;
    for (int i = 0; i < N; i++) {
        xs += analogRead(JOY_X_PIN);
        ys += analogRead(JOY_Y_PIN);
        delay(5);
    }
    joyXCenter = xs / N;
    joyYCenter = ys / N;
    Serial.printf("[JOY] Center X=%d Y=%d\n", joyXCenter, joyYCenter);
}

void processJoystick() {
    if (millis() - joyLastMs < JOY_INTERVAL) return;
    joyLastMs = millis();

    int xRaw = analogRead(JOY_X_PIN);
    int yRaw = analogRead(JOY_Y_PIN);

    float xD = 0, yD = 0;
    if (abs(xRaw - joyXCenter) > JOY_DEADZONE)
        xD = (xRaw - joyXCenter) / (float)joyXCenter * JOY_SPEED;
    if (abs(yRaw - joyYCenter) > JOY_DEADZONE)
        yD = (yRaw - joyYCenter) / (float)joyYCenter * JOY_SPEED;

    uint8_t jA = PAIR[joyMode][0];
    uint8_t jB = PAIR[joyMode][1];
    joyActive = (xD != 0.0f || yD != 0.0f);

    if (xD != 0.0f) {
        joyF[jA] = constrain(joyF[jA] + xD, (float)joints[jA].lo, (float)joints[jA].hi);
        int a = (int)joyF[jA];
        if (a != joints[jA].cur) sendPWM(jA, a);
    }
    if (yD != 0.0f) {
        joyF[jB] = constrain(joyF[jB] + yD, (float)joints[jB].lo, (float)joints[jB].hi);
        int b = (int)joyF[jB];
        if (b != joints[jB].cur) sendPWM(jB, b);
    }
}

bool btnPressed(uint8_t pin, uint32_t& last) {
    if (digitalRead(pin) == LOW && millis() - last > DEBOUNCE_MS) {
        last = millis(); return true;
    }
    return false;
}

void processButtons() {
    if (btnPressed(JOY_SW_PIN,    lastSW))    { joyMode=(joyMode+1)%3; Serial.printf("[JOY] %s\n",PAIR_NAME[joyMode]); pendingBroadcast=true; }
    if (btnPressed(BTN_REC_PIN,   lastRec))   recordPose();
    if (btnPressed(BTN_PLAY_PIN,  lastPlay))  armState == ARM_PLAYING ? stopPlayback() : startPlayback();
    if (btnPressed(BTN_CLR_PIN,   lastClr))   clearRecording();
    if (btnPressed(BTN_CYCLE_PIN, lastCycle)) armState == ARM_CYCLING ? stopCycle() : startCycle();
}

// ─── Web touch jog ──────────────────────────────────────────────────────────
void processWebJog() {
    if (armState != ARM_IDLE) { webJogActive = false; return; }
    if (millis() - lastWebJogMs < WEB_JOG_INTERVAL) return;
    lastWebJogMs = millis();

    bool moved = false, anyActive = false;
    for (int j = 0; j < 6; j++) {
        if (webJog[j] == 0) { webJogAcc[j] = 0; continue; }
        anyActive = true;
        webJogAcc[j] += (webJog[j] / 100.0f) * WEB_JOG_SPEED;
        int delta = (int)webJogAcc[j];
        if (delta == 0) continue;
        int target = constrain(joints[j].cur + delta, joints[j].lo, joints[j].hi);
        if (target == joints[j].cur) { webJogAcc[j] = 0; continue; }
        webJogAcc[j] -= (target - joints[j].cur);
        sendPWM(j, target);
        joyF[j] = (float)target;
        moved = true;
    }
    webJogActive = anyActive;
    if (moved) pendingBroadcast = true;
}

void processPlayback() {
    if (armState == ARM_IDLE) return;
    if (millis() < playNextMs)    return;

    if (playIdx >= seqLen) {
        if (armState == ARM_CYCLING) { playIdx = 0; }
        else {
            transitionTo(ARM_IDLE);
            playIdx = 0;
            Serial.println("[PLAY] Done");
            return;
        }
    }

    smoothMoveAll(seq[playIdx].a);
    Serial.printf("[%s] %d/%d \"%s\"\n",
        armState == ARM_CYCLING ? "CYC" : "PLAY", playIdx + 1, seqLen, seq[playIdx].label);
    playIdx++;
    playNextMs = millis() + PLAY_STEP_MS;
    pendingBroadcast = true;
}

// ─── WS command parser (text protocol, zero-alloc) ──────────────────────────
// Format: "XX" or "XX:a1" or "XX:a1:a2" where XX is a 2-letter tag.
//   JG:j:v        jog (-100..100)
//   SV:j:a        set servo
//   PR:name       preset (home/ready/pick/place)
//   MD:m          joystick mode (0..2)
//   SP:v          move speed (1-10)
//   RC            record
//   RN:p:name     rename pose
//   GT:p          goto pose
//   PY            play
//   ST            stop
//   CY            cycle toggle
//   CL            clear
//   SA            save
//   LD            load

#define TAG(a,b) ((uint16_t)(((a)<<8) | (b)))

void processWsCmd(char* msg) {
    if (msg[0] == 0 || msg[1] == 0) return;
    uint16_t tag = TAG((uint8_t)msg[0], (uint8_t)msg[1]);

    char* a1 = nullptr;
    char* a2 = nullptr;
    if (msg[2] == ':') {
        a1 = msg + 3;
        char* sep = strchr(a1, ':');
        if (sep) { *sep = '\0'; a2 = sep + 1; }
    }

    switch (tag) {
        case TAG('J','G'): {
            if (!a1 || !a2) break;
            int j = atoi(a1), v = atoi(a2);
            if (j >= 0 && j < 6) webJog[j] = constrain(v, -100, 100);
            break;
        }
        case TAG('S','V'): {
            if (!a1 || !a2) break;
            int j = atoi(a1), a = atoi(a2);
            if (j >= 0 && j < 6 && a >= 0) { setServo(j, a); pendingBroadcast = true; }
            break;
        }
        case TAG('P','R'): {
            if (!a1) break;
            const int* p = nullptr;
            if      (!strcmp(a1, "home"))  p = POSE_HOME;
            else if (!strcmp(a1, "ready")) p = POSE_READY;
            else if (!strcmp(a1, "pick"))  p = POSE_PICK;
            else if (!strcmp(a1, "place")) p = POSE_PLACE;
            if (p) { smoothMoveAll(p); pendingBroadcast = true; }
            break;
        }
        case TAG('M','D'): {
            if (!a1) break;
            int m = atoi(a1);
            if (m >= 0 && m < 3) { joyMode = (uint8_t)m; pendingBroadcast = true; }
            break;
        }
        case TAG('S','P'): {
            if (!a1) break;
            int v = atoi(a1);
            if (v >= 1 && v <= 10) { moveSpeed = (uint8_t)v; pendingBroadcast = true; }
            break;
        }
        case TAG('R','C'): recordPose(); broadcastPoses(); break;
        case TAG('R','N'): {
            if (!a1 || !a2 || !*a2) break;
            int p = atoi(a1);
            if (p >= 0 && p < seqLen) {
                renamePose(p, a2);
                broadcastPoses();
                saveToFlash();
            }
            break;
        }
        case TAG('G','T'): {
            if (!a1) break;
            int p = atoi(a1);
            if (p >= 0 && p < seqLen) {
                smoothMoveAll(seq[p].a);
                pendingBroadcast = true;
            }
            break;
        }
        case TAG('P','Y'): startPlayback(); break;
        case TAG('S','T'): stopPlayback(); break;
        case TAG('C','Y'): armState == ARM_CYCLING ? stopCycle() : startCycle(); break;
        case TAG('C','L'): clearRecording(); broadcastPoses(); break;
        case TAG('S','A'): saveToFlash(); break;
        case TAG('L','D'): loadFromFlash(); broadcastPoses(); break;
    }
}

void onWsEvent(AsyncWebSocket*, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        client->text(buildStatus());
        client->text(buildPoses());
    } else if (type == WS_EVT_DISCONNECT) {
        for (int j = 0; j < 6; j++) webJog[j] = 0;
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            WsMsg m; size_t n = min(len, sizeof(m.buf) - 1);
            memcpy(m.buf, data, n); m.buf[n] = '\0';
            xQueueSend(wsQueue, &m, 0);
        }
    }
}

// ─── Web UI (minified) ──────────────────────────────────────────────────────
const char HTML_PAGE[] = R"raw(<!DOCTYPE html><html lang=en><head><meta charset=UTF-8><meta name=viewport content="width=device-width,initial-scale=1"><title>RoboArm</title><style>*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}body{font-family:Segoe UI,sans-serif;background:#1a1a2e;color:#eee;min-height:100vh;display:flex;flex-direction:column;align-items:center;padding:14px}h1{color:#e94560;font-size:1.5rem;margin-bottom:2px}.sub{color:#555;font-size:.7rem;letter-spacing:1px;margin-bottom:14px}.p{background:#16213e;border-radius:12px;padding:14px;width:100%;max-width:560px;margin-bottom:12px}.p h2{font-size:.66rem;color:#e94560;margin-bottom:11px;text-transform:uppercase;letter-spacing:3px}.jg{display:flex;flex-wrap:wrap;justify-content:center;gap:12px}.jb{width:82px;text-align:center}.jn{font-size:.7rem;color:#4fc3f7;margin-bottom:4px;font-weight:600}.jv{color:#e94560;font-weight:700;font-size:.85rem;margin-bottom:5px}.jc{width:60px;height:140px;margin:0 auto;background:#0f1f3a;border-radius:30px;position:relative;touch-action:none;border:2px solid #2a3a5e;overflow:hidden}.jd{width:46px;height:46px;background:#e94560;border-radius:50%;position:absolute;left:5px;top:45px;box-shadow:0 2px 8px rgba(233,69,96,.4);transition:background .1s}.jd.act{background:#27ae60;box-shadow:0 2px 12px rgba(39,174,96,.6)}.g2{display:grid;grid-template-columns:1fr 1fr;gap:8px}.g3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px}.g4{display:grid;grid-template-columns:repeat(4,1fr);gap:8px}button{padding:10px 5px;border:0;border-radius:8px;background:#0f3460;color:#eee;font-size:.78rem;cursor:pointer;font-weight:600;line-height:1.3;transition:background .12s}button:hover{background:#e94560}button:active{transform:scale(.97)}button.on{background:#e94560}button.red{background:#3a0f0f}button.red:hover{background:#c0392b}button.grn{background:#0f3a1a}button.grn:hover{background:#27ae60}button.amb{background:#3a2a00;font-size:.88rem;letter-spacing:.5px}button.amb:hover{background:#e6a817}button.amb.on{background:#e6a817;color:#111}.mb{background:#0f1f3a;border-radius:6px;padding:7px 11px;margin-bottom:10px;font-size:.76rem;color:#666}.mb b{color:#e94560}.bd{display:inline-block;background:#e94560;color:#fff;border-radius:10px;font-size:.65rem;padding:1px 7px;margin-left:5px;vertical-align:middle}.pl{max-height:180px;overflow-y:auto;margin-top:10px}.pi{display:flex;align-items:center;gap:7px;padding:5px 9px;border-radius:6px;margin-bottom:3px;background:#0f1f3a}.pnn{color:#e94560;font-weight:700;font-size:.78rem;min-width:26px}.pll{color:#ccc;font-size:.78rem;cursor:pointer;flex:1;padding:2px 4px;border-radius:4px}.pll:hover{background:#1a3060;color:#e94560}.pa{color:#444;font-size:.66rem;cursor:pointer}.pa:hover{color:#aaa}.ri{background:#0f1f3a;color:#eee;border:1px solid #e94560;border-radius:4px;padding:2px 5px;font-size:.78rem;width:110px;outline:none}.dot{display:inline-block;width:8px;height:8px;border-radius:50%;background:#444;margin-right:5px;transition:background .3s}.dot.ok{background:#27ae60}.dot.err{background:#e94560}#sb{color:#444;font-size:.72rem;text-align:center;margin-top:6px;min-height:1rem}</style></head><body><h1>RoboArm Control</h1><p class=sub>6-DOF ARM &mdash; ESP32 + PCA9685</p><div class=p><h2>Joints &mdash; Drag to Jog</h2><div class=jg id=joys></div></div><div class=p><h2>Presets</h2><div class=g4><button onclick="w('PR:home')">Home</button><button onclick="w('PR:ready')">Ready</button><button onclick="w('PR:pick')">Pick</button><button onclick="w('PR:place')">Place</button></div></div><div class=p><h2>Move Speed</h2><div style="display:flex;align-items:center;gap:10px;padding:4px 0"><span style="color:#555;font-size:.75rem">Slow</span><input type=range id=spd min=1 max=10 value=5 step=1 style="flex:1;accent-color:#e94560" oninput="w('SP:'+this.value);document.getElementById('spv').textContent=this.value"><span style="color:#555;font-size:.75rem">Fast</span><span id=spv style="color:#e94560;font-weight:700;font-size:.9rem;min-width:18px;text-align:right">5</span></div></div><div class=p><h2>HW Joystick Pair</h2><div class=mb>Hardware joystick: <b id=mn>Base + Shoulder</b></div><div class=g3><button id=m0 class=on onclick="w('MD:0')">Base<br>Shoulder</button><button id=m1 onclick="w('MD:1')">Elbow<br>Wrist P.</button><button id=m2 onclick="w('MD:2')">Wrist R.<br>Gripper</button></div></div><div class=p><h2>Recording <span class=bd id=rc>0</span></h2><div class=g4 style=margin-bottom:8px><button class=grn onclick="w('RC')">&#9679; REC</button><button id=bp onclick="w('PY')">&#9654; Play</button><button class=red onclick="w('ST')">&#9646;&#9646; Stop</button><button class=red onclick="dc()">&#128465; Clear</button></div><button id=bc class=amb style=width:100%;margin-bottom:8px onclick="w('CY')">&#9654;&#9654; CYCLE LOOP &mdash; OFF</button><div class=g2><button onclick="w('SA');st('Saved')">&#128190; Save</button><button onclick="w('LD');st('Loading&hellip;')">&#128228; Load</button></div><div class=pl id=pls></div></div><div id=sb><span class=dot id=dt></span><span id=stx>Connecting&hellip;</span></div><script>
const JD=[{n:'BASE',mn:0,mx:180},{n:'SHOULDER',mn:30,mx:150},{n:'ELBOW',mn:0,mx:135},{n:'WRIST PITCH',mn:0,mx:180},{n:'WRIST ROLL',mn:0,mx:180},{n:'GRIPPER',mn:0,mx:90}];
const MN=['Base + Shoulder','Elbow + Wrist Pitch','Wrist Roll + Gripper'];
let cM=0,ps=[],s,rT;const R=46,C=45;
const jc=document.getElementById('joys');
JD.forEach((j,i)=>jc.insertAdjacentHTML('beforeend',`<div class=jb data-j=${i}><div class=jn>${j.n}</div><div class=jv id=v${i}>90&deg;</div><div class=jc><div class=jd id=d${i}></div></div></div>`));
function cn(){
 s=new WebSocket(`ws://${location.hostname}/ws`);
 s.onopen=()=>{sd(1);st('Connected');clearTimeout(rT)};
 s.onclose=()=>{sd(0);st('Disconnected &mdash; reconnecting&hellip;');rT=setTimeout(cn,2500)};
 s.onerror=()=>s.close();
 s.onmessage=(e)=>{const d=JSON.parse(e.data);if(d.t==='s')aS(d);else if(d.t==='p')aP(d)};
}
function w(x){if(s&&s.readyState===1)s.send(x)}
function sd(o){const d=document.getElementById('dt');d.classList.toggle('ok',!!o);d.classList.toggle('err',!o)}
function st(m){document.getElementById('stx').innerHTML=m}
function aS(d){
 d.j.forEach((a,i)=>document.getElementById('v'+i).innerHTML=a+'&deg;');
 if(d.m!==cM){cM=d.m;[0,1,2].forEach(i=>document.getElementById('m'+i).classList.toggle('on',i===cM));document.getElementById('mn').textContent=MN[cM]}
 document.getElementById('rc').textContent=d.l;
 document.getElementById('bp').classList.toggle('on',!!d.p);
 const cb=document.getElementById('bc');cb.classList.toggle('on',!!d.c);
 cb.innerHTML=d.c?'&#9632; CYCLE LOOP &mdash; ON':'&#9654;&#9654; CYCLE LOOP &mdash; OFF';
 if(d.c)st(`Cycle &mdash; pose ${d.i}/${d.l}`);else if(d.p)st(`Playing &mdash; pose ${d.i}/${d.l}`);
 if(d.sp!==undefined){document.getElementById('spd').value=d.sp;document.getElementById('spv').textContent=d.sp}
}
function aP(d){ps=d.i||[];document.getElementById('rc').textContent=d.c;rP()}
function eh(x){return x.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/"/g,'&quot;')}
function rP(){
 const e=document.getElementById('pls');e.innerHTML='';
 ps.forEach((p,i)=>{
  const d=document.createElement('div');d.className='pi';
  d.innerHTML=`<span class=pnn>#${i+1}</span><span class=pll id=pl${i}>${eh(p.n)}</span><span class=pa title="Move arm here">${p.a.map((v,k)=>JD[k].n[0]+v+'&deg;').join(' ')}</span>`;
  d.querySelector('.pll').onclick=()=>sR(i);
  d.querySelector('.pa').onclick=()=>w('GT:'+i);
  e.appendChild(d);
 });
}
function sR(i){
 const e=document.getElementById('pl'+i),o=ps[i].n;
 e.innerHTML=`<input class=ri type=text value="${eh(o)}" maxlength=19>`;
 const I=e.querySelector('input');I.focus();I.select();
 I.onblur=()=>{const v=I.value.trim().replace(/:/g,' ');if(v&&v!==o)w('RN:'+i+':'+v);else e.textContent=o};
 I.onkeydown=ev=>{if(ev.key==='Enter')I.blur();if(ev.key==='Escape'){I.value=o;I.blur()}ev.stopPropagation()};
}
let A=null;
function eA(){if(!A)return;const a=A;A=null;a.d.classList.remove('act');a.d.style.top=C+'px';clearInterval(a.iv);w('JG:'+a.j+':0')}
document.querySelectorAll('.jb').forEach(b=>{
 const j=+b.dataset.j,c=b.querySelector('.jc'),d=b.querySelector('.jd');
 function sa(e){eA();const a={j,c,d,L:0,iv:0};a.iv=setInterval(()=>w('JG:'+a.j+':'+a.L),50);d.classList.add('act');A=a;e.preventDefault()}
 c.addEventListener('mousedown',sa);c.addEventListener('touchstart',sa,{passive:false});
});
function mv(e){
 if(!A)return;const r=A.c.getBoundingClientRect(),cy=e.touches?e.touches[0].clientY:e.clientY;
 let y=cy-(r.top+r.height/2);y=Math.max(-R,Math.min(R,y));A.d.style.top=(C+y)+'px';A.L=Math.round((-y/R)*100);
}
addEventListener('mouseup',eA);addEventListener('touchend',eA);addEventListener('mousemove',mv);addEventListener('touchmove',mv,{passive:false});
function dc(){if(confirm('Clear all recorded poses?'))w('CL')}
cn();
</script></body></html>)raw";

// ─── Serial commands ────────────────────────────────────────────────────────
void processSerial() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); cmd.toUpperCase();

    if (cmd.startsWith("S ")) {
        int s1 = cmd.indexOf(' '), s2 = cmd.indexOf(' ', s1+1);
        if (s2 < 0) { Serial.println("Usage: S <joint 0-5> <angle>"); return; }
        int i = cmd.substring(s1+1, s2).toInt();
        int a = cmd.substring(s2+1).toInt();
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
            setServo(i, joints[i].lo);   delay(600);
            setServo(i, joints[i].home); delay(600);
            setServo(i, joints[i].hi);   delay(600);
            setServo(i, joints[i].home); delay(600);
        }
        Serial.println("  done.");
        pendingBroadcast = true;
    }
    else if (cmd == "HOME")  { applyPreset(POSE_HOME);  pendingBroadcast = true; }
    else if (cmd == "READY") { applyPreset(POSE_READY); pendingBroadcast = true; }
    else if (cmd == "PICK")  { applyPreset(POSE_PICK);  pendingBroadcast = true; }
    else if (cmd == "PLACE") { applyPreset(POSE_PLACE); pendingBroadcast = true; }
    else if (cmd.startsWith("SPEED ")) {
        int v = cmd.substring(6).toInt();
        if (v >= 1 && v <= 10) { moveSpeed = (uint8_t)v; Serial.printf("[SPEED] %d\n", moveSpeed); pendingBroadcast = true; }
        else Serial.println("Speed 1-10");
    }
    else if (cmd == "REC")   recordPose();
    else if (cmd == "PLAY")  startPlayback();
    else if (cmd == "STOP")  stopPlayback();
    else if (cmd == "CYCLE") armState == ARM_CYCLING ? stopCycle() : startCycle();
    else if (cmd == "CLEAR") clearRecording();
    else if (cmd == "SAVE")  saveToFlash();
    else if (cmd == "LOAD")  loadFromFlash();
    else if (cmd == "STATUS") {
        Serial.println("-- Joints --");
        for (int i = 0; i < 6; i++)
            Serial.printf("  %d. %-14s ch%-2d  %3d deg  invert=%s\n",
                i, joints[i].name, joints[i].ch, joints[i].cur, joints[i].invert?"Y":"N");
        Serial.printf("  HW joy pair : %s\n", PAIR_NAME[joyMode]);
        Serial.printf("  Recording   : %d / %d\n", seqLen, MAX_POSES);
        Serial.printf("  State       : %s\n", stateName(armState));
        Serial.printf("  Move speed  : %d / 10  (%d steps x %d ms = ~%d ms)\n",
            moveSpeed, speedSteps(), speedMs(), speedSteps() * speedMs());
    }
    else if (cmd == "HELP") {
        Serial.println("-- Commands --");
        Serial.println("  S <joint 0-5> <angle>      Set joint angle");
        Serial.println("  INVERT <joint 0-5>         Toggle servo direction");
        Serial.println("  TEST                       Sweep all servos");
        Serial.println("  HOME/READY/PICK/PLACE      Preset poses");
        Serial.println("  REC/PLAY/STOP/CYCLE/CLEAR  Recording");
        Serial.println("  SPEED <1-10>               Move speed (1=slow, 10=fast)");
        Serial.println("  SAVE/LOAD                  Flash");
        Serial.println("  STATUS                     System status");
    }
    else if (cmd.length() > 0) Serial.println("Unknown. HELP.");
}

// ─── Setup ──────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== RoboArm 6-DOF ===");

    // Precompute PCA9685 PWM counts per integer angle
    for (int a = 0; a <= 180; a++) {
        long us = SERVO_MIN_US + (a * (SERVO_MAX_US - SERVO_MIN_US)) / 180;
        pwmTable[a] = (uint16_t)((us * 4096UL) / 20000UL);
    }

    servoMutex = xSemaphoreCreateMutex();
    wsQueue    = xQueueCreate(12, sizeof(WsMsg));

    Wire.begin(I2C_SDA, I2C_SCL);
    pca9685.begin();
    pca9685.setOscillatorFrequency(OSC_FREQ);
    pca9685.setPWMFreq(PWM_FREQ_HZ);
    delay(10);
    Serial.println("[PCA9685] OK");

    pinMode(JOY_SW_PIN,    INPUT_PULLUP);
    pinMode(BTN_REC_PIN,   INPUT_PULLUP);
    pinMode(BTN_PLAY_PIN,  INPUT_PULLUP);
    pinMode(BTN_CLR_PIN,   INPUT_PULLUP);
    pinMode(BTN_CYCLE_PIN, INPUT_PULLUP);
    for (int i = 0; i < 6; i++) joyF[i] = (float)joints[i].home;

    calibrateJoystick();
    moveToHome();
    Serial.println("[Servos] Home");

    loadFromFlash();

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("[WiFi] Joining \"%s\"", WIFI_SSID);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
        delay(400); Serial.print('.');
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED)
        Serial.printf("[WiFi] http://%s\n", WiFi.localIP().toString().c_str());
    else
        Serial.println("[WiFi] FAILED");

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req){
        req->send(200, "text/html", HTML_PAGE);
    });
    server.begin();
    Serial.println("Send HELP for serial commands.");
}

// ─── Loop ───────────────────────────────────────────────────────────────────
void loop() {
    WsMsg m;
    while (xQueueReceive(wsQueue, &m, 0)) processWsCmd(m.buf);

    processJoystick();
    processWebJog();
    processButtons();
    processPlayback();
    processSerial();

    ws.cleanupClients();

    if (pendingBroadcast || ((joyActive || webJogActive) && millis() - lastBroadMs >= 50)) {
        broadcastStatus();
        lastBroadMs      = millis();
        pendingBroadcast = false;
    }
}
