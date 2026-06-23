#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>

// ─── PCA9685 ─────────────────────────────────────────────────────────────────
#define PCA9685_ADDR 0x40
#define PWM_FREQ_HZ  50
#define OSC_FREQ     27000000
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define I2C_SDA      21
#define I2C_SCL      22

// ─── WiFi (STA mode — joins existing AP, same as new_case/test_case) ─────────
const char* WIFI_SSID = "ASEM Training";
const char* WIFI_PASS = "Class@Asem";

// ─── Joystick & Buttons (ADC1 only — ADC2 conflicts with WiFi) ───────────────
// Wiring: VCC→3.3V  GND→GND  VRX→34  VRY→35  SW→32
#define JOY_X_PIN     34
#define JOY_Y_PIN     35
#define JOY_SW_PIN    32   // cycle joint pair
#define BTN_REC_PIN   33   // record pose
#define BTN_PLAY_PIN  25   // play / stop
#define BTN_CLR_PIN   26   // clear recording
#define BTN_CYCLE_PIN 27   // factory loop on/off

#define JOY_DEADZONE  300
#define JOY_SPEED     1.2f   // max °/tick at full deflection (tick = 20 ms)
#define JOY_INTERVAL  20
#define DEBOUNCE_MS   220

// Joystick controls two joints at a time; SW cycles through pairs
const uint8_t PAIR[3][2]   = { {0,1}, {2,3}, {4,5} };
const char*   PAIR_NAME[3] = { "Base+Shoulder", "Elbow+Wrist Pitch", "Wrist Roll+Gripper" };

// ─── Recording ────────────────────────────────────────────────────────────────
#define MAX_POSES    50
#define PLAY_STEP_MS 1200   // ms the arm holds each pose during playback/cycle

struct Pose {
    int  a[6];
    char label[20];
};

Pose     seq[MAX_POSES];
int      seqLen     = 0;
bool     isPlaying  = false;
bool     isCycling  = false;
int      playIdx    = 0;
uint32_t playNextMs = 0;

// ─── Joint table ──────────────────────────────────────────────────────────────
struct Joint {
    const char* name;
    uint8_t     ch;        // PCA9685 channel
    int         lo, hi;    // mechanical limits (°)
    int         home;      // startup angle (°)
    int         cur;       // current logical angle (°)
    bool        invert;    // true = servo physically mounted in reverse
};

// Set invert=true for any servo where direction is backwards.
// Use serial command  INVERT <0-5>  to toggle at runtime without reflashing.
Joint joints[6] = {
    //  name           ch   lo    hi  home  cur  invert
    { "Base",           5,   0,  180,  90,  90, false },
    { "Shoulder",       0,  30,  150,  90,  90, true  },
    { "Elbow",          1,   0,  135,  90,  90, false },
    { "Wrist Pitch",    2,   0,  180,  90,  90, false },
    { "Wrist Roll",     3,   0,  180,  90,  90, false },
    { "Gripper",        4,   0,   90,  45,  45, false },
};

const int POSE_HOME [6] = {  90,  90,  90,  90,  90, 45 };
const int POSE_READY[6] = {  90,  60,  90,  90,  90, 45 };
const int POSE_PICK [6] = {  90,  45,  45,  90,  90,  0 };
const int POSE_PLACE[6] = {  90,  45,  45,  90,  90, 90 };

// ─── Globals ──────────────────────────────────────────────────────────────────
Adafruit_PWMServoDriver pca9685(PCA9685_ADDR);
AsyncWebServer  server(80);
AsyncWebSocket  ws("/ws");
Preferences     prefs;
SemaphoreHandle_t servoMutex;

float    joyF[6];
uint8_t  joyMode    = 0;
int      joyXCenter = 2048;
int      joyYCenter = 2048;
bool     joyActive  = false;
uint32_t joyLastMs  = 0;
uint32_t lastBroadMs = 0;

uint32_t lastSW=0, lastRec=0, lastPlay=0, lastClr=0, lastCycle=0;

// WS messages from callback land in this queue; main loop drains it safely
struct WsMsg { char buf[128]; };
QueueHandle_t wsQueue;
bool pendingBroadcast = false;

// ─── Servo helpers ────────────────────────────────────────────────────────────
uint16_t toCounts(int angle) {
    long us = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
    return (uint16_t)((us * 4096UL) / 20000UL);
}

// Full setServo: constrains, updates joyF, applies invert, uses I2C mutex
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

// Fast path for joystick hot-loop: no mutex, no joyF sync (called only from main loop)
inline void sendPWM(uint8_t i, int angle) {
    joints[i].cur = angle;
    int phys = joints[i].invert ? (joints[i].lo + joints[i].hi - angle) : angle;
    pca9685.setPWM(joints[i].ch, 0, toCounts(phys));
}

void smoothMove(uint8_t i, int target, uint8_t steps = 40, uint8_t ms = 15) {
    int start = joints[i].cur;
    target = constrain(target, joints[i].lo, joints[i].hi);
    for (uint8_t s = 1; s <= steps; s++) {
        setServo(i, start + (target - start) * s / steps);
        delay(ms);
    }
}

void moveToHome() {
    for (int i = 0; i < 6; i++) setServo(i, joints[i].home);
}

void applyPreset(const int p[6]) {
    for (int i = 0; i < 6; i++) smoothMove(i, p[i]);
}

// ─── Recording ────────────────────────────────────────────────────────────────
void recordPose() {
    if (seqLen >= MAX_POSES) { Serial.println("[REC] Full (50 max)"); return; }
    for (int i = 0; i < 6; i++) seq[seqLen].a[i] = joints[i].cur;
    snprintf(seq[seqLen].label, sizeof(seq[seqLen].label), "Pose %d", seqLen + 1);
    seqLen++;
    Serial.printf("[REC] %d poses total\n", seqLen);
    pendingBroadcast = true;
}

void renamePose(int idx, const char* name) {
    if (idx < 0 || idx >= seqLen) return;
    strncpy(seq[idx].label, name, sizeof(seq[idx].label) - 1);
    seq[idx].label[sizeof(seq[idx].label) - 1] = '\0';
}

void startPlayback() {
    if (!seqLen) { Serial.println("[PLAY] Nothing recorded"); return; }
    isCycling = false; isPlaying = true;
    playIdx = 0; playNextMs = millis();
    Serial.printf("[PLAY] %d poses\n", seqLen);
    pendingBroadcast = true;
}

void stopPlayback()  { isPlaying = false; isCycling = false; pendingBroadcast = true; }

void startCycle() {
    if (!seqLen) { Serial.println("[CYCLE] Nothing recorded"); return; }
    isPlaying = false; isCycling = true;
    playIdx = 0; playNextMs = millis();
    Serial.printf("[CYCLE] Looping %d poses forever\n", seqLen);
    pendingBroadcast = true;
}

void stopCycle() { isCycling = false; pendingBroadcast = true; }

void clearRecording() {
    seqLen = 0; isPlaying = false; isCycling = false; pendingBroadcast = true;
    Serial.println("[REC] Cleared");
}

void saveToFlash() {
    prefs.begin("roboarm", false);
    prefs.putInt("len", seqLen);
    if (seqLen > 0) prefs.putBytes("seq", seq, seqLen * sizeof(Pose));
    prefs.end();
    Serial.printf("[FLASH] Saved %d poses\n", seqLen);
}

void loadFromFlash() {
    prefs.begin("roboarm", true);
    seqLen = prefs.getInt("len", 0);
    if (seqLen > 0) prefs.getBytes("seq", seq, seqLen * sizeof(Pose));
    prefs.end();
    Serial.printf("[FLASH] Loaded %d poses\n", seqLen);
    pendingBroadcast = true;
}

// ─── Joystick ─────────────────────────────────────────────────────────────────
void calibrateJoystick() {
    long xs = 0, ys = 0;
    const int N = 64;
    for (int i = 0; i < N; i++) {
        xs += analogRead(JOY_X_PIN);
        ys += analogRead(JOY_Y_PIN);
        delay(5);
    }
    joyXCenter = (int)(xs / N);
    joyYCenter = (int)(ys / N);
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
    if (btnPressed(JOY_SW_PIN,    lastSW))    { joyMode = (joyMode+1) % 3; Serial.printf("[JOY] %s\n", PAIR_NAME[joyMode]); pendingBroadcast = true; }
    if (btnPressed(BTN_REC_PIN,   lastRec))     recordPose();
    if (btnPressed(BTN_PLAY_PIN,  lastPlay))    isPlaying ? stopPlayback() : startPlayback();
    if (btnPressed(BTN_CLR_PIN,   lastClr))     clearRecording();
    if (btnPressed(BTN_CYCLE_PIN, lastCycle))   isCycling ? stopCycle() : startCycle();
}

void processPlayback() {
    if (!isPlaying && !isCycling) return;
    if (millis() < playNextMs) return;

    if (playIdx >= seqLen) {
        if (isCycling) {
            playIdx = 0;
            Serial.println("[CYCLE] Looping back to pose 1");
        } else {
            isPlaying = false; playIdx = 0;
            Serial.println("[PLAY] Done");
            pendingBroadcast = true;
            return;
        }
    }

    for (int i = 0; i < 6; i++) setServo(i, seq[playIdx].a[i]);
    Serial.printf("[%s] %d/%d  \"%s\"\n",
        isCycling ? "CYCLE" : "PLAY", playIdx + 1, seqLen, seq[playIdx].label);
    playIdx++;
    playNextMs  = millis() + PLAY_STEP_MS;
    pendingBroadcast = true;
}

// ─── WebSocket helpers ────────────────────────────────────────────────────────
String statusJson() {
    String j = "{\"type\":\"status\",\"j\":[";
    for (int i = 0; i < 6; i++) { j += joints[i].cur; if (i < 5) j += ','; }
    j += "],\"mode\":";   j += joyMode;
    j += ",\"seqLen\":";  j += seqLen;
    j += ",\"playing\":"; j += isPlaying ? "true" : "false";
    j += ",\"cycling\":"; j += isCycling ? "true" : "false";
    j += ",\"playIdx\":"; j += playIdx;
    return j + "}";
}

String posesJson() {
    String j = "{\"type\":\"poses\",\"count\":"; j += seqLen; j += ",\"items\":[";
    for (int i = 0; i < seqLen; i++) {
        j += "{\"a\":[";
        for (int k = 0; k < 6; k++) { j += seq[i].a[k]; if (k < 5) j += ','; }
        j += "],\"n\":\""; j += seq[i].label; j += "\"}";
        if (i < seqLen - 1) j += ',';
    }
    return j + "]}";
}

// Minimal JSON field extractors — sufficient for our fixed protocol
int    jInt(const String& s, const char* k) {
    String key = String('"') + k + "\":";
    int i = s.indexOf(key); return i < 0 ? -1 : s.substring(i + key.length()).toInt();
}
String jStr(const String& s, const char* k) {
    String key = String('"') + k + "\":\"";
    int i = s.indexOf(key); if (i < 0) return "";
    int st = i + key.length(), en = s.indexOf('"', st);
    return en < 0 ? "" : s.substring(st, en);
}

void processWsCmd(const String& msg) {
    String cmd = jStr(msg, "cmd");

    if (cmd == "servo") {
        int j = jInt(msg, "j"), a = jInt(msg, "a");
        if (j >= 0 && j < 6 && a >= 0) { setServo(j, a); pendingBroadcast = true; }
    }
    else if (cmd == "preset") {
        String n = jStr(msg, "name"); const int* p = nullptr;
        if (n=="home") p=POSE_HOME; else if(n=="ready") p=POSE_READY;
        else if(n=="pick") p=POSE_PICK; else if(n=="place") p=POSE_PLACE;
        if (p) { for(int i=0;i<6;i++) setServo(i,p[i]); pendingBroadcast = true; }
    }
    else if (cmd == "joymode") {
        int m = jInt(msg, "m");
        if (m >= 0 && m < 3) { joyMode = (uint8_t)m; pendingBroadcast = true; }
    }
    else if (cmd == "record") { recordPose(); ws.textAll(posesJson()); }
    else if (cmd == "rename") {
        int p = jInt(msg, "p"); String n = jStr(msg, "name");
        if (p >= 0 && p < seqLen && n.length()) {
            renamePose(p, n.c_str());
            ws.textAll(posesJson());
            saveToFlash();   // persist name immediately
        }
    }
    else if (cmd == "goto") {
        int p = jInt(msg, "p");
        if (p >= 0 && p < seqLen) { for(int i=0;i<6;i++) setServo(i,seq[p].a[i]); pendingBroadcast = true; }
    }
    else if (cmd == "play")   startPlayback();
    else if (cmd == "stop")   stopPlayback();
    else if (cmd == "cycle")  isCycling ? stopCycle() : startCycle();
    else if (cmd == "clear")  { clearRecording(); ws.textAll(posesJson()); }
    else if (cmd == "save")   saveToFlash();
    else if (cmd == "load")   { loadFromFlash(); ws.textAll(posesJson()); }
}

void onWsEvent(AsyncWebSocket* sv, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        client->text(statusJson());
        client->text(posesJson());
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            WsMsg m; size_t n = min(len, sizeof(m.buf) - 1);
            memcpy(m.buf, data, n); m.buf[n] = 0;
            xQueueSend(wsQueue, &m, 0);
        }
    }
}

// ─── Web UI ───────────────────────────────────────────────────────────────────
const char HTML_PAGE[] = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RoboArm</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',sans-serif;background:#1a1a2e;color:#eee;
     min-height:100vh;display:flex;flex-direction:column;align-items:center;padding:20px}
h1{color:#e94560;margin-bottom:5px;font-size:1.8rem}
.sub{color:#555;margin-bottom:22px;font-size:.78rem;letter-spacing:1px}
.panel{background:#16213e;border-radius:12px;padding:18px;width:100%;max-width:540px;margin-bottom:14px}
.panel h2{font-size:.7rem;color:#e94560;margin-bottom:13px;text-transform:uppercase;letter-spacing:3px}
.joint{margin-bottom:15px}
.jrow{display:flex;justify-content:space-between;margin-bottom:5px}
.jname{font-weight:600;font-size:.88rem}
.jval{color:#e94560;font-weight:700;font-size:.88rem;min-width:46px;text-align:right}
.jlim{color:#2a3a5e;font-size:.67rem;display:flex;justify-content:space-between;margin-top:2px}
input[type=range]{width:100%;accent-color:#e94560;cursor:pointer;height:5px}
.g2{display:grid;grid-template-columns:1fr 1fr;gap:9px}
.g3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:9px}
.g4{display:grid;grid-template-columns:1fr 1fr 1fr 1fr;gap:9px}
button{padding:11px 6px;border:none;border-radius:8px;background:#0f3460;color:#eee;
       font-size:.82rem;cursor:pointer;transition:background .15s;font-weight:600;line-height:1.35}
button:hover{background:#e94560}
button:active{transform:scale(.97)}
button.on{background:#e94560}
button.red{background:#3a0f0f}
button.red:hover{background:#c0392b}
button.grn{background:#0f3a1a}
button.grn:hover{background:#27ae60}
button.amber{background:#3a2a00;font-size:.93rem;letter-spacing:.5px}
button.amber:hover{background:#e6a817}
button.amber.on{background:#e6a817;color:#111}
.mbar{background:#0f1f3a;border-radius:7px;padding:8px 12px;margin-bottom:12px;font-size:.8rem;color:#666}
.mbar b{color:#e94560}
.badge{display:inline-block;background:#e94560;color:#fff;border-radius:10px;
       font-size:.67rem;padding:1px 7px;margin-left:6px;vertical-align:middle}
.plist{max-height:180px;overflow-y:auto;margin-top:10px}
.pi{display:flex;align-items:center;gap:8px;padding:6px 10px;border-radius:6px;
    margin-bottom:3px;background:#0f1f3a}
.pn{color:#e94560;font-weight:700;font-size:.82rem;min-width:28px}
.plabel{color:#ccc;font-size:.82rem;cursor:pointer;flex:1;padding:2px 4px;border-radius:4px}
.plabel:hover{background:#1a3060;color:#e94560}
.pa{color:#444;font-size:.7rem;cursor:pointer}
.pa:hover{color:#aaa}
.ri{background:#0f1f3a;color:#eee;border:1px solid #e94560;border-radius:4px;
    padding:2px 6px;font-size:.82rem;width:120px;outline:none}
.dot{display:inline-block;width:8px;height:8px;border-radius:50%;background:#444;
     margin-right:6px;transition:background .3s}
.dot.ok{background:#27ae60}
.dot.err{background:#e94560}
#status{color:#444;font-size:.72rem;text-align:center;margin-top:6px;min-height:1rem}
</style>
</head>
<body>
<h1>RoboArm Control</h1>
<p class="sub">6-DOF DESKTOP ARM &mdash; ESP32 + PCA9685</p>

<div class="panel">
  <h2>Joints</h2>
  <div id="joints"></div>
</div>

<div class="panel">
  <h2>Presets</h2>
  <div class="g4">
    <button onclick="preset('home')">Home</button>
    <button onclick="preset('ready')">Ready</button>
    <button onclick="preset('pick')">Pick</button>
    <button onclick="preset('place')">Place</button>
  </div>
</div>

<div class="panel">
  <h2>Joystick Mode</h2>
  <div class="mbar">Controlling: <b id="mname">Base + Shoulder</b></div>
  <div class="g3">
    <button id="mb0" class="on"  onclick="setMode(0)">Base<br>Shoulder</button>
    <button id="mb1"             onclick="setMode(1)">Elbow<br>Wrist P.</button>
    <button id="mb2"             onclick="setMode(2)">Wrist R.<br>Gripper</button>
  </div>
</div>

<div class="panel">
  <h2>Recording <span class="badge" id="rcnt">0</span></h2>
  <div class="g4" style="margin-bottom:9px">
    <button class="grn" onclick="doRec()">&#9679; REC</button>
    <button id="bplay"  onclick="doPlay()">&#9654; Play</button>
    <button class="red" onclick="doStop()">&#9646;&#9646; Stop</button>
    <button class="red" onclick="doClear()">&#128465; Clear</button>
  </div>
  <button id="bcycle" class="amber" style="width:100%;margin-bottom:9px"
    onclick="doCycle()">&#9654;&#9654; CYCLE LOOP &mdash; OFF</button>
  <div class="g2">
    <button onclick="doSave()">&#128190; Save to Flash</button>
    <button onclick="doLoad()">&#128228; Load from Flash</button>
  </div>
  <div class="plist" id="plist"></div>
</div>

<div id="status"><span class="dot" id="dot"></span>Connecting…</div>

<script>
const JD=[
  {name:'Base',       min:0,  max:180},
  {name:'Shoulder',   min:30, max:150},
  {name:'Elbow',      min:0,  max:135},
  {name:'Wrist Pitch',min:0,  max:180},
  {name:'Wrist Roll', min:0,  max:180},
  {name:'Gripper',    min:0,  max:90},
];
const MN=['Base + Shoulder','Elbow + Wrist Pitch','Wrist Roll + Gripper'];
let curM=0, poses=[], ws, reconnTimer;

// Build joint sliders
const jc=document.getElementById('joints');
JD.forEach((j,i)=>{
  jc.innerHTML+=`<div class="joint">
    <div class="jrow"><span class="jname">${j.name}</span><span class="jval" id="v${i}">90&deg;</span></div>
    <input type="range" min="${j.min}" max="${j.max}" value="90" id="s${i}"
      oninput="onSlide(${i},this.value)" onchange="onSlideEnd(${i},this.value)">
    <div class="jlim"><span>${j.min}&deg;</span><span>${j.max}&deg;</span></div>
  </div>`;
});

// ── WebSocket ──────────────────────────────────────────────────────────────────
function connect(){
  ws=new WebSocket(`ws://${location.hostname}/ws`);
  ws.onopen=()=>{setDot(true); st('Connected'); clearTimeout(reconnTimer);};
  ws.onclose=()=>{setDot(false); st('Disconnected — reconnecting…'); reconnTimer=setTimeout(connect,2500);};
  ws.onerror=()=>ws.close();
  ws.onmessage=(e)=>{
    const d=JSON.parse(e.data);
    if(d.type==='status') applyStatus(d);
    if(d.type==='poses')  applyPoses(d);
  };
}

function send(obj){
  if(ws&&ws.readyState===WebSocket.OPEN) ws.send(JSON.stringify(obj));
}

function setDot(ok){
  const d=document.getElementById('dot');
  d.classList.toggle('ok',ok); d.classList.toggle('err',!ok);
}

// ── Status & pose sync ─────────────────────────────────────────────────────────
function applyStatus(d){
  d.j.forEach((a,i)=>setSlider(i,a));
  if(d.mode!==curM){
    curM=d.mode;
    [0,1,2].forEach(i=>document.getElementById('mb'+i).classList.toggle('on',i===curM));
    document.getElementById('mname').textContent=MN[curM];
  }
  document.getElementById('rcnt').textContent=d.seqLen;
  document.getElementById('bplay').classList.toggle('on',d.playing);
  const cb=document.getElementById('bcycle');
  cb.classList.toggle('on',d.cycling);
  cb.textContent=d.cycling?'⏹ CYCLE LOOP — ON':'▶▶ CYCLE LOOP — OFF';
  if(d.cycling) st(`▶▶ Cycle — pose ${d.playIdx}/${d.seqLen}`);
  else if(d.playing) st(`▶ Playing — pose ${d.playIdx}/${d.seqLen}`);
}

function applyPoses(d){
  poses=d.items;
  document.getElementById('rcnt').textContent=d.count;
  renderPoses();
}

function renderPoses(){
  const el=document.getElementById('plist');
  el.innerHTML='';
  poses.forEach((p,i)=>{
    const div=document.createElement('div');
    div.className='pi';
    div.innerHTML=
      `<span class="pn">#${i+1}</span>`+
      `<span class="plabel" id="pl${i}">${escHtml(p.n)}</span>`+
      `<span class="pa" title="Click to move arm here">${p.a.map((v,k)=>JD[k].name[0]+v+'°').join(' ')}</span>`;
    div.querySelector('.plabel').onclick=()=>startRename(i);
    div.querySelector('.pa').onclick=()=>send({cmd:'goto',p:i});
    el.appendChild(div);
  });
}

function escHtml(s){return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/"/g,'&quot;');}

function startRename(i){
  const el=document.getElementById('pl'+i);
  const old=poses[i].n;
  el.innerHTML=`<input class="ri" type="text" value="${escHtml(old)}" maxlength="19">`;
  const inp=el.querySelector('input');
  inp.focus(); inp.select();
  inp.onblur=()=>{
    const v=inp.value.trim();
    if(v&&v!==old) send({cmd:'rename',p:i,name:v});
    else el.textContent=old;
  };
  inp.onkeydown=e=>{
    if(e.key==='Enter') inp.blur();
    if(e.key==='Escape'){inp.value=old;inp.blur();}
    e.stopPropagation();
  };
}

// ── Controls ──────────────────────────────────────────────────────────────────
function setSlider(i,a){document.getElementById('v'+i).textContent=a+'°';document.getElementById('s'+i).value=a;}

let sDebounce={};
function onSlide(i,v){document.getElementById('v'+i).textContent=v+'°';clearTimeout(sDebounce[i]);sDebounce[i]=setTimeout(()=>send({cmd:'servo',j:i,a:parseInt(v)}),40);}
function onSlideEnd(i,v){clearTimeout(sDebounce[i]);send({cmd:'servo',j:i,a:parseInt(v)});}

function preset(n){send({cmd:'preset',name:n});st('Moving to '+n+'…');}
function setMode(m){send({cmd:'joymode',m});}
function doRec() {send({cmd:'record'});}
function doPlay(){send({cmd:'play'});}
function doStop(){send({cmd:'stop'});}
function doCycle(){send({cmd:'cycle'});}
function doClear(){if(confirm('Clear all recorded poses?'))send({cmd:'clear'});}
function doSave(){send({cmd:'save'});st('Saved to flash');}
function doLoad(){send({cmd:'load'});st('Loading…');}

function st(msg){document.getElementById('status').textContent=msg;}

connect();
</script>
</body>
</html>
)rawliteral";

// ─── Serial commands ──────────────────────────────────────────────────────────
void processSerial() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); cmd.toUpperCase();

    if (cmd.startsWith("S ")) {
        int s1=cmd.indexOf(' '), s2=cmd.indexOf(' ',s1+1);
        if (s2<0){Serial.println("Usage: S <joint 0-5> <angle>");return;}
        int i=cmd.substring(s1+1,s2).toInt(), a=cmd.substring(s2+1).toInt();
        if (i<0||i>=6){Serial.println("Joint 0-5");return;}
        setServo(i,a); Serial.printf("[%s] -> %d deg\n",joints[i].name,joints[i].cur);
    }
    else if (cmd.startsWith("INVERT ")) {
        int j=cmd.substring(7).toInt();
        if (j>=0&&j<6){
            joints[j].invert=!joints[j].invert;
            Serial.printf("[%s] invert = %s  (re-home to see effect)\n",
                joints[j].name, joints[j].invert?"ON":"OFF");
        }
    }
    // TEST sweeps every joint lo→home→hi→home so you can verify each channel
    else if (cmd == "TEST") {
        Serial.println("── Servo channel test ────────────");
        for (int i=0;i<6;i++){
            Serial.printf("  Joint %d (%s) ch%d ... lo=%d hi=%d\n",
                i,joints[i].name,joints[i].ch,joints[i].lo,joints[i].hi);
            setServo(i,joints[i].lo);  delay(600);
            setServo(i,joints[i].home);delay(600);
            setServo(i,joints[i].hi);  delay(600);
            setServo(i,joints[i].home);delay(600);
        }
        Serial.println("  Test complete.");
    }
    else if(cmd=="HOME")   { applyPreset(POSE_HOME); Serial.println("HOME"); }
    else if(cmd=="READY")  { applyPreset(POSE_READY);Serial.println("READY"); }
    else if(cmd=="PICK")   { applyPreset(POSE_PICK); Serial.println("PICK"); }
    else if(cmd=="PLACE")  { applyPreset(POSE_PLACE);Serial.println("PLACE"); }
    else if(cmd=="REC")    { recordPose(); }
    else if(cmd=="PLAY")   { startPlayback(); }
    else if(cmd=="STOP")   { stopPlayback(); }
    else if(cmd=="CYCLE")  { isCycling?stopCycle():startCycle(); }
    else if(cmd=="CLEAR")  { clearRecording(); }
    else if(cmd=="SAVE")   { saveToFlash(); }
    else if(cmd=="LOAD")   { loadFromFlash(); }
    else if(cmd=="STATUS") {
        Serial.println("── Joints ─────────────────────────────────────────");
        for(int i=0;i<6;i++)
            Serial.printf("  %d. %-14s ch%-2d  %3d°  invert=%s\n",
                i,joints[i].name,joints[i].ch,joints[i].cur,joints[i].invert?"Y":"N");
        Serial.printf("  Joy mode : %s\n",PAIR_NAME[joyMode]);
        Serial.printf("  Recording: %d / %d\n",seqLen,MAX_POSES);
        Serial.printf("  Playing  : %s   Cycling: %s\n",isPlaying?"yes":"no",isCycling?"yes":"no");
    }
    else if(cmd=="HELP") {
        Serial.println("── Commands ──────────────────────────────────────────");
        Serial.println("  S <joint 0-5> <angle>  Set joint angle");
        Serial.println("  INVERT <joint 0-5>     Toggle servo direction");
        Serial.println("  TEST                   Sweep all servos (verify channels)");
        Serial.println("  HOME/READY/PICK/PLACE  Preset poses");
        Serial.println("  REC                    Record current pose");
        Serial.println("  PLAY / STOP            One-shot playback");
        Serial.println("  CYCLE                  Toggle factory loop");
        Serial.println("  CLEAR                  Clear recording");
        Serial.println("  SAVE / LOAD            Flash persistence");
        Serial.println("  STATUS                 System status");
    }
    else if(cmd.length()>0) Serial.println("Unknown. Send HELP.");
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== RoboArm 6-DOF Controller ===");

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
    Serial.println("[Servos] Home reached");

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
        Serial.printf("[WiFi] http://%s  (ws://%s/ws)\n",
            WiFi.localIP().toString().c_str(),
            WiFi.localIP().toString().c_str());
    else
        Serial.println("[WiFi] FAILED — check SSID/password");

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req){
        req->send(200, "text/html", HTML_PAGE);
    });
    server.begin();
    Serial.println("Serial ready. Send HELP for commands.");
    Serial.println("Send TEST to sweep all servos and verify channels.");
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
    // Drain WS command queue
    WsMsg m;
    while (xQueueReceive(wsQueue, &m, 0)) {
        processWsCmd(String(m.buf));
    }

    processJoystick();
    processButtons();
    processPlayback();
    processSerial();

    ws.cleanupClients();

    // Broadcast state changes + joystick movement at 50 ms intervals
    if (pendingBroadcast || (joyActive && millis() - lastBroadMs >= 50)) {
        ws.textAll(statusJson());
        lastBroadMs      = millis();
        pendingBroadcast = false;
    }
}
