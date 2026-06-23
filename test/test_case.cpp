#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_PWMServoDriver.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>

// ================= WiFi credentials =================
const char* WIFI_SSID = "ASEM Training";
const char* WIFI_PASS = "Class@Asem";

// ================= PCA9685 =================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define SDA_PIN 21
#define SCL_PIN 22
#define SERVOMIN  102
#define SERVOMAX  512
#define SERVO_FREQ 50

// ================= Joint table (from main.cpp) =================
struct Joint {
    const char* name;
    uint8_t     ch;        // PCA9685 channel
    int         lo, hi;    // mechanical limits (°)
    int         home;      // startup angle (°)
    int         cur;       // current logical angle (°)
    bool        invert;    // true = servo physically mounted in reverse
};

const int NUM_JOINTS = 6;

// Set invert=true for any servo whose direction is backwards.
Joint joints[NUM_JOINTS] = {
    //  name           ch   lo    hi  home  cur  invert
    { "BASE",           5,   0,  180,  90,  90, false },
    { "SHOULDER",       0,  30,  150,  90,  90, true  },
    { "ELBOW",          1,   0,  135,  90,  90, false },
    { "WRIST_PITCH",    2,   0,  180,  90,  90, false },
    { "WRIST_ROLL",     3,   0,  180,  90,  90, false },
    { "GRIPPER",        4,   0,   90,  45,  45, false },
};

// Preset poses
const int POSE_HOME [NUM_JOINTS] = { 90, 90, 90, 90, 90, 45 };
const int POSE_READY[NUM_JOINTS] = { 90, 60, 90, 90, 90, 45 };
const int POSE_PICK [NUM_JOINTS] = { 90, 45, 45, 90, 90,  0 };
const int POSE_PLACE[NUM_JOINTS] = { 90, 45, 45, 90, 90, 90 };

// ================= Recording state =================
struct Pose { int angles[NUM_JOINTS]; };
#define MAX_POSES 50
Pose recordedSequence[MAX_POSES];
int  poseCount = 0;

bool recordingMode = true;
int  holdMsBetweenPoses = 1000;

// ================= Servo helpers =================
int angleToPulse(int angle) {
    angle = constrain(angle, 0, 180);
    return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void setJointAngle(uint8_t j, int angle) {
    angle = constrain(angle, joints[j].lo, joints[j].hi);
    joints[j].cur = angle;
    int phys = joints[j].invert ? (joints[j].lo + joints[j].hi - angle) : angle;
    pwm.setPWM(joints[j].ch, 0, angleToPulse(phys));
}

void moveAllToHome() {
    for (int i = 0; i < NUM_JOINTS; i++) setJointAngle(i, joints[i].home);
}

void moveToPose(const int targetAngles[NUM_JOINTS], int steps = 30, int stepDelayMs = 15) {
    int startAngles[NUM_JOINTS];
    for (int i = 0; i < NUM_JOINTS; i++) startAngles[i] = joints[i].cur;
    for (int s = 1; s <= steps; s++) {
        for (int j = 0; j < NUM_JOINTS; j++) {
            int interp = startAngles[j] + (targetAngles[j] - startAngles[j]) * s / steps;
            setJointAngle(j, interp);
        }
        delay(stepDelayMs);
    }
}

// ================= Web server / WebSocket =================
AsyncWebServer server(80);
WebSocketsServer webSocket(81);

#define JOG_SPEED_MAX 3
#define JOG_INTERVAL_MS 30
unsigned long lastJogTime = 0;

// Each joint has its own live jog value, -100..100, updated as joysticks are dragged
int joystickValue[NUM_JOINTS] = {0, 0, 0, 0, 0, 0};

void recordCurrentPose() {
    if (poseCount >= MAX_POSES) {
        Serial.println("Pose buffer full.");
        return;
    }
    for (int i = 0; i < NUM_JOINTS; i++) recordedSequence[poseCount].angles[i] = joints[i].cur;
    poseCount++;
    Serial.printf("Recorded pose %d\n", poseCount);
}

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Arm Control</title>
<style>
  body { font-family: sans-serif; text-align: center; background: #1e1e1e; color: #eee; margin: 0; padding: 16px; }
  h2 { margin-bottom: 10px; }
  #joysticks {
    display: flex; flex-wrap: wrap; justify-content: center; gap: 18px;
  }
  .joyBlock { width: 90px; }
  .joyLabel { font-size: 0.85em; color: #4fc3f7; margin-bottom: 6px; }
  .joyContainer {
    width: 70px; height: 160px; margin: 0 auto;
    background: #333; border-radius: 35px; position: relative;
    touch-action: none; border: 2px solid #555;
  }
  .joyDot {
    width: 50px; height: 50px; background: #4fc3f7; border-radius: 50%;
    position: absolute; left: 10px; top: 55px;
  }
  .row { margin: 18px 0; }
  button {
    font-size: 1em; padding: 12px 20px; margin: 5px; border: none;
    border-radius: 8px; cursor: pointer;
  }
  #recordBtn { background: #2e7d32; color: #fff; }
  #finishBtn { background: #c62828; color: #fff; }
  #status { margin-top: 15px; font-size: 0.9em; color: #aaa; }
  #angles { font-size: 0.8em; color: #888; margin-top: 10px; line-height: 1.5; }
</style>
</head>
<body>
<h2>6-DOF Arm Control</h2>

<div id="joysticks">
  <div class="joyBlock" data-joint="0">
    <div class="joyLabel">BASE</div>
    <div class="joyContainer"><div class="joyDot"></div></div>
  </div>
  <div class="joyBlock" data-joint="1">
    <div class="joyLabel">SHOULDER</div>
    <div class="joyContainer"><div class="joyDot"></div></div>
  </div>
  <div class="joyBlock" data-joint="2">
    <div class="joyLabel">ELBOW</div>
    <div class="joyContainer"><div class="joyDot"></div></div>
  </div>
  <div class="joyBlock" data-joint="3">
    <div class="joyLabel">WRIST PITCH</div>
    <div class="joyContainer"><div class="joyDot"></div></div>
  </div>
  <div class="joyBlock" data-joint="4">
    <div class="joyLabel">WRIST ROLL</div>
    <div class="joyContainer"><div class="joyDot"></div></div>
  </div>
  <div class="joyBlock" data-joint="5">
    <div class="joyLabel">GRIPPER</div>
    <div class="joyContainer"><div class="joyDot"></div></div>
  </div>
</div>

<div class="row">
  <button id="recordBtn">RECORD POSE</button>
  <button id="finishBtn">FINISH &amp; REPLAY</button>
</div>
<div id="status">Connecting...</div>
<div id="angles"></div>

<script>
let ws = new WebSocket("ws://" + location.hostname + ":81/");
ws.onopen = () => { document.getElementById('status').innerText = "Connected"; };
ws.onclose = () => { document.getElementById('status').innerText = "Disconnected"; };

ws.onmessage = (evt) => {
  let msg = evt.data;
  if (msg.startsWith("ANGLES:")) {
    document.getElementById('angles').innerText = msg.substring(7);
  } else if (msg.startsWith("MSG:")) {
    document.getElementById('status').innerText = msg.substring(4);
  }
};

// ---- Set up one vertical joystick per joint ----
const radius = 55; // half of (containerHeight - dotHeight) = (160-50)/2

document.querySelectorAll('.joyBlock').forEach(block => {
  const joint = parseInt(block.dataset.joint);
  const container = block.querySelector('.joyContainer');
  const dot = block.querySelector('.joyDot');

  let dragging = false;
  let sendInterval = null;
  let lastVal = 0;

  function setDot(y) {
    dot.style.top = (55 + y) + "px";
  }

  function startDrag(e) {
    dragging = true;
    sendInterval = setInterval(() => ws.send("JOG:" + joint + ":" + lastVal), 50);
    e.preventDefault();
  }

  function endDrag() {
    if (!dragging) return;
    dragging = false;
    setDot(0);
    lastVal = 0;
    if (sendInterval) clearInterval(sendInterval);
    ws.send("JOG:" + joint + ":0");
  }

  function moveDrag(e) {
    if (!dragging) return;
    const rect = container.getBoundingClientRect();
    const clientY = e.touches ? e.touches[0].clientY : e.clientY;
    let y = clientY - (rect.top + rect.height / 2);
    y = Math.max(-radius, Math.min(radius, y));
    setDot(y);
    // Up = positive angle increase, so invert y
    lastVal = Math.round((-y / radius) * 100);
  }

  container.addEventListener('mousedown', startDrag);
  container.addEventListener('touchstart', startDrag);
  window.addEventListener('mouseup', endDrag);
  window.addEventListener('touchend', endDrag);
  window.addEventListener('mousemove', moveDrag);
  window.addEventListener('touchmove', moveDrag, {passive: false});
});

document.getElementById('recordBtn').onclick = () => ws.send("RECORD");
document.getElementById('finishBtn').onclick = () => {
  if (confirm("Finish recording and start replay?")) ws.send("FINISH");
};
</script>
</body>
</html>
)rawliteral";

void broadcastState() {
    String anglesMsg = "ANGLES:";
    for (int i = 0; i < NUM_JOINTS; i++) {
        anglesMsg += joints[i].name;
        anglesMsg += "=";
        anglesMsg += joints[i].cur;
        if (i < NUM_JOINTS - 1) anglesMsg += " ";
    }
    webSocket.broadcastTXT(anglesMsg);
}

void webSocketEvent(uint8_t client, WStype_t type, uint8_t* payload, size_t length) {
    if (type == WStype_TEXT) {
        String msg = String((char*)payload).substring(0, length);

        if (!recordingMode) return; // ignore jog/record commands while replaying

        if (msg.startsWith("JOG:")) {
            // Format: JOG:<jointIndex>:<value>
            int firstColon  = msg.indexOf(':');
            int secondColon = msg.indexOf(':', firstColon + 1);
            int joint = msg.substring(firstColon + 1, secondColon).toInt();
            int value = msg.substring(secondColon + 1).toInt();
            if (joint >= 0 && joint < NUM_JOINTS) {
                joystickValue[joint] = value;
            }
        } else if (msg == "RECORD") {
            recordCurrentPose();
            webSocket.broadcastTXT(String("MSG:Recorded pose ") + poseCount);
        } else if (msg == "FINISH") {
            recordingMode = false;
            webSocket.broadcastTXT(String("MSG:Replaying ") + poseCount + " poses...");
        }
    }
}

void handleJogging() {
    if (millis() - lastJogTime < JOG_INTERVAL_MS) return;
    lastJogTime = millis();

    bool anyMoved = false;
    for (int j = 0; j < NUM_JOINTS; j++) {
        if (joystickValue[j] == 0) continue;
        float normalized = constrain(joystickValue[j] / 100.0, -1.0, 1.0);
        int delta = (int)(normalized * JOG_SPEED_MAX);
        if (delta != 0) {
            setJointAngle(j, joints[j].cur + delta);
            anyMoved = true;
        }
    }
    if (anyMoved) broadcastState();
}

// ================= Replay mode =================
void replaySequence() {
    if (poseCount == 0) {
        webSocket.broadcastTXT("MSG:No poses recorded.");
        delay(2000);
        return;
    }
    for (int p = 0; p < poseCount; p++) {
        webSocket.broadcastTXT(String("MSG:Replaying pose ") + (p + 1) + "/" + poseCount);
        moveToPose(recordedSequence[p].angles);
        broadcastState();
        delay(holdMsBetweenPoses);
    }
    delay(1000);
}

// ================= Setup / Loop =================
void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);

    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);
    moveAllToHome();

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(400);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("Connected. IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("Open this IP in a browser on the same network.");

    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "text/html", index_html);
    });
    server.begin();

    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
}

void loop() {
    webSocket.loop();

    if (recordingMode) {
        handleJogging();
    } else {
        replaySequence();
    }
}
