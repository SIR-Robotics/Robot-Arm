# RoboArm — 6-DOF Desktop Robot Arm Controller

PlatformIO firmware for a 6-DOF desktop robot arm driven by an ESP32. Supports physical joystick control, a browser-based WebSocket UI, pose recording/playback, and serial debugging.

---

## Hardware

| Component | Detail |
|-----------|--------|
| MCU | ESP32 (esp32dev) |
| PWM driver | Adafruit PCA9685, I2C addr `0x40`, SDA=21, SCL=22 |
| Servos | 6× hobby servo on PCA9685 channels 0–5 |
| Joystick | Analog (VRX=GPIO34, VRY=GPIO35), SW=GPIO32 |
| Buttons | REC=GPIO33, PLAY=GPIO25, CLR=GPIO26, CYCLE=GPIO27 (all INPUT_PULLUP) |

All joystick and button pins are on ADC1 to avoid the ADC2/WiFi conflict on ESP32.

### Joint table

| # | Name | PCA9685 ch | Range | Home | Inverted |
|---|------|-----------|-------|------|---------|
| 0 | Base | 5 | 0–180° | 90° | No |
| 1 | Shoulder | 0 | 30–150° | 90° | Yes |
| 2 | Elbow | 1 | 0–135° | 90° | No |
| 3 | Wrist Pitch | 2 | 0–180° | 90° | No |
| 4 | Wrist Roll | 3 | 0–180° | 90° | No |
| 5 | Gripper | 4 | 0–90° | 45° | No |

---

## Build & Flash

Single PlatformIO environment: `esp32dev`.

```powershell
pio run                  # compile
pio run -t upload        # flash over USB
pio run -t monitor       # 115200 baud serial monitor
pio run -t clean         # clean build artifacts
```

The `pio` binary is at `$env:USERPROFILE\.platformio\penv\Scripts\pio.exe`.

### Active source file

`platformio.ini` uses `build_src_filter` to select which `.cpp` gets compiled. `src/main.cpp` is excluded. Check the `build_src_filter` line in `platformio.ini` to confirm which file is the active build target — currently `src/main_new.cpp`. Do not rename files to swap the active build; edit `build_src_filter` instead.

---

## WiFi

The firmware runs in **STA mode** and connects to a hardcoded SSID (`ASEM Training`). Once connected, the arm's IP is printed over serial. Open that IP in a browser to load the web UI.

---

## Web UI

A single-page app served from the ESP32 over HTTP. It connects back to the ESP32 via WebSocket (`/ws`) for real-time control. Features:

- **Touch joystick** — on-screen jog pad controlling one joint pair at a time
- **Joint mode selector** — switch between Base+Shoulder / Elbow+Wrist Pitch / Wrist Roll+Gripper
- **Preset buttons** — Home, Ready, Pick, Place, Red, Yellow, Blue
- **Pose recorder** — Record, Play, Cycle, Clear, Save, Load, per-pose rename

### HTTP API

Use the ESP32 IP printed on serial as the host.

| Method | Path | Action | Response |
|--------|------|--------|----------|
| `GET` | `/` | Serve the web UI | HTML |
| `GET` | `/poses.json` | Download the live recorded sequence | `{"t":"p","c":...,"i":[...]}` |
| `POST` | `/poses.json` | Replace the live recorded sequence from JSON and save it | `{"ok":true,"n":<count>}` |
| `GET`/`POST` | `/api/run/red` | Run preset slot `4` (`Red`) | `{"ok":true,"preset":"red","len":<poses>}` |
| `GET`/`POST` | `/api/run/yellow` | Run preset slot `5` (`Yellow`) | `{"ok":true,"preset":"yellow","len":<poses>}` |
| `GET`/`POST` | `/api/run/blue` | Run preset slot `6` (`Blue`) | `{"ok":true,"preset":"blue","len":<poses>}` |

Example:

```bash
curl -X POST http://ROBOT_IP/api/run/red
```

The color routes call `runRed()`, `runYellow()`, and `runBlue()`. Record a movement in the UI, then save the recording into preset `4`, `5`, or `6`; the API runs whatever sequence is stored there.

### WebSocket protocol

Inbound messages use a zero-allocation `TAG:arg:arg` text format (no JSON parsing):

| Tag | Args | Action |
|-----|------|--------|
| `JG` | joint, value (−100..100) | Jog joint continuously |
| `JX` | joint1, value1, joint2, value2 | Jog two joints from one stick frame |
| `SV` | joint, angle | Set servo to absolute angle |
| `PR` | index | Play preset `0..6` (`4=Red`, `5=Yellow`, `6=Blue`) |
| `SP` | index | Save current joint pose into preset |
| `SQ` | index | Save current recording sequence into preset |
| `PN` | index, name | Rename preset |
| `MD` | mode (0–2) | Set joystick pair mode |
| `RC` | — | Record current pose |
| `RN` | index, name | Rename pose |
| `GT` | index | Go to recorded pose |
| `PY` | — | Start one-shot playback |
| `ST` | — | Stop playback |
| `CY` | — | Toggle cycle mode |
| `CL` | — | Clear all poses |
| `SA` | — | Save poses to flash |
| `LD` | — | Load poses from flash |
| `MV` | x, y, z[, ry] | IK move, optionally fixed pitch |
| `ID` | dx, dy, dz, dry, drx | IK jog input (−100..100 per axis) |
| `IK` | 0 or 1 | Toggle IK control mode |

Outbound JSON uses three message types: `{"t":"s",...}` for status, `{"t":"p",...}` for the live pose list, and `{"t":"pl",...}` for preset names/lengths. They are built with `snprintf` into static buffers — no heap allocation on the hot path.

---

## Physical Controls

| Control | Action |
|---------|--------|
| Joystick X/Y | Move the two joints in the active pair |
| Joystick SW (push) | Cycle to the next joint pair |
| BTN_REC | Record current pose |
| BTN_PLAY | Toggle one-shot playback |
| BTN_CLR | Clear all recorded poses |
| BTN_CYCLE | Toggle continuous cycle playback |

Joystick center is auto-calibrated at startup (64-sample average). Deadzone: ±300 counts. Update rate: every 20 ms.

<img width="1920" height="936" alt="image" src="https://github.com/user-attachments/assets/0b33519a-7a54-425e-b52f-cccc793670e3" />

---

## Pose Recording & Persistence

Up to **50 poses** can be recorded. Each pose stores all 6 joint angles and a label (up to 19 chars). Two playback modes:

- **One-shot** (`isPlaying`) — runs through the sequence once and stops.
- **Cycle** (`isCycling`) — loops indefinitely. Starting one cancels the other.

Step interval: 1200 ms per pose. Persistence via the ESP32 `Preferences` library (namespace `roboarm`, keys `len` and `seq`). Recordings survive power cycles. Poses are saved to flash on explicit Save; renames write through immediately.

---

## Serial Commands (115200 baud)

Connect with `pio run -t monitor`. Send `HELP` for the full menu.

| Command | Description |
|---------|-------------|
| `TEST` | Sweep every joint lo → home → hi → home |
| `STATUS` | Dump joint table, angles, joystick mode, recording state |
| `S <joint> <angle>` | Direct servo write |
| `INVERT <0-5>` | Toggle a joint's invert flag at runtime |
| `PRESET <0-6>` | Run preset, including `4=Red`, `5=Yellow`, `6=Blue` |
| `RED` / `YELLOW` / `BLUE` | Run the matching color preset |
| `H` | Run the Home preset |

`INVERT` is useful when wiring a servo in reverse — toggle until motion direction is correct, then update the `joints[]` table and reflash.

---

## Architecture

**Single-loop cooperative design.** `loop()` runs four pollers each tick: drain WS queue → joystick → buttons → playback. No application FreeRTOS tasks; only the network stack runs on the second core.

**Concurrency at I2C.** The AsyncWebServer callback fires on the network core. WS commands are copied into a `WsMsg` struct and sent over a FreeRTOS queue. `loop()` drains the queue and dispatches through `processWsCmd` so all servo writes are single-threaded on the main core.

- `setServo(i, angle)` — mutex-protected I2C write, updates logical state. Use from WS handlers and presets.
- `sendPWM(i, angle)` — no mutex, no state sync. Only safe from `processJoystick()` inside `loop()`.

**Angle → PWM.** A 181-entry lookup table (`pwmTable[0..180]`) maps degrees to PCA9685 counts. Built once in `setup()` from `SERVO_MIN_US`/`SERVO_MAX_US`. Servo writes use this table via `toCounts(angle)`.

**State broadcast.** Mutations set `pendingBroadcast = true`. The bottom of `loop()` coalesces and sends — rate-limited to 50 ms while the joystick is active.

--

## Dependencies

Managed by PlatformIO via `lib_deps` in `platformio.ini`:

- `adafruit/Adafruit PWM Servo Driver Library` ^3.0.2
- `ESP32Async/ESPAsyncWebServer` (GitHub)
- `ESP32Async/AsyncTCP` (GitHub)
