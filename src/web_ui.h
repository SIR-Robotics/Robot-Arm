// ─── web_ui.h — Browser page served from / ──────────────────────────────────
// Game-pad layout:
//   • Left thumbstick (XY) → Base / Shoulder
//   • Right thumbstick (XY) → Elbow / Wrist Pitch
//   • Two vertical triggers → Wrist Roll, Gripper
//   • Keyboard: WASD (left stick), IJKL (right stick), Q/E (roll), Z/X (grip),
//               Space=REC, P=PLAY, S=STOP, C=CYCLE, H=HOME
//   • Browser Gamepad API: real Xbox/PS controller mapped to the same controls
#pragma once
extern const char HTML_PAGE[];
