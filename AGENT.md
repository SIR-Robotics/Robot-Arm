# Robot Arm Agent Guide

This repo is the firmware for the robot arm in the NSEP AI Robotics system.

The arm detects tags on blocks, maps each tag to a preset sequence, and executes the recorded pickup/place movement. Preset sequences are recorded through the arm UI or controls, then replayed by color/tag triggers.

## Current Role

- Detect block tags with HuskyLens.
- Map learned tag IDs to preset sequences.
- Run the matching recorded arm sequence.
- Avoid interrupting a sequence already in motion.

In the current code, `src/vision.cpp` maps tag IDs to colors and triggers the same color run path used by `/api/run/red`, `/api/run/yellow`, and `/api/run/blue`.

## Firmware Notes

- PlatformIO project for ESP32.
- Active build target is controlled by `build_src_filter` in `platformio.ini`.
- At the time of writing, the active source is `src/main_new.cpp` plus `arm`, `input`, `protocol`, `web_ui`, and `vision` modules.
- Do not rename source files to switch builds; edit `platformio.ini`.
- Keep servo limits, home positions, and timing constants explicit because the physical arm needs calibration.

## Control Context

- Web UI and WebSocket commands control manual jog, presets, recording, and playback.
- HTTP routes `/api/run/red`, `/api/run/yellow`, and `/api/run/blue` execute the stored color presets.
- Tag detection should call the existing preset trigger path, not duplicate arm movement logic.

## Editing Rules

- Prefer existing helpers in `arm`, `protocol`, `input`, `web_ui`, and `vision`.
- Keep preset recording/playback as the source of truth for movement sequences.
- Do not add a new dependency unless the installed libraries cannot reasonably do the job.
- For behavior changes, run the smallest PlatformIO build or targeted test that proves the firmware still compiles.
