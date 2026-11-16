# Robot profile

Filled in on: 2026-11-15
Filled in by: Claude (Claude Code), interviewing the owner, Abdelfattah. Answers
from this session's interview plus decisions already recorded in his build
manual (V8 Carrier).

## The robot today

- How far along: parts on the bench. CAD, firmware and ROS 2 code exist (not
  yet pushed to his repo); nothing is assembled.
- Vacuum (exact model): Dyson V8 Cyclone (SV55), unmodified, carried in a slot.
- How the vacuum motor is switched on and off: a hobby servo on the robot taps
  the vacuum's red power button. One press each way (plain on/off); speed is
  set by hand on the chrome disc before a run.
- How the robot knows the motor is running: a sensor. An INMP441 microphone on
  the cradle, read by the ESP32-S3; on after 1.5 s of motor sound, off after
  3 s of quiet, one retry per transition. Exposed as vacuum_state in the
  firmware status.
- Known limit of that: not tested on hardware. A loud room could read as
  "on"; then the voice service stays deaf (the safe direction).

## The base

- Controller: ESP32-S3 (motors, encoders, IMU, bumper, cliff sensors, battery
  monitor, servo, emergency stop) plus a Raspberry Pi 5, 4 GB (ROS 2 Jazzy,
  Nav2, LiDAR).
- Software and language: ESP32 firmware in C/C++; ROS 2 nodes on the Pi.
- How commands reach it: ROS 2 topics on the Pi (for example
  /vacuum/set_power); a serial protocol from the Pi to the ESP32.
- Docking: no dock yet (planned, Stage 10).
- What "dock" does for now: drive to a fixed "home" spot saved on the map.

## Computer and sound

- Computer running the voice service: the Raspberry Pi 5 (4 GB) on the robot,
  Raspberry Pi OS 64-bit. Not set up yet.
- `pytest` result on it: not run yet (no Pi set up).
- Microphone: decided in kind, not bought: a USB array with built-in echo
  cancellation and noise suppression, XVF3800 class.
- Speaker: decided in kind, not bought: MAX98357A I2S amplifier with a 3 W
  speaker, or a small USB speaker.
- Where the microphone sits relative to the vacuum: planned at the robot's
  highest point, facing up, at least 10 cm from the speaker, away from the
  wheel motors and the vacuum exhaust. Exact distance from the Dyson motor
  open until the CAD is done.
- `input_device` / `output_device` from `robot-voice devices`: not yet.

## Network

- Internet from the voice computer: plain home Wi-Fi.
- How the vacuum or base is reached: not over the network. The voice service
  and the robot software both run on the same Pi; the link stays on
  127.0.0.1.
- Addresses or names: not needed (local link only).
- When Wi-Fi drops: voice may simply not work. Offline commands stay off
  ([offline] enabled = false), to save memory on the 4 GB Pi. Stopping does
  not depend on voice (see Safety).

## Voice, names, account

- Robot's name: Cuby.
- Wake word now / later: train "Cuby" now (openWakeWord notebook); "hey
  jarvis" only as a placeholder until that model works.
- Languages: English, German and Arabic, mixed freely.
- Gemini key in `.env`: no key yet. The owner will create a paid-tier key and
  put it in `.env` himself.
- Voice: one of Google's prebuilt voices.

## Safety

- What stops the robot while the vacuum runs: a physical stop button on top,
  wired to the ESP32's emergency stop (no Pi or network needed), plus the
  bumper and cliff sensors and the web page. Decided, not built.
- Where the first real test happens, and who is there: the living room, with
  the owner present.
- The agent may do without asking: read status; send stop, pause and dock;
  start the vacuum. Note: the owner chose to allow starts without asking. Until
  the robot has passed its first supervised runs, the agent still starts the
  vacuum or moves the robot only while the owner is present for that run
  (ONBOARDING.md rule).
- The agent must ask first: spending API money on test runs (live checks,
  `robot-voice check`, `say`, `simulate` with the cloud, `run`).
- Fragile things, places to keep out of: children's toys and pets on the
  floor: slow down or avoid. The household includes children.
- Privacy (owner's rule): camera images and face data never leave the Pi.
  Voice may go to Google after the wake word.

## Chosen path

- Path from `docs/CONNECT_YOUR_ROBOT.md`: "The base runs ROS": a ROS 2 node on
  the Pi that serves POST /command and GET /status on 127.0.0.1 with the
  guide's request checks (Host, Origin, Content-Type, body size), and maps
  them to the robot's ROS topics. vacuum_on comes from the microphone run
  sensor's status.
- Where the robot code lives: the owner's repository abdelfatta7/v8-carrier,
  under ros2_ws/ (being written by his coordinator's code agent; not pushed
  yet).
- `[robot]` settings in `config.toml`: adapter = "http",
  base_url = "http://127.0.0.1:8080" (port to confirm when the node exists).

## Goal of the current session

- Done means: this profile written and the path and steps agreed. No hardware
  today.
- Out of scope: anything on real hardware; spending API money.

## Open questions

- Microphone array and speaker: exact models, then their placement relative
  to the Dyson motor.
- Gemini paid-tier key: to be created by the owner.
- "Cuby" wake-word model: to be trained and tested.
- Port and exact topic names of the ROS bridge node.
- Pi 5 memory, 4 GB or 8 GB: open in the owner's build manual (item 31).
- Whether the microphone run sensor reports the vacuum reliably in a real
  room.

## Log of real runs

-
