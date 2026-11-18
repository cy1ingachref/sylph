# Sylph

An autonomous AI scout drone: a small aerial platform that accepts high-level
orders, scouts autonomously, analyzes terrain, detects plant/tree infections,
and returns home safely on low battery.

This repository is the project home for **Sylph**. It starts from the complete
build guide in [`docs/Autonomous_AI_Scout_Drone_Guide.md`](docs/Autonomous_AI_Scout_Drone_Guide.md).

## Architecture

```
Ground Station / App        (orders, mission planning, live monitoring)
        | MAVLink / MQTT / 4G
Companion Computer          (mission logic, onboard AI, camera, battery decisions)
        | MAVLink (UART / USB)
Flight Controller           (stabilization, GPS, RTH, failsafes)
```

- **Flight controller** handles low-level safety and flight (ArduPilot / PX4).
- **Companion computer** handles AI, mission logic, and high-level decisions
  (Jetson Orin Nano recommended, or Pi 5 + Coral TPU).
- Always keep a safety margin for return-to-home.

## Roadmap

- **Phase 0** — Foundations & safety: assembly, firmware, manual flight, failsafe tests.
- **Phase 1** — Autonomous flight: waypoints, automatic RTL, MAVLink link.
- **Phase 2** — Vision pipeline: downward camera, geotagged images, disease detection model,
  plus an offline phone/Jetson app that diagnoses leaves from a photo using the same
  on-device model (no network required in the field).
- **Phase 3** — Full mission autonomy: grid scouting, speech recognition with high-level
  commands (e.g. "scout the north field"), a live map of disease detections that updates
  in flight as each geotagged detection arrives, battery-aware planning.
- **Phase 4** — Field ready: real data, reporting, better sensors, optimization.

See the [full guide](docs/Autonomous_AI_Scout_Drone_Guide.md) for hardware,
software stack, model training, smart RTL, and safety/legal details.

## Status

Early planning. Use simulation (SITL / Gazebo or AirSim) before any real flight.
