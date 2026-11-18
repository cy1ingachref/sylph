# Autonomous AI Scout Drone – Complete Build Guide

**Goal:** Build a small autonomous drone with onboard AI that can:
- Accept high-level orders
- Hover and scout autonomously (no continuous controller)
- Analyze terrain
- Detect plant/tree infections
- Return home automatically when battery is low (with safety margin)

---

## Table of Contents
1. [Architecture Overview](#1-architecture-overview)
2. [Hardware Recommendations](#2-hardware-recommendations)
3. [Software Stack](#3-software-stack)
4. [Plant & Tree Infection Detection](#4-plant--tree-infection-detection)
5. [Smart Low-Battery Return-to-Home](#5-smart-low-battery-return-to-home)
6. [Resources](#6-resources)
7. [Step-by-Step Roadmap](#7-step-by-step-roadmap)
8. [Safety & Legal Instructions](#8-safety--legal-instructions)
9. [How to Use This Guide](#9-how-to-use-this-guide)

---

## 1. Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                     Ground Station / App                     │
│          (Orders, Mission Planning, Live Monitoring)         │
└────────────────────────────┬────────────────────────────────┘
                             │ MAVLink / MQTT / 4G
┌────────────────────────────▼────────────────────────────────┐
│                  Companion Computer (Jetson / Pi)            │
│  • Mission logic & high-level orders                         │
│  • Onboard AI (YOLO / MobileNet)                             │
│  • Camera processing & geotagging                            │
│  • Battery estimation & decision making                      │
└────────────────────────────┬────────────────────────────────┘
                             │ MAVLink (UART / USB)
┌────────────────────────────▼────────────────────────────────┐
│               Flight Controller (Pixhawk / Cube)             │
│  • Stabilization, GPS navigation, RTH, failsafes             │
│  • Motors / ESCs / Sensors                                   │
└─────────────────────────────────────────────────────────────┘
```

**Key Principles:**
- Flight controller handles low-level safety and flight.
- Companion computer handles AI, mission logic, and high-level decisions.
- Always keep a safety margin for return-to-home.

---

## 2. Hardware Recommendations

### Core Components (Realistic Budget: $600 – $1500)

| Component              | Recommended Options                          | Purpose                              | Notes |
|------------------------|----------------------------------------------|--------------------------------------|-------|
| Frame                  | 450–550 mm carbon fiber (F450 / F550 / Tarot)| Airframe                             | Good payload capacity |
| Flight Controller      | Pixhawk 6C, Cube Orange, or Matek H743       | Stabilization + Autonomous flight    | Use ArduPilot or PX4 |
| Motors + ESCs          | 920–1000 KV brushless + 30–40A 4-in-1 ESC    | Propulsion                           | Match to battery voltage |
| Battery                | 4S–6S LiPo 5000–10000 mAh                    | Power                                | Aim for 15–25+ min flight |
| GPS + Compass          | Here3, Beitian BN-880, or dual-band module   | Positioning & RTH                    | Critical for accuracy |
| Companion Computer     | **NVIDIA Jetson Orin Nano** (best) or Raspberry Pi 5 + Coral TPU | AI + Mission logic               | Jetson preferred for vision |
| Camera                 | Global shutter camera (Arducam / Pi Global Shutter) downward facing | Plant detection                  | Avoid rolling shutter blur |
| Optional Sensors       | Optical flow, cheap ToF/LiDAR, barometer     | Terrain following & obstacle awareness | Highly recommended |
| Telemetry              | SiK radio or 4G/LTE modem                    | Remote orders & monitoring           | — |

### Power Tips
- Use a separate BEC or power module for the companion computer.
- Monitor both voltage **and** current for accurate battery estimation.
- Keep total weight as low as possible — every gram reduces flight time.

---

## 3. Software Stack

| Layer                  | Recommended Tools                            | Notes |
|------------------------|----------------------------------------------|-------|
| Flight Firmware        | **ArduPilot** (recommended) or PX4           | Excellent battery failsafes & SmartRTL |
| Companion OS           | Ubuntu 22.04 / 24.04                         | — |
| Middleware             | ROS 2 (Humble or Jazzy)                      | Optional but powerful for complex logic |
| Communication          | MAVLink (pymavlink, MAVSDK, MAVROS)          | Standard for drones |
| Vision AI              | YOLOv8n / YOLOv11n or MobileNetV2/V3 + TensorRT / TFLite | Edge-optimized |
| Mission Planning       | QGroundControl / Mission Planner + custom Python | — |
| Order Interface        | Simple REST API, MQTT, or mobile app         | High-level commands |

---

## 4. Plant & Tree Infection Detection

### Recommended Models
- **YOLOv8n or YOLOv11n** → Best for real-time detection + localization of diseased areas.
- **MobileNetV2 / MobileNetV3** → Very lightweight classification (Healthy vs Diseased classes).

### Training Advice
1. Start with the **PlantVillage** dataset.
2. Collect your own aerial images as early as possible (critical — ground photos look different from drone photos).
3. Fine-tune the model on a mix of public + your own data.
4. Quantize the model (TensorRT on Jetson or TFLite INT8) for real-time performance.

### Output
- Detected disease class + confidence
- GPS coordinates of the detection
- Optional: severity estimation and field map generation

---

## 5. Smart Low-Battery Return-to-Home

Do **not** rely only on simple voltage thresholds.

**Better approach:**
1. Continuously estimate remaining flight energy (based on current draw + voltage).
2. Calculate energy required to return home (distance × current consumption + safety margin).
3. Trigger RTL when:  
   `Remaining Energy ≤ (Energy to Home + 20–30% safety margin)`

**ArduPilot / PX4 Features to Use:**
- Battery Failsafe
- SmartRTL (follows the path you came from — safer with obstacles)
- Geofence
- RTL altitude settings

Always test the failsafe thoroughly in a safe open area.

---

## 6. Resources

### Open-Source Projects
- AgriBot (ROS 2 + ArduPilot disease detection simulation)
- Smart-Agriculture GitHub repositories
- NathanBuildsDIY weed-killing drone (Raspberry Pi + TFLite)
- Various Jetson + Pixhawk autonomous drone projects on GitHub
- Awesome UAV AI / Autonomy curated lists

### Essential Documentation
- ArduPilot Documentation
- PX4 Documentation
- MAVSDK / pymavlink docs
- Ultralytics YOLO documentation
- NVIDIA Jetson developer guides

### Datasets
- PlantVillage (starting point)
- Your own drone-captured images (most important long-term)

---

## 7. Step-by-Step Roadmap

### Phase 0 – Foundations & Safety (2–4 weeks)
- [ ] Learn basic multirotor assembly and soldering
- [ ] Flash and configure ArduPilot or PX4
- [ ] Achieve stable manual flight + GPS position hold
- [ ] Test all failsafes (low battery, signal loss, geofence)

### Phase 1 – Autonomous Flight (1–2 months)
- [ ] Build reliable airframe with Pixhawk + GPS
- [ ] Fly waypoint missions successfully
- [ ] Implement automatic Return-to-Launch on low battery
- [ ] Connect companion computer and establish MAVLink communication

### Phase 2 – Vision Pipeline (1–2 months)
- [ ] Mount downward-facing global shutter camera
- [ ] Stream and save images with GPS tags
- [ ] Train and deploy a lightweight disease detection model onboard
- [ ] Verify detections are correctly geotagged
- [ ] Build phone / Jetson app that diagnoses a leaf from a photo **offline**
      (reuse the quantized on-device model; no network needed in the field)

### Phase 3 – Full Mission Autonomy (2–3 months)
- [ ] Implement lawnmower / grid scouting patterns
- [ ] Create high-level order interface (app or simple commands)
- [ ] Add speech recognition for high-level commands (e.g. "scout the north field")
      with a wake word, on-device or companion-computer STT, and a confirm/fail-safe
      step before any arming or movement
- [ ] Publish a live map of disease detections that updates as the drone flies:
      stream each geotagged detection over MQTT/MAVLink to the ground station and
      plot it incrementally (new markers appear in near real time, not just post-flight)
- [ ] Add battery-aware mission planning (abort early if needed)
- [ ] Basic terrain following / height holding

### Phase 4 – Field Ready (Ongoing)
- [ ] Collect real farm/orchard data and retrain models
- [ ] Improve reporting (maps, severity, recommendations)
- [ ] Add better sensors (optical flow, LiDAR, thermal)
- [ ] Optimize weight and flight time
- [ ] Full end-to-end testing in real conditions

---

## 8. Safety & Legal Instructions

**Mandatory Safety Rules:**
- Always test new software in simulation first (Gazebo + SITL or AirSim).
- Never fly over people, roads, or restricted areas.
- Keep a physical kill switch / RC override available during early testing.
- Test battery failsafes repeatedly in a large open field.
- Use a geofence from day one.
- Have a fire extinguisher nearby when charging LiPo batteries.

**Legal Notes (check your country):**
- Register the drone if required by weight.
- Follow local altitude and airspace restrictions.
- Respect privacy laws when flying over private land.
- Insurance is strongly recommended once you fly outside a private test area.

---

## 9. How to Use This Guide

1. **Read the entire document** once to understand the full scope.
2. Start with **Phase 0**. Do not skip safety and basic flight skills.
3. Buy hardware gradually — only purchase what you need for the current phase.
4. Document everything (photos, flight logs, model versions).
5. Use simulation heavily before risking real flights.
6. Join communities: ArduPilot Discord, PX4 Discuss, r/diydrones, Jetson forums.
7. Update this file with your own notes, BOM costs, and lessons learned.

### Suggested Next Actions
- Decide your budget and preferred companion computer (Jetson Orin Nano recommended).
- Create a detailed Bill of Materials spreadsheet.
- Set up ArduPilot SITL simulation this week.
- Start collecting or downloading PlantVillage + aerial plant images.

---

**Good luck with your build!**  
This is an ambitious but very achievable project. Many successful agricultural drones follow almost exactly this architecture.

*Last updated: November 2026*