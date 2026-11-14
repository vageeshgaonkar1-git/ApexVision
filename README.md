# 🚦 ApexVision — Autonomous Blind-Curve Hazard Warning & Cloud Command System

> **Built for COLOSSUS 3.0** — 24-Hour National Hackathon · Hardware / IoT & Robotics Track  
> **Team OutOfMemory:** Narendra Babu M · Alan Joy V · Vageesh Vishwanath Gaonkar

**Live Dashboard:** [https://apexvision-navinya.vercel.app](https://apexvision-navinya.vercel.app)  
**Repository:** [https://github.com/vageeshgaonkar1-git/ApexVision](https://github.com/vageeshgaonkar1-git/ApexVision)

---

ApexVision is an off-grid, solar-powered road-safety node engineered for zero-visibility mountain hairpin curves (such as the Agumbe Ghat and Western Ghats corridors). Unlike passive convex mirrors that fog over in monsoon mist or blind drivers with oncoming high-beam glare at night, ApexVision uses **dual 940 nm Class-1 eye-safe Time-of-Flight laser sensors** (STMicroelectronics VL53L0X) and a **dual-core FreeRTOS architecture** on the Espressif ESP32 to detect approaching vehicles in **< 30 ms**, trigger graded visual warnings at the curve apex, and stream live telemetry — distance, speed in both km/h and mm/sec, and stall emergencies — to a cloud command centre deployed on Vercel.

---

## 📌 Table of Contents

1. [Problem Statement & Why Convex Mirrors Fail](#1-problem-statement)
2. [End-to-End System Architecture](#2-system-architecture)
3. [Hardware Bill of Materials](#3-hardware-bom)
4. [Complete Pinout & Power Stage Wiring](#4-pinout--wiring)
5. [Dual-Core FreeRTOS Firmware Architecture](#5-firmware-architecture)
6. [Hazard State Machine & Speed Estimation Math](#6-state-machine--speed-math)
7. [Step-by-Step Setup & Deployment Guide](#7-setup--deployment)
8. [Cloud Command Dashboard Features](#8-dashboard-features)

---

## 🌫️ 1. Problem Statement

India records **1,68,491 annual road fatalities** (MoRTH Road Accident Report). Epidemiological studies of mountain ghat corridors document an Accident Severity Index of **62 deaths per 100 crashes**, with **69.5% driven by sightline obstruction** and path-judgment errors on sharp curves *(NIH PMC Hilly Regions Traffic Study)*.

| Hazard Condition | Traditional Convex Mirror | Ultrasonic / PIR Sensor | ApexVision (940 nm Laser ToF + IoT) |
|---|---|---|---|
| Heavy Monsoon Fog & Mist | ❌ Condenses into an opaque mirror | ⚠️ Acoustic damping in heavy rain | ✅ Active 940 nm laser ranging + self-emissive LEDs |
| Nighttime High-Beam Glare | ❌ Blinds oncoming drivers via reflection | ✅ Unaffected by light | ✅ Zero optical reflection; clear colour-coded alerts |
| Static Cliff / Guardrail Tare | ❌ Fixed geometry | ❌ Wide 30°–60° acoustic cone hits rocks | ✅ Narrow 25° optical FOV + 15-sample Auto-Tare |
| Stalled / Broken-Down Vehicle | ❌ No memory or remote alert | ❌ No cloud connectivity | ✅ Automated Stall Timer + Cloud Emergency Dispatch |

---

## 🏗️ 2. System Architecture

```
┌────────────────────────────────────────────────────────────────────────────┐
│               OFF-GRID SOLAR & 2S BATTERY POWER STAGE                      │
│  [Solar Panel 9–12V] ──► [2S BMS + 2×18650 Li-ion] ──► [LM2596 Buck 5V]   │
└──────────────────────────────────────┬─────────────────────────────────────┘
                                       │ 5.0 V → VIN / GND
                                       ▼
┌────────────────────────────────────────────────────────────────────────────┐
│                     ESP32 DEVKIT V1 (DUAL-CORE MCU)                        │
│                                                                            │
│  ┌──────────────────────────────────────┐  ┌──────────────────────────┐   │
│  │  CORE 1: Real-Time Safety Loop       │  │  CORE 0: Cloud Sync Task │   │
│  │  (33 Hz, deterministic)              │  │  (non-blocking)          │   │
│  │  • Dual VL53L0X ToF Polling (I²C)    │◄►│  • Persistent TLS        │   │
│  │  • OOR > 2000 mm Snap-EMA filter     │  │  • PUT /telemetry 650 ms │   │
│  │  • 4-State Hazard Logic + Stall      │  │  • GET /config  1500 ms  │   │
│  │  • 2 Hz / 5 Hz Traffic Module Driver │  │  • FreeRTOS Mutex Sync   │   │
│  └────────────▲────────────────┬────────┘  └───────────────┬──────────┘   │
└───────────────┼────────────────┼──────────────────────────┼───────────────┘
                │ Shared I²C Bus │ GPIO 18 / 19             │ HTTPS REST
       ┌────────┴───────┐        ▼                          ▼
       │                │  ┌──────────────┐     ┌─────────────────────┐
  ┌────┴──────┐ ┌───────┴──┐ │ 3-Light     │     │ Firebase RTDB       │
  │ Sensor A  │ │ Sensor B  │ │ Traffic     │     └──────────┬──────────┘
  │ I²C 0x30  │ │ I²C 0x29  │ │ Module      │               │ 500 ms poll
  │ XSHUT G16 │ │ XSHUT G17 │ │ Y: GPIO 18  │               ▼
  └───────────┘ └───────────┘ │ R: GPIO 19  │  ┌─────────────────────┐
                               └──────────────┘  │  Vercel Dashboard   │
                                                  └─────────────────────┘
```

---

## 🧰 3. Hardware BOM

| Subsystem | Component | Qty | Specifications |
|---|---|---|---|
| Compute & Wireless | Espressif ESP32 DevKit V1 | 1 | 38-pin, dual-core Tensilica LX6 @ 240 MHz, 3.3 V logic, integrated Wi-Fi |
| Optical Sensing | GY-53 / VL53L0X ToF Laser Sensor | 2 | 940 nm VCSEL, 30–2000 mm range, 25° FOV |
| Visual Signaling | 3-Light Traffic LED Module | 1 | Common-cathode; Y and R pins active |
| Energy Harvesting | Polycrystalline Solar Panel | 1 | 9–12 V input to 2S BMS |
| Energy Storage | 18650 Li-ion Cells | 2 | 2S series (7.4 V nominal, 8.4 V full) |
| Battery Protection | 2S Li-ion BMS Board | 1 | Overcharge, over-discharge, short-circuit, cell balance |
| Voltage Regulation | LM2596 DC-DC Buck Converter | 1 | 7.4–8.4 V → regulated 5.0 V @ 3 A for ESP32 VIN |

---

## 🔌 4. Pinout & Wiring

### 4.1 Solar + 2S BMS + LM2596 Power Stage

| Stage | Source Terminal | Destination | Notes |
|---|---|---|---|
| Solar Charging | Solar Plate + (via Schottky diode) / − | 2S BMS P+ / P− | 9–12 V input charges the 2S pack |
| Battery Negative | Cell 1 − (0 V) | 2S BMS B− | Negative end of series string |
| Mid-Point Tap | Cell 1 + / Cell 2 − (3.7 V) | 2S BMS BM | Balance sensing node |
| Battery Positive | Cell 2 + (7.4–8.4 V) | 2S BMS B+ | Positive end of series string |
| BMS Protected Output | 2S BMS P+ / P− | LM2596 IN+ / IN− | Protected pack voltage to buck |
| Regulated 5 V Rail | LM2596 OUT+ / OUT− | ESP32 VIN / GND | ⚠️ Calibrate trimpot to **5.00 V** before connecting ESP32 |

### 4.2 Sensor A — Left Approach (GY-53 / VL53L0X)

| Sensor Pin | ESP32 GPIO | Notes |
|---|---|---|
| VCC | 3V3 | Onboard 3.3 V LDO |
| GND | GND | Common ground |
| SDA | GPIO 21 | Shared I²C bus (`Wire.begin(21, 22)`) |
| SCL | GPIO 22 | Shared I²C clock |
| XSHUT | **GPIO 16** | Pulled HIGH first → reassigns Sensor A from 0x29 to **0x30** |
| GPIO1 | Leave Empty (NC) | Unused interrupt |

### 4.3 Sensor B — Right Approach (GY-53 / VL53L0X)

| Sensor Pin | ESP32 GPIO | Notes |
|---|---|---|
| VCC | 3V3 | Onboard 3.3 V LDO |
| GND | GND | Common ground |
| SDA | GPIO 21 | Shared bus (same as Sensor A) |
| SCL | GPIO 22 | Shared clock (same as Sensor A) |
| XSHUT | **GPIO 17** | Held LOW while A reassigns, then HIGH → retains **0x29** |
| GPIO1 | Leave Empty (NC) | Unused interrupt |

### 4.4 3-Light Traffic Module

| Module Pin | ESP32 GPIO | Behaviour |
|---|---|---|
| Y (Yellow) | **GPIO 18** | One-side caution: 2 Hz blink / Stall: alternating flash |
| R (Red) | **GPIO 19** | Bilateral danger: 5 Hz strobe / Stall: solid hold |
| GND | GND | Common return |
| G (Green) | Leave Empty (NC) | Road-clear = all LEDs off (solar conservation) |

---

## ⚙️ 5. Firmware Architecture

Standard single-loop ESP32 sketches freeze for **200–1500 ms** during HTTPS TLS handshakes, causing LED strobes to stutter and missing fast vehicles. ApexVision isolates real-time sensing from cloud I/O across the two physical cores:

| Core | Thread | Responsibilities | Timing |
|---|---|---|---|
| **Core 1** | Arduino `loop()` | Polls Sensor A (0x30) & B (0x29), runs snap-EMA filter, 4-state hazard machine, drives GPIO 18/19 | Deterministic ~30 ms cycle (33 Hz), zero network blocking |
| **Core 0** | `firebaseTask` | HTTPS PUT `/telemetry` (650 ms) + GET `/config` (1500 ms) via persistent TCP keep-alive (`http.setReuse(true)`) | Async, via `telemMutex` & `configMutex` |

### Fault Tolerance

- **I²C Bus Lockup Recovery** — `Wire.setTimeOut(50)`: resets the I²C peripheral within 50 ms on clock-stretch hang from capacitive cable noise
- **Self-Healing Sensor Watchdog** — if `sensorA_ok` or `sensorB_ok` drops, firmware toggles XSHUT (GPIO 16/17) every 5 s to re-enumerate without rebooting
- **Snap-Recovery EMA Filter** — readings > 2000 mm are flagged OOR (`2500.0f`). The instant a vehicle enters ≤ 2000 mm, EMA bypasses exponential decay and snaps to the first valid reading, then resumes α = 0.65 smoothing
- **ECO_SLEEP Mode** — disables Wi-Fi radio (saves 80–240 mA) and enters `esp_light_sleep_start()` between wake cycles

---

## 📊 6. State Machine & Speed Math

### 6.1 Startup Auto-Tare Calibration

On boot (or remote Auto-Tare from dashboard), the node samples 15 readings per sensor on an empty track and sets dynamic thresholds:

```
thresh(A,B) = clamp( baseline(A,B) × 0.75,  100 mm,  900 mm )
```

### 6.2 Four-State Hazard Logic

| State | Name | Trigger Condition | Traffic Module Output |
|---|---|---|---|
| **0** | CLEAR | dA ≥ threshA **AND** dB ≥ threshB | All LEDs OFF (solar conservation) |
| **1** | ONE-SIDE CAUTION | Exactly one side < thresh | Yellow (GPIO 18) blinks at **2 Hz** (250 ms toggle) |
| **2** | BILATERAL DANGER | dA < threshA **AND** dB < threshB | Red (GPIO 19) strobes at **5 Hz** (100 ms toggle) |
| **3** | STALL / BREAKDOWN | Occupancy ≥ stallTimeoutSec (5 s node / 120 s cloud) | Red solid + Yellow fast-alternating + **Dashboard Emergency Overlay** |

### 6.3 Simultaneous Dual-Unit Speed Estimation

The dashboard computes velocity across a **5-sample sliding window** (duplicate Firebase frames ignored):

```
v(mm/s) = |D_oldest − D_newest| / (t_newest − t_oldest)

v(km/h) = v(mm/s) × 0.0036
```

| Distance Reading | Distance Card | Speed Card |
|---|---|---|
| > 2000 mm (OOR sentinel 2500) | **Out of Range** | `0.00 km/h • Out of Range` / `0 mm/sec` |
| 30–2000 mm (vehicle present) | Exact mm value | km/h (↓ Approaching / ↑ Receding / • Stationary) + mm/sec below |

---

## 🚀 7. Setup & Deployment

### Step 1 — Power Stage Calibration ⚠️ Critical First

1. Wire 2× 18650 Li-ion cells (B−, BM, B+) to the 2S BMS
2. Connect BMS output (P+, P−) to LM2596 input (IN+, IN−)
3. **Before connecting the ESP32:** adjust the LM2596 brass trimpot while measuring OUT+ / OUT− until a multimeter reads exactly **5.00 V DC**
4. Connect OUT+ → ESP32 VIN, OUT− → ESP32 GND

### Step 2 — Configure Firebase RTDB

1. Open project **apexvision-8f1e4** in [Firebase Console](https://console.firebase.google.com/)
2. Create Realtime Database in **Asia Southeast 1** region
3. Set Security Rules to Test Mode:

```json
{
  "rules": {
    ".read": true,
    ".write": true
  }
}
```

4. Database URL: `https://apexvision-8f1e4-default-rtdb.asia-southeast1.firebasedatabase.app`

RTDB JSON schema used by both firmware and dashboard:

```json
{
  "nodes": {
    "node_01": {
      "telemetry": {
        "dA": 450, "dB": 620,
        "threshA": 300, "threshB": 300,
        "state": 0, "stallSec": 0.0,
        "modeOverride": "NONE",
        "rssi": -62, "uptimeSec": 3600,
        "sensorA_ok": true, "sensorB_ok": true
      },
      "config": {
        "modeOverride": "NONE",
        "threshA": 300, "threshB": 300,
        "stallTimeoutSec": 5,
        "recalibrate": false
      }
    }
  }
}
```

### Step 3 — Flash the ESP32 Firmware

1. Install **Arduino IDE 2.x** with `esp32 by Espressif Systems` board package (≥ 3.x)
2. Install libraries via Library Manager:
   - `Adafruit_VL53L0X` ≥ 1.2.4
   - `ArduinoJson` ≥ 7.x
3. Open `firmware/apexvision_node/apexvision_node.ino` and set credentials:

```cpp
const char* WIFI_SSID     = "Your_Hotspot_SSID";
const char* WIFI_PASSWORD = "Your_Hotspot_Password";
```

4. Board: **ESP32 Dev Module** · Upload Speed: **115200** · Click **Upload**
5. Open Serial Monitor at 115200 baud — keep the track **completely clear** during the 15-sample boot calibration so thresholds lock to the real background baseline

### Step 4 — Deploy Dashboard to Vercel

1. Push repo to GitHub
2. Open [vercel.com](https://vercel.com) → **Add New Project** → Import `ApexVision`
3. Settings:
   - Framework Preset: **Other**
   - Root Directory: **`dashboard`**
   - Build Command / Output Directory: *(leave blank)*
4. Click **Deploy** — live globally over HTTPS in under 15 seconds

---

## 🖥️ 8. Dashboard Features

| Module | Capability |
|---|---|
| **Live Telemetry KPI Cards** | Real-time Sensor A & B distance (30–2000 mm or **Out of Range** above 2000 mm), active hazard state, continuous stall timer |
| **Simultaneous km/h & mm/sec** | Directional velocity (↓ Approaching, ↑ Receding, • Stationary) in both km/h and mm/sec on each approach card |
| **Interactive Hairpin Digital Twin** | Top-down SVG of the blind curve with animated vehicle markers and live speed badges |
| **60-Point Rolling ToF Chart** | Dual-trace Chart.js time-series with clean gap handling when lanes are clear (OOR) |
| **Two-Way Remote Regulation** | Live sliders for Threshold A/B (100–900 mm), Stall Timeout, Mode Override (FORCE_RED_STOP, FORCE_YELLOW_CAUTION, ECO_SLEEP), one-click Auto-Tare |
| **Traffic vs. Stall Discriminator** | Ignores < 2 s gaps between back-to-back vehicles; triggers full-screen **Emergency Dispatch Overlay** (Lock Closure / Dispatch Patrol / Clear) on sustained blockage |
| **Stale-Node Watchdog** | Flags **⚠ Node Offline / Stale** if `uptimeSec` stops incrementing for 4 s — prevents showing stale cached Firebase data as Live |
| **Offline Simulator Mode** | Built-in hardware-free demo that sweeps sensors through full OOR → approach → recession cycles |

---

## 📄 License

Distributed under the **MIT License**.  
Built with pride by **Team OutOfMemory** for **COLOSSUS 3.0**.
