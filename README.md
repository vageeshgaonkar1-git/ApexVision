# ApexVision — Blind-Curve Hazard Warning System

A real-time IoT safety system that detects vehicles approaching a blind hairpin curve from both sides, triggers visual/audio alerts, and streams live data to a cloud dashboard.

---

## Architecture

```
GY-53 (A) ──┐
             ├── ESP32 ── WiFi ── Firebase RTDB ── Vercel Dashboard
GY-53 (B) ──┘
```

---

## Hardware BOM

| Component        | Qty | Notes                            |
|------------------|-----|----------------------------------|
| ESP32 DevKit V1  | 1   | 38-pin, 3.3 V logic              |
| GY-53 (VL53L0X) | 2   | ToF sensor, 30–2000 mm, 25° FOV |
| LED (Yellow)     | 1   | One-side caution signal          |
| LED (Red)        | 1   | Both-side danger signal          |

---

## Pin Mapping (Prototype)

| Signal         | GPIO |
|----------------|------|
| I2C SDA        | 21   |
| I2C SCL        | 22   |
| XSHUT Sensor A | 18   |
| XSHUT Sensor B | 19   |
| RED LED        | 25   |
| YELLOW LED     | 26   |

Sensor A is reassigned from default `0x29` → `0x30` via XSHUT sequencing.  
Sensor B stays at default `0x29`.

---

## Firebase Setup

1. Open project **apexvision-8f1e4** in [Firebase Console](https://console.firebase.google.com/).
2. Navigate to **Realtime Database** → enable in **Test Mode** (Asia Southeast 1).
3. Database URL: `https://apexvision-8f1e4-default-rtdb.asia-southeast1.firebasedatabase.app`
4. No auth token needed in Test Mode.

### RTDB Structure

```json
{
  "nodes": {
    "node_01": {
      "telemetry": {
        "dA": 450, "dB": 620,
        "threshA": 300, "threshB": 300,
        "state": 0, "stallSec": 0,
        "modeOverride": "NONE",
        "rssi": -62, "uptimeSec": 3600,
        "sensorA_ok": true, "sensorB_ok": true
      },
      "config": {
        "modeOverride": "NONE",
        "threshA": 300,
        "threshB": 300,
        "stallTimeoutSec": 5,
        "recalibrate": false
      }
    }
  }
}
```

---

## Firmware Setup

### Dependencies (Arduino IDE / PlatformIO)

```
Adafruit_VL53L0X   >= 1.2.4
ArduinoJson        >= 7.x
WiFiClientSecure   (bundled with ESP32 core)
HTTPClient         (bundled with ESP32 core)
```

Install via Arduino Library Manager or add to `platformio.ini`:
```ini
lib_deps =
  adafruit/Adafruit VL53L0X
  bblanchon/ArduinoJson
```

### Credentials

Edit in `apexvision_node.ino` before flashing:
```cpp
const char* WIFI_SSID     = "YourSSID";
const char* WIFI_PASSWORD = "YourPassword";
```

### Flash

1. Open `firmware/apexvision_node/apexvision_node.ino` in Arduino IDE.
2. Board: **ESP32 Dev Module**, Port: your COM port.
3. Upload speed: 115200 baud.
4. Open Serial Monitor at 115200 to watch calibration + WiFi logs.

---

## Boot Calibration

On power-up the ESP32 samples 15 readings from each sensor on an **empty track** and sets:

```
threshA = clamp(baseline_A × 0.75,  100 mm, 900 mm)
threshB = clamp(baseline_B × 0.75,  100 mm, 900 mm)
```

EMA smoothing (`α = 0.65`) is applied to all subsequent readings.

Trigger a re-calibration remotely from the dashboard using the **Auto-Tare** button.

---

## State Machine (Firmware)

| State | Condition                          | LED Output                        |
|-------|------------------------------------|-----------------------------------|
| 0     | Clear — no vehicles                | All LEDs off                      |
| 1     | One side < threshold               | Yellow LED blinks @ 2 Hz          |
| 2     | Both sides < threshold             | Red LED blinks @ 5 Hz             |
| 3     | Both occupied ≥ stallTimeoutSec    | Red solid + Yellow fast-alternates|

---

## Remote Threshold Control

Thresholds can be changed live from the dashboard without reflashing:

1. Open **Remote Regulation** panel on the dashboard.
2. Drag **Threshold A** or **Threshold B** sliders (range: 100–900 mm).
3. Click **Apply to ESP32** — the value is PATCHed to Firebase config.
4. The ESP32 picks it up within **1.5 seconds** and uses the new threshold immediately.
5. The "Current firmware:" label under each slider confirms the value was received.

Thresholds are clamped on the firmware side to 100–900 mm regardless of what is sent.

---

## Speed Estimation (Dashboard)

Speed is calculated **client-side** using consecutive distance readings:

```
v = |D_oldest − D_newest| / (t_newest − t_oldest)   [mm/s]
speed_kmh = v × 0.0036
```

A **sliding window of 4 samples** (~2 seconds) is used instead of two consecutive points to reduce noise. The result is further smoothed with EMA (α = 0.35).

Displayed per sensor as:
- **km/h** value (1 decimal for < 10 km/h, integer above)
- Direction: `▼ Approaching`, `▲ Receding`, `■ Stationary`

> Accuracy note: At a 500 ms poll rate this gives directionally correct speed estimates. Expect ±3–8 km/h variation due to EMA smoothing and sensor quantisation noise. This is intended as a situational indicator, not precision measurement.

---

## Stall Detection & Emergency Alert

The dashboard runs a **client-side stall tracker** independent of the firmware state machine.

### How it works

- Timer starts the moment a sensor drops below threshold.
- **Back-to-back vehicles** (brief gap < 2 seconds between them) do **not** reset the timer — the gap is treated as part of the same occupancy event.
- Only when the sensor stays **clear for ≥ 2 seconds** is the timer genuinely reset. This correctly distinguishes continuous traffic flow from a single stalled vehicle.
- If either sensor remains occupied for **≥ 2 minutes** (120 seconds) continuously, a full-screen emergency overlay fires.

### Alert actions

| Button          | Action                                              |
|-----------------|-----------------------------------------------------|
| Lock Closure    | PATCHes `FORCE_RED_STOP` to firmware                |
| Dispatch Patrol | Logs the event and shows operator notification      |
| Clear           | Resets stall timer, removes override, closes overlay|

The stall timer display shows minutes and seconds (e.g. `Stall: 1m 45s`) once it exceeds 60 seconds.

---

## Dashboard Deployment (Vercel)

1. Push this repo to GitHub (already configured).
2. Go to [vercel.com](https://vercel.com) → **Add New Project** → Import `ApexVision`.
3. Settings:
   - Framework: **Other**
   - Root Directory: **`dashboard`**
   - Build Command: *(leave empty)*
   - Output Directory: *(leave empty)*
4. Click **Deploy**. Done — no build step required.

---

## Dashboard Features

| Feature | Description |
|---------|-------------|
| Live KPI cards | Sensor A/B distance, system state, stall timer |
| Speed cards | Estimated approach/recession speed per sensor in km/h |
| Range bars | Visual fill that turns red when sensor is below threshold |
| SVG hairpin twin | Top-down view with live vehicle indicators and speed labels |
| Live ToF chart | 60-point rolling distance stream, null gaps for OOR readings |
| Remote Regulation | Threshold sliders, mode override, stall timeout, Auto-Tare |
| Incident log | State-change log with speed, stall time, CSV export |
| Simulator mode | Built-in offline simulator for testing without hardware |

---

## Git Identity (this repo)

Commits use the GitHub no-reply email to comply with email privacy settings:
```
user.email = 271687128+vageeshgaonkar1-git@users.noreply.github.com
```
This is set at repo level only (`git config user.email`, not `--global`).

---

## License

MIT
