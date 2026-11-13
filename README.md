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

| Component          | Qty | Notes                              |
|--------------------|-----|------------------------------------|
| ESP32 DevKit V1    | 1   | 38-pin, 3.3 V logic                |
| GY-53 (VL53L0X)   | 2   | ToF sensor, 30–2000 mm, 25° FOV   |
| LED (Yellow)       | 2   | Panel A & B approach warning       |
| LED (Red)          | 2   | Panel A & B stop signal            |
| Piezo Buzzer       | 1   | Active buzzer, 3.3 V               |
| Li-ion / LiPo pack | 1   | 3.7 V; voltage divider to GPIO 34 |
| Solar panel + MPPT | 1   | Optional outdoor power             |

---

## Pin Mapping

| Signal        | GPIO |
|---------------|------|
| I2C SDA       | 21   |
| I2C SCL       | 22   |
| XSHUT Sensor A| 16   |
| XSHUT Sensor B| 17   |
| PWM fallback A| 32   |
| PWM fallback B| 33   |
| LED Yellow A  | 25   |
| LED Red A     | 26   |
| LED Yellow B  | 27   |
| LED Red B     | 14   |
| Buzzer        | 18   |
| Battery ADC   | 34   |

---

## Firebase Setup

1. Go to [Firebase Console](https://console.firebase.google.com/) and open project **apexvision-8f1e4**.
2. Navigate to **Realtime Database** → enable in **Test Mode** (Asia Southeast 1 region).
3. Database URL: `https://apexvision-8f1e4-default-rtdb.asia-southeast1.firebasedatabase.app`
4. No auth token required in Test Mode.

### RTDB Structure

```json
{
  "nodes": {
    "node_01": {
      "telemetry": {
        "dA": 450, "dB": 620,
        "threshA": 500, "threshB": 500,
        "state": 0,
        "stallSec": 0,
        "modeOverride": "NONE",
        "vBat": 3.85,
        "rssi": -62,
        "uptimeSec": 3600,
        "sensorA_ok": true,
        "sensorB_ok": true
      },
      "config": {
        "modeOverride": "NONE",
        "threshA": 500,
        "threshB": 500,
        "stallTimeoutSec": 5,
        "pwmDuty": 128,
        "buzzerEnabled": true,
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

Install via Arduino Library Manager or `platformio.ini`:
```ini
lib_deps =
  adafruit/Adafruit VL53L0X
  bblanchon/ArduinoJson
```

### Credentials

Edit these lines in `apexvision_node.ino` before flashing:

```cpp
const char* WIFI_SSID     = "YourSSID";
const char* WIFI_PASSWORD = "YourPassword";
```

### Flash

1. Open `firmware/apexvision_node/apexvision_node.ino` in Arduino IDE.
2. Select board: **ESP32 Dev Module**, Port: your COM port.
3. Upload speed: **115200**.
4. Open Serial Monitor at 115200 baud to watch calibration + WiFi logs.

---

## PWM Fallback Mode

If I2C sensors are unavailable, uncomment:

```cpp
#define USE_PWM_MODE
```

Connect GY-53 PWM output pins to GPIO 32 (Sensor A) and GPIO 33 (Sensor B).  
Pulse width to distance: **10 µs = 1 mm**.

---

## State Machine

| State | Condition                        | Output                             |
|-------|----------------------------------|------------------------------------|
| 0     | Clear — no vehicles              | All LEDs off                       |
| 1     | One side < threshold             | Opposite Yellow LED blinks @ 2 Hz  |
| 2     | Both sides < threshold           | Both Red LEDs + Buzzer strobe 5 Hz |
| 3     | Both occupied ≥ stallTimeoutSec  | Alternating Red+Yellow hazard      |

---

## Dashboard Deployment (Vercel)

1. Push this repo to GitHub (already configured).
2. Import the **`dashboard/`** folder in [vercel.com](https://vercel.com) as a new project.
3. Framework: **Other** (static site).
4. Root directory: `dashboard`.
5. Deploy — Vercel will serve `index.html` at your `.vercel.app` URL.

No build step required — the dashboard is a single self-contained HTML file.

---

## Remote Regulation

From the dashboard **Remote Regulation Panel** you can:

- Override node state: `FORCE_RED_STOP`, `FORCE_YELLOW_CAUTION`, `ECO_SLEEP`, or `NONE`
- Adjust detection thresholds (180–900 mm per sensor)
- Change stall timeout (2–30 s)
- Toggle buzzer
- Trigger a remote Auto-Tare (recalibration on empty track)

All changes are written via `PATCH /nodes/node_01/config.json` and picked up by the ESP32 within 1.5 s.

---

## Emergency Response (State 3)

When a vehicle stalls for ≥ stallTimeoutSec the dashboard shows a flashing crimson banner with three actions:

| Button           | Action                                               |
|------------------|------------------------------------------------------|
| Lock Red Closure | PATCHes `FORCE_RED_STOP` — holds both panels red    |
| Dispatch Patrol  | Logs the event and shows an alert for operators     |
| Clear            | Resets override to `NONE`, dismisses banner         |

An optional Web Audio siren can be toggled from the banner.

---

## License

MIT — see [LICENSE](LICENSE) if present.
