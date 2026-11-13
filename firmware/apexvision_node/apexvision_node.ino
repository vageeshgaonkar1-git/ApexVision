// =============================================================
//  ApexVision — Blind-Curve Hazard Warning Node
//  Target : ESP32 (38-pin or DevKit V1)
//
//  Hardware connections (final wiring):
//
//  Sensor A (left approach — GY-53 / VL53L0X)
//    VCC   → 3V3
//    GND   → GND
//    SDA   → GPIO 21
//    SCL   → GPIO 22
//    XSHUT → GPIO 16   (reassigns Sensor A to I2C 0x30)
//    GPIO1 → leave empty
//
//  Sensor B (right approach — GY-53 / VL53L0X)
//    VCC   → 3V3
//    GND   → GND
//    SDA   → GPIO 21   (shared I2C bus)
//    SCL   → GPIO 22   (shared I2C bus)
//    XSHUT → GPIO 17   (Sensor B stays at 0x29)
//    GPIO1 → leave empty
//
//  3-light traffic module
//    Y (Yellow) → GPIO 18   (one-side caution)
//    R (Red)    → GPIO 19   (both-side danger / stall)
//    GND        → GND
//
//  Requires: Adafruit_VL53L0X, WiFiClientSecure, HTTPClient, ArduinoJson
//  Firebase RTDB: apexvision-8f1e4 (Test Mode, no auth)
//
//  Core assignment:
//    Core 1 (Arduino loop()) — ToF reads, EMA, state machine, LEDs (~40 Hz)
//    Core 0 (FreeRTOS task)  — Firebase PUT telemetry + GET config (non-blocking)
// =============================================================

#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ─────────────────────────────────────────────
//  WiFi Credentials
// ─────────────────────────────────────────────
const char* WIFI_SSID     = "Alan";
const char* WIFI_PASSWORD = "alanjoy252005";

// ─────────────────────────────────────────────
//  Firebase RTDB  (Test Mode — no auth token)
// ─────────────────────────────────────────────
const char* FB_BASE = "https://apexvision-8f1e4-default-rtdb.asia-southeast1.firebasedatabase.app";
const char* NODE_ID = "node_01";

// ─────────────────────────────────────────────
//  Hardware Pins
// ─────────────────────────────────────────────
#define SDA_PIN     21
#define SCL_PIN     22
#define XSHUT_A     16      // Sensor A XSHUT → reassigned to 0x30
#define XSHUT_B     17      // Sensor B XSHUT → stays at 0x29
#define ADDR_A      0x30
#define ADDR_B      0x29

// 3-light traffic module
#define YELLOW_LED  18      // Y pin → one-side caution
#define RED_LED     19      // R pin → both-side danger / stall

// ─────────────────────────────────────────────
//  Calibration & Detection
// ─────────────────────────────────────────────
#define DISTANCE_THRESHOLD  150
#define CALIB_SAMPLES        15
#define THRESH_RATIO       0.75f
#define THRESH_MIN_MM       100
#define THRESH_MAX_MM       900
#define EMA_ALPHA          0.65f

float threshA = DISTANCE_THRESHOLD;
float threshB = DISTANCE_THRESHOLD;
float emaA    = (float)DISTANCE_THRESHOLD;
float emaB    = (float)DISTANCE_THRESHOLD;

// ─────────────────────────────────────────────
//  State Machine
// ─────────────────────────────────────────────
int   systemState     = 0;
float stallTimeoutSec = 5.0f;
unsigned long stallStartMs = 0;
float stallSec        = 0.0f;

// ─────────────────────────────────────────────
//  Remote Config
// ─────────────────────────────────────────────
String modeOverride = "NONE";

// ─────────────────────────────────────────────
//  Timing (Core 1 — real-time loop)
// ─────────────────────────────────────────────
unsigned long lastBlinkMs = 0;
unsigned long bootMs      = 0;

#define BLINK_2HZ_MS   250
#define BLINK_5HZ_MS   100

bool blinkState = false;

// ─────────────────────────────────────────────
//  Sensors
// ─────────────────────────────────────────────
Adafruit_VL53L0X sensorA = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorB = Adafruit_VL53L0X();
bool sensorA_ok = false;
bool sensorB_ok = false;
float dA = 9999.0f;
float dB = 9999.0f;

// ─────────────────────────────────────────────
//  Shared telemetry struct (Core 1 writes, Core 0 reads)
//  Protected by a FreeRTOS mutex.
// ─────────────────────────────────────────────
struct TelemetrySnapshot {
  int   dA, dB;
  int   threshA, threshB;
  int   state;
  float stallSec;
  int   rssi;
  unsigned long uptimeSec;
  bool  sensorA_ok, sensorB_ok;
  char  modeOverride[24];
};

struct ConfigUpdate {
  bool   hasUpdate;
  char   modeOverride[24];
  float  threshA, threshB;
  float  stallTimeoutSec;
  bool   recalibrate;
};

TelemetrySnapshot latestTelemetry = {};
ConfigUpdate      pendingConfig   = {};
SemaphoreHandle_t          telemMutex      = NULL;
SemaphoreHandle_t          configMutex     = NULL;

// ─────────────────────────────────────────────
//  Prototypes
// ─────────────────────────────────────────────
void  initSensors();
bool  reinitSensor(bool isA);
void  calibrate();
float readSensor(bool isA);
void  updateEMA();
void  updateStateMachine();
void  driveOutputs();
void  allLEDsOff();
void  firebaseTask(void* param);   // Core 0 task

// =============================================================
//  SETUP  (Core 1)
// =============================================================
void setup() {
  Serial.begin(115200);
  bootMs = millis();

  pinMode(XSHUT_A,    OUTPUT);
  pinMode(XSHUT_B,    OUTPUT);
  pinMode(RED_LED,    OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  allLEDsOff();

  // I2C with bus timeout — prevents lockup on long wires
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setTimeOut(50);   // 50 ms I2C timeout; Wire resets bus if exceeded

  initSensors();
  calibrate();

  // WiFi
  Serial.printf("[WiFi] Connecting to %s ...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 30) {
    delay(500); Serial.print("."); tries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] Connected. IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[WiFi] Failed — running offline.");
  }

  // Create mutexes
  telemMutex  = xSemaphoreCreateMutex();
  configMutex = xSemaphoreCreateMutex();

  // ── Pin Firebase task to Core 0, stack 8 KB, priority 1 ──
  xTaskCreatePinnedToCore(
    firebaseTask,     // function
    "FirebaseTask",   // name
    8192,             // stack bytes
    NULL,             // param
    1,                // priority (lower than loop()'s 1 on Core 1)
    NULL,             // handle
    0                 // Core 0
  );

  Serial.println("--------------------------");
  Serial.println("ApexVision Hazard Node");
  Serial.printf ("Thresh A: %.0f mm  Thresh B: %.0f mm\n", threshA, threshB);
  Serial.println("--------------------------");
}

// =============================================================
//  LOOP  (Core 1 — real-time: sensors + state machine + LEDs)
//  Target cycle: ~25 ms  (ToF read ~20 ms + processing ~5 ms)
// =============================================================
void loop() {
  // ── Read sensors ──
  dA = readSensor(true);
  dB = readSensor(false);

  // ── Auto-recover failed sensors via XSHUT toggle ──
  static unsigned long lastRecoveryMs = 0;
  if (!sensorA_ok || !sensorB_ok) {
    unsigned long now = millis();
    if (now - lastRecoveryMs > 5000) {   // attempt recovery every 5 s
      lastRecoveryMs = now;
      if (!sensorA_ok) {
        Serial.println("[I2C] Attempting Sensor A recovery...");
        sensorA_ok = reinitSensor(true);
      }
      if (!sensorB_ok) {
        Serial.println("[I2C] Attempting Sensor B recovery...");
        sensorB_ok = reinitSensor(false);
      }
    }
  }

  // ── EMA + state machine ──
  updateEMA();
  updateStateMachine();

  // ── Apply any pending config from Core 0 ──
  if (xSemaphoreTake(configMutex, 0) == pdTRUE) {
    if (pendingConfig.hasUpdate) {
      modeOverride    = String(pendingConfig.modeOverride);
      threshA         = constrain(pendingConfig.threshA, THRESH_MIN_MM, THRESH_MAX_MM);
      threshB         = constrain(pendingConfig.threshB, THRESH_MIN_MM, THRESH_MAX_MM);
      stallTimeoutSec = pendingConfig.stallTimeoutSec;
      if (pendingConfig.recalibrate) {
        calibrate();
        pendingConfig.recalibrate = false;
      }
      pendingConfig.hasUpdate = false;
    }
    xSemaphoreGive(configMutex);
  }

  // ── Publish telemetry snapshot for Core 0 ──
  if (xSemaphoreTake(telemMutex, 0) == pdTRUE) {
    latestTelemetry.dA          = (int)emaA;
    latestTelemetry.dB          = (int)emaB;
    latestTelemetry.threshA     = (int)threshA;
    latestTelemetry.threshB     = (int)threshB;
    latestTelemetry.state       = systemState;
    latestTelemetry.stallSec    = stallSec;
    latestTelemetry.rssi        = WiFi.RSSI();
    latestTelemetry.uptimeSec   = (millis() - bootMs) / 1000;
    latestTelemetry.sensorA_ok  = sensorA_ok;
    latestTelemetry.sensorB_ok  = sensorB_ok;
    strncpy((char*)latestTelemetry.modeOverride, modeOverride.c_str(), 23);
    xSemaphoreGive(telemMutex);
  }

  // ── Drive LEDs ──
  driveOutputs();

  // Serial debug (brief)
  Serial.printf("A:%.0f B:%.0f thA:%.0f thB:%.0f St:%d Stall:%.1fs\n",
                emaA, emaB, threshA, threshB, systemState, stallSec);

  delay(25);   // ~40 Hz real-time loop
}

// =============================================================
//  FIREBASE TASK  (Core 0 — network I/O, never blocks Core 1)
// =============================================================
void firebaseTask(void* param) {
  // Persistent WiFiClientSecure with connection reuse
  // avoids full TLS handshake on every request
  WiFiClientSecure secureClient;
  secureClient.setInsecure();

  unsigned long lastPutMs = 0;
  unsigned long lastGetMs = 0;

  #define PUT_INTERVAL_MS   650
  #define GET_INTERVAL_MS  1500

  for (;;) {
    unsigned long now = millis();

    if (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    // ── PUT telemetry every 650 ms ──
    if (now - lastPutMs >= PUT_INTERVAL_MS) {
      lastPutMs = now;

      // Snapshot under mutex
      TelemetrySnapshot snap = {};
      if (xSemaphoreTake(telemMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        snap = latestTelemetry;
        xSemaphoreGive(telemMutex);
      }

      StaticJsonDocument<512> doc;
      doc["dA"]          = snap.dA;
      doc["dB"]          = snap.dB;
      doc["threshA"]     = snap.threshA;
      doc["threshB"]     = snap.threshB;
      doc["state"]       = snap.state;
      doc["stallSec"]    = snap.stallSec;
      doc["modeOverride"]= snap.modeOverride;
      doc["rssi"]        = snap.rssi;
      doc["uptimeSec"]   = snap.uptimeSec;
      doc["sensorA_ok"]  = snap.sensorA_ok;
      doc["sensorB_ok"]  = snap.sensorB_ok;

      String body;
      serializeJson(doc, body);

      String url = String(FB_BASE) + "/nodes/" + NODE_ID + "/telemetry.json";
      HTTPClient http;
      http.begin(secureClient, url);
      http.addHeader("Content-Type", "application/json");
      http.setReuse(true);   // keep TCP connection alive between requests
      int code = http.PUT(body);
      if (code < 0)
        Serial.printf("[FB] PUT error: %s\n", http.errorToString(code).c_str());
      http.end();
    }

    // ── GET config every 1500 ms ──
    if (now - lastGetMs >= GET_INTERVAL_MS) {
      lastGetMs = now;

      String url = String(FB_BASE) + "/nodes/" + NODE_ID + "/config.json";
      HTTPClient http;
      http.begin(secureClient, url);
      http.setReuse(true);
      int code = http.GET();

      if (code == 200) {
        String payload = http.getString();
        StaticJsonDocument<512> cfg;
        if (!deserializeJson(cfg, payload)) {

          if (xSemaphoreTake(configMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            pendingConfig.hasUpdate = true;

            if (!cfg["modeOverride"].isNull())
              strncpy(pendingConfig.modeOverride, cfg["modeOverride"].as<const char*>(), 23);
            pendingConfig.threshA = cfg["threshA"].isNull()
              ? threshA : constrain((float)cfg["threshA"], THRESH_MIN_MM, THRESH_MAX_MM);
            pendingConfig.threshB = cfg["threshB"].isNull()
              ? threshB : constrain((float)cfg["threshB"], THRESH_MIN_MM, THRESH_MAX_MM);
            pendingConfig.stallTimeoutSec = cfg["stallTimeoutSec"].isNull()
              ? stallTimeoutSec : (float)cfg["stallTimeoutSec"];
            pendingConfig.recalibrate = (cfg["recalibrate"] == true);

            xSemaphoreGive(configMutex);
          }

          // Clear recalibrate flag on Firebase
          if (cfg["recalibrate"] == true) {
            HTTPClient hp;
            hp.begin(secureClient, String(FB_BASE) + "/nodes/" + NODE_ID + "/config.json");
            hp.addHeader("Content-Type", "application/json");
            hp.addHeader("X-HTTP-Method-Override", "PATCH");
            hp.setReuse(true);
            hp.POST("{\"recalibrate\":false}");
            hp.end();
            Serial.println("[CAL] Remote recalibrate flag cleared.");
          }
        }
      } else if (code < 0) {
        Serial.printf("[FB] GET error: %s\n", http.errorToString(code).c_str());
      }
      http.end();
    }

    // Yield to other tasks — do not busy-wait
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// =============================================================
//  SENSOR INIT  (dual I2C address sequencing)
// =============================================================
void initSensors() {
  digitalWrite(XSHUT_A, LOW);
  digitalWrite(XSHUT_B, LOW);
  delay(10);

  // Sensor A → 0x30
  digitalWrite(XSHUT_A, HIGH);
  delay(10);
  if (!sensorA.begin(0x29, false, &Wire)) {
    Serial.println("[ToF] Sensor A not detected!");
  } else {
    sensorA.setAddress(ADDR_A);
    sensorA_ok = true;
    Serial.println("[ToF] Sensor A OK at 0x30");
  }

  // Sensor B → 0x29
  digitalWrite(XSHUT_B, HIGH);
  delay(10);
  if (!sensorB.begin(ADDR_B, false, &Wire)) {
    Serial.println("[ToF] Sensor B not detected!");
  } else {
    sensorB_ok = true;
    Serial.println("[ToF] Sensor B OK at 0x29");
  }
}

// =============================================================
//  SENSOR AUTO-RECOVERY  (toggle XSHUT and re-init one sensor)
// =============================================================
bool reinitSensor(bool isA) {
  int xshut = isA ? XSHUT_A : XSHUT_B;
  uint8_t addr = isA ? ADDR_A : ADDR_B;

  // Briefly pull XSHUT low to reset the sensor
  digitalWrite(xshut, LOW);
  delay(10);
  digitalWrite(xshut, HIGH);
  delay(10);

  if (isA) {
    // Must re-init A before B to avoid address collision
    if (!sensorA.begin(0x29, false, &Wire)) return false;
    sensorA.setAddress(ADDR_A);
    Serial.println("[ToF] Sensor A recovered at 0x30");
    return true;
  } else {
    if (!sensorB.begin(ADDR_B, false, &Wire)) return false;
    Serial.println("[ToF] Sensor B recovered at 0x29");
    return true;
  }
}

// =============================================================
//  READ SENSOR
// =============================================================
float readSensor(bool isA) {
  VL53L0X_RangingMeasurementData_t measure;
  if (isA && sensorA_ok) {
    sensorA.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) return (float)measure.RangeMilliMeter;
  } else if (!isA && sensorB_ok) {
    sensorB.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) return (float)measure.RangeMilliMeter;
  }
  return 9999.0f;
}

// =============================================================
//  EMA SMOOTHING
// =============================================================
void updateEMA() {
  if (dA < 9000.0f) emaA = EMA_ALPHA * dA + (1.0f - EMA_ALPHA) * emaA;
  if (dB < 9000.0f) emaB = EMA_ALPHA * dB + (1.0f - EMA_ALPHA) * emaB;
}

// =============================================================
//  BOOT CALIBRATION
// =============================================================
void calibrate() {
  Serial.println("[CAL] Calibrating on empty track...");
  float sumA = 0, sumB = 0;
  int   cntA = 0, cntB = 0;
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    float rA = readSensor(true);
    float rB = readSensor(false);
    if (rA < 9000.0f) { sumA += rA; cntA++; }
    if (rB < 9000.0f) { sumB += rB; cntB++; }
    delay(60);
  }
  if (cntA > 0) {
    float base = sumA / cntA;
    threshA = constrain(base * THRESH_RATIO, THRESH_MIN_MM, THRESH_MAX_MM);
    emaA    = base;
  }
  if (cntB > 0) {
    float base = sumB / cntB;
    threshB = constrain(base * THRESH_RATIO, THRESH_MIN_MM, THRESH_MAX_MM);
    emaB    = base;
  }
  Serial.printf("[CAL] threshA=%.0f mm  threshB=%.0f mm\n", threshA, threshB);
}

// =============================================================
//  STATE MACHINE
// =============================================================
void updateStateMachine() {
  if (modeOverride == "FORCE_RED_STOP")       { systemState = 2; return; }
  if (modeOverride == "FORCE_YELLOW_CAUTION") { systemState = 1; return; }
  if (modeOverride == "ECO_SLEEP") {
    systemState = 0;
    // True low-power: disable WiFi, light-sleep 1 s, re-enable
    // (Firebase task on Core 0 will pause since WiFi is off)
    WiFi.disconnect(true);
    esp_sleep_enable_timer_wakeup(1000000ULL);  // 1 s
    esp_light_sleep_start();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    return;
  }

  bool occA = (emaA < threshA);
  bool occB = (emaB < threshB);

  if (!occA && !occB) {
    systemState  = 0;
    stallStartMs = 0;
    stallSec     = 0.0f;
  } else if (occA && occB) {
    if (stallStartMs == 0) stallStartMs = millis();
    stallSec    = (millis() - stallStartMs) / 1000.0f;
    systemState = (stallSec >= stallTimeoutSec) ? 3 : 2;
  } else {
    stallStartMs = 0;
    stallSec     = 0.0f;
    systemState  = 1;
  }
}

// =============================================================
//  OUTPUT DRIVER
// =============================================================
void driveOutputs() {
  unsigned long now = millis();
  switch (systemState) {
    case 0:
      allLEDsOff();
      break;
    case 1: {
      if (now - lastBlinkMs >= BLINK_2HZ_MS) { lastBlinkMs = now; blinkState = !blinkState; }
      digitalWrite(RED_LED,    LOW);
      digitalWrite(YELLOW_LED, blinkState ? HIGH : LOW);
      break;
    }
    case 2: {
      if (now - lastBlinkMs >= BLINK_5HZ_MS) { lastBlinkMs = now; blinkState = !blinkState; }
      digitalWrite(RED_LED,    blinkState ? HIGH : LOW);
      digitalWrite(YELLOW_LED, LOW);
      break;
    }
    case 3: {
      if (now - lastBlinkMs >= BLINK_5HZ_MS) { lastBlinkMs = now; blinkState = !blinkState; }
      digitalWrite(RED_LED,    HIGH);
      digitalWrite(YELLOW_LED, blinkState ? HIGH : LOW);
      break;
    }
  }
}

void allLEDsOff() {
  digitalWrite(RED_LED,    LOW);
  digitalWrite(YELLOW_LED, LOW);
}
