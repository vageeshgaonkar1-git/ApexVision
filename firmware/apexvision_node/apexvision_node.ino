// =============================================================
//  ApexVision — Blind-Curve Hazard Warning Node
//  Target: ESP32 (38-pin or DevKit V1)
//
//  Actual hardware connections (matched to working prototype):
//    I2C SDA       → GPIO 21
//    I2C SCL       → GPIO 22
//    XSHUT Sensor A→ GPIO 18   (Sensor A reassigned to 0x30)
//    XSHUT Sensor B→ GPIO 19   (Sensor B stays at 0x29)
//    RED LED       → GPIO 25   (danger / both-side alert)
//    YELLOW LED    → GPIO 26   (caution / one-side alert)
//
//  Requires: Adafruit_VL53L0X, WiFiClientSecure, HTTPClient, ArduinoJson
//  Firebase RTDB: apexvision-8f1e4 (Test Mode, no auth)
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
//  Hardware Pins  (matched to prototype wiring)
// ─────────────────────────────────────────────
#define SDA_PIN     21
#define SCL_PIN     22
#define XSHUT_A     18      // Sensor A shutdown pin
#define XSHUT_B     19      // Sensor B shutdown pin
#define ADDR_A      0x30    // Sensor A reassigned address
#define ADDR_B      0x29    // Sensor B stays at default

#define RED_LED     25      // Red  LED  — both-side danger
#define YELLOW_LED  26      // Yellow LED — one-side caution

// ─────────────────────────────────────────────
//  Detection & Calibration
// ─────────────────────────────────────────────
#define DISTANCE_THRESHOLD  150   // hard fallback threshold (mm)
#define CALIB_SAMPLES        15   // boot calibration sample count
#define THRESH_RATIO       0.75f  // threshold = 0.75 × baseline
#define THRESH_MIN_MM       100   // clamp low  (mm)
#define THRESH_MAX_MM       900   // clamp high (mm)
#define EMA_ALPHA          0.65f  // EMA weight on newest reading

float threshA = DISTANCE_THRESHOLD;
float threshB = DISTANCE_THRESHOLD;
float emaA    = (float)DISTANCE_THRESHOLD;
float emaB    = (float)DISTANCE_THRESHOLD;

// ─────────────────────────────────────────────
//  State Machine
// ─────────────────────────────────────────────
//  0 = Clear   1 = One-side caution
//  2 = Both-side danger   3 = Stall hazard
int   systemState     = 0;
float stallTimeoutSec = 5.0f;
unsigned long stallStartMs = 0;
float stallSec        = 0.0f;

// ─────────────────────────────────────────────
//  Remote Config / Override
// ─────────────────────────────────────────────
String modeOverride  = "NONE";  // FORCE_RED_STOP | FORCE_YELLOW_CAUTION | ECO_SLEEP | NONE
bool   buzzerEnabled = true;    // kept in config schema even though no buzzer pin wired
int    pwmDuty       = 128;

// ─────────────────────────────────────────────
//  Timing
// ─────────────────────────────────────────────
unsigned long lastTelemetryMs = 0;
unsigned long lastConfigMs    = 0;
unsigned long lastBlinkMs     = 0;
unsigned long bootMs          = 0;

#define TELEMETRY_INTERVAL_MS   650
#define CONFIG_INTERVAL_MS     1500
#define BLINK_2HZ_MS            250   // 500 ms period → 250 ms half-period
#define BLINK_5HZ_MS            100   // 200 ms period → 100 ms half-period

bool blinkState = false;

// ─────────────────────────────────────────────
//  Sensor Objects
// ─────────────────────────────────────────────
Adafruit_VL53L0X sensorA = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorB = Adafruit_VL53L0X();

bool sensorA_ok = false;
bool sensorB_ok = false;

float dA = 9999.0f;
float dB = 9999.0f;

// ─────────────────────────────────────────────
//  Prototypes
// ─────────────────────────────────────────────
void  initSensors();
void  calibrate();
float readSensor(bool isA);
void  updateEMA();
void  updateStateMachine();
void  driveOutputs();
void  pushTelemetry();
void  fetchConfig();
void  allLEDsOff();

// =============================================================
//  SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  bootMs = millis();

  // ── GPIO init ──
  pinMode(XSHUT_A,   OUTPUT);
  pinMode(XSHUT_B,   OUTPUT);
  pinMode(RED_LED,   OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  allLEDsOff();

  // ── I2C ──
  Wire.begin(SDA_PIN, SCL_PIN);

  // ── Sensors ──
  initSensors();
  calibrate();

  // ── WiFi ──
  Serial.printf("[WiFi] Connecting to %s ...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 30) {
    delay(500);
    Serial.print(".");
    tries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] Connected. IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[WiFi] Failed — running offline.");
  }

  Serial.println("--------------------------");
  Serial.println("ApexVision Hazard Node");
  Serial.printf ("Thresh A: %.0f mm  Thresh B: %.0f mm\n", threshA, threshB);
  Serial.println("--------------------------");
}

// =============================================================
//  LOOP
// =============================================================
void loop() {
  unsigned long now = millis();

  // ── Read & smooth sensors ──
  dA = readSensor(true);
  dB = readSensor(false);
  updateEMA();

  // ── State machine ──
  updateStateMachine();

  // ── Drive LEDs ──
  driveOutputs();

  // ── Serial debug ──
  Serial.printf("A: %.0f mm  B: %.0f mm  | thA: %.0f  thB: %.0f  | State: %d  Stall: %.1f s\n",
                emaA, emaB, threshA, threshB, systemState, stallSec);

  // ── Firebase telemetry push ──
  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;
    if (WiFi.status() == WL_CONNECTED) pushTelemetry();
  }

  // ── Firebase config pull ──
  if (now - lastConfigMs >= CONFIG_INTERVAL_MS) {
    lastConfigMs = now;
    if (WiFi.status() == WL_CONNECTED) fetchConfig();
  }

  delay(50);
}

// =============================================================
//  SENSOR INIT
//  Both VL53L0X boot at 0x29.
//  Strategy: shut both down, bring A up → reassign to 0x30,
//            then bring B up → it stays at 0x29.
// =============================================================
void initSensors() {
  digitalWrite(XSHUT_A, LOW);
  digitalWrite(XSHUT_B, LOW);
  delay(10);

  // ── Sensor A: boot at 0x29, reassign to 0x30 ──
  digitalWrite(XSHUT_A, HIGH);
  delay(10);
  if (!sensorA.begin(0x29, false, &Wire)) {
    Serial.println("[ToF] Sensor A not detected!");
  } else {
    sensorA.setAddress(ADDR_A);   // move to 0x30
    sensorA_ok = true;
    Serial.println("[ToF] Sensor A OK at 0x30");
  }

  // ── Sensor B: boot at 0x29, stays at 0x29 ──
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
//  READ SENSOR — returns distance in mm, 9999 on error/timeout
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
//  EMA SMOOTHING  (only update when reading is valid)
// =============================================================
void updateEMA() {
  if (dA < 9000.0f) emaA = EMA_ALPHA * dA + (1.0f - EMA_ALPHA) * emaA;
  if (dB < 9000.0f) emaB = EMA_ALPHA * dB + (1.0f - EMA_ALPHA) * emaB;
}

// =============================================================
//  BOOT CALIBRATION — 15-sample tare on empty track
// =============================================================
void calibrate() {
  Serial.println("[CAL] Starting baseline calibration...");
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
  // Remote override takes priority
  if (modeOverride == "FORCE_RED_STOP")       { systemState = 2; return; }
  if (modeOverride == "FORCE_YELLOW_CAUTION") { systemState = 1; return; }
  if (modeOverride == "ECO_SLEEP")            { systemState = 0; return; }

  bool occA = (emaA < threshA);
  bool occB = (emaB < threshB);

  if (!occA && !occB) {
    // State 0 — clear
    systemState  = 0;
    stallStartMs = 0;
    stallSec     = 0.0f;

  } else if (occA && occB) {
    // Both occupied — start stall timer
    if (stallStartMs == 0) stallStartMs = millis();
    stallSec    = (millis() - stallStartMs) / 1000.0f;
    systemState = (stallSec >= stallTimeoutSec) ? 3 : 2;

  } else {
    // One side only
    stallStartMs = 0;
    stallSec     = 0.0f;
    systemState  = 1;
  }
}

// =============================================================
//  OUTPUT DRIVER
//  Hardware: RED_LED (GPIO 25), YELLOW_LED (GPIO 26)
//
//  State 0 — both off
//  State 1 — yellow blinks 2 Hz (caution)
//  State 2 — red blinks 5 Hz (danger)
//  State 3 — red solid + yellow alternate 5 Hz (stall hazard)
// =============================================================
void driveOutputs() {
  unsigned long now = millis();

  switch (systemState) {

    // ── State 0: All clear ──
    case 0:
      allLEDsOff();
      break;

    // ── State 1: One side — Yellow blinks 2 Hz ──
    case 1: {
      if (now - lastBlinkMs >= BLINK_2HZ_MS) {
        lastBlinkMs = now;
        blinkState  = !blinkState;
      }
      digitalWrite(RED_LED,    LOW);
      digitalWrite(YELLOW_LED, blinkState ? HIGH : LOW);
      break;
    }

    // ── State 2: Both sides — Red blinks 5 Hz ──
    case 2: {
      if (now - lastBlinkMs >= BLINK_5HZ_MS) {
        lastBlinkMs = now;
        blinkState  = !blinkState;
      }
      digitalWrite(RED_LED,    blinkState ? HIGH : LOW);
      digitalWrite(YELLOW_LED, LOW);
      break;
    }

    // ── State 3: Stall hazard — Red solid, Yellow fast-alternates ──
    case 3: {
      if (now - lastBlinkMs >= BLINK_5HZ_MS) {
        lastBlinkMs = now;
        blinkState  = !blinkState;
      }
      digitalWrite(RED_LED,    HIGH);
      digitalWrite(YELLOW_LED, blinkState ? HIGH : LOW);
      break;
    }
  }
}

// =============================================================
//  ALL LEDs OFF helper
// =============================================================
void allLEDsOff() {
  digitalWrite(RED_LED,    LOW);
  digitalWrite(YELLOW_LED, LOW);
}

// =============================================================
//  PUSH TELEMETRY  PUT /nodes/node_01/telemetry.json
// =============================================================
void pushTelemetry() {
  WiFiClientSecure client;
  client.setInsecure();   // Test Mode — skip cert validation

  HTTPClient http;
  String url = String(FB_BASE) + "/nodes/" + NODE_ID + "/telemetry.json";
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<512> doc;
  doc["dA"]           = (int)emaA;
  doc["dB"]           = (int)emaB;
  doc["threshA"]      = (int)threshA;
  doc["threshB"]      = (int)threshB;
  doc["state"]        = systemState;
  doc["stallSec"]     = stallSec;
  doc["modeOverride"] = modeOverride;
  doc["vBat"]         = 0.0;        // no battery ADC wired on this prototype
  doc["rssi"]         = WiFi.RSSI();
  doc["uptimeSec"]    = (millis() - bootMs) / 1000;
  doc["sensorA_ok"]   = sensorA_ok;
  doc["sensorB_ok"]   = sensorB_ok;

  String body;
  serializeJson(doc, body);

  int code = http.PUT(body);
  if (code < 0) {
    Serial.printf("[FB] Telemetry PUT error: %s\n", http.errorToString(code).c_str());
  }
  http.end();
}

// =============================================================
//  FETCH CONFIG  GET /nodes/node_01/config.json
// =============================================================
void fetchConfig() {
  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  String url = String(FB_BASE) + "/nodes/" + NODE_ID + "/config.json";
  http.begin(client, url);

  int code = http.GET();
  if (code == 200) {
    String payload = http.getString();
    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, payload);
    if (!err) {
      if (!doc["modeOverride"].isNull())
        modeOverride = doc["modeOverride"].as<String>();
      if (!doc["threshA"].isNull())
        threshA = constrain((float)doc["threshA"], THRESH_MIN_MM, THRESH_MAX_MM);
      if (!doc["threshB"].isNull())
        threshB = constrain((float)doc["threshB"], THRESH_MIN_MM, THRESH_MAX_MM);
      if (!doc["stallTimeoutSec"].isNull())
        stallTimeoutSec = (float)doc["stallTimeoutSec"];
      if (!doc["buzzerEnabled"].isNull())
        buzzerEnabled = (bool)doc["buzzerEnabled"];
      if (!doc["pwmDuty"].isNull())
        pwmDuty = (int)doc["pwmDuty"];

      // Remote recalibrate trigger
      if (doc["recalibrate"] == true) {
        calibrate();
        // Clear the flag on Firebase
        WiFiClientSecure c2;
        c2.setInsecure();
        HTTPClient hp;
        String patchUrl = String(FB_BASE) + "/nodes/" + NODE_ID + "/config.json";
        hp.begin(c2, patchUrl);
        hp.addHeader("Content-Type", "application/json");
        hp.addHeader("X-HTTP-Method-Override", "PATCH");
        hp.POST("{\"recalibrate\":false}");
        hp.end();
        Serial.println("[CAL] Remote recalibrate done, flag cleared.");
      }
    }
  } else if (code < 0) {
    Serial.printf("[FB] Config GET error: %s\n", http.errorToString(code).c_str());
  }
  http.end();
}
