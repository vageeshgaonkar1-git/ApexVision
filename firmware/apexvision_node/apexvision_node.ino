// =============================================================
//  ApexVision — Blind-Curve Hazard Warning Node
//  Target: ESP32 (38-pin or DevKit V1)
//  Requires: Adafruit_VL53L0X, WiFiClientSecure, HTTPClient
//  Firebase RTDB: apexvision-8f1e4 (Test Mode, no auth)
// =============================================================

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ── Uncomment to use PWM pulse-width fallback instead of I2C ToF ──
// #define USE_PWM_MODE

// ─────────────────────────────────────────────
//  WiFi Credentials
// ─────────────────────────────────────────────
const char* WIFI_SSID     = "Alan";
const char* WIFI_PASSWORD = "alanjoy252005";

// ─────────────────────────────────────────────
//  Firebase RTDB (Test Mode — no auth token needed)
// ─────────────────────────────────────────────
const char* FB_HOST   = "apexvision-8f1e4-default-rtdb.asia-southeast1.firebasedatabase.app";
const char* FB_BASE   = "https://apexvision-8f1e4-default-rtdb.asia-southeast1.firebasedatabase.app";
const char* NODE_ID   = "node_01";

// ─────────────────────────────────────────────
//  Hardware Pins
// ─────────────────────────────────────────────
#define SDA_PIN       21
#define SCL_PIN       22
#define XSHUT_A       16    // Sensor A XSHUT
#define XSHUT_B       17    // Sensor B XSHUT
#define ADDR_A        0x30
#define ADDR_B        0x31

#ifdef USE_PWM_MODE
#define PWM_A_PIN     32    // PWM echo pin for Sensor A
#define PWM_B_PIN     33    // PWM echo pin for Sensor B
#define US_PER_MM     10    // 10 µs pulse = 1 mm
#endif

// Directional LED Panels
#define LED_YEL_A     25
#define LED_RED_A     26
#define LED_YEL_B     27
#define LED_RED_B     14
#define BUZZER_PIN    18

// Battery ADC (3.3 V rail → voltage divider → GPIO34)
#define BAT_ADC_PIN   34
// Divider ratio: adjust to match your hardware (e.g. 100k/100k = 0.5)
#define BAT_DIVIDER   2.0f
#define ADC_VREF      3.3f
#define ADC_BITS      4095.0f

// ─────────────────────────────────────────────
//  Calibration & Control Parameters
// ─────────────────────────────────────────────
#define CALIB_SAMPLES     15
#define THRESH_RATIO      0.75f
#define THRESH_MIN_MM     180
#define THRESH_MAX_MM     900
#define EMA_ALPHA         0.65f   // weight on new reading

float threshA = 500.0f;
float threshB = 500.0f;
float emaA    = 500.0f;
float emaB    = 500.0f;

// ─────────────────────────────────────────────
//  State Machine
// ─────────────────────────────────────────────
// 0 = Clear  1 = One-side  2 = Both-side  3 = Stall hazard
int   systemState     = 0;
float stallTimeoutSec = 5.0f;
unsigned long stallStartMs = 0;
float stallSec        = 0.0f;

// ─────────────────────────────────────────────
//  Remote Config / Override
// ─────────────────────────────────────────────
String modeOverride  = "NONE";   // FORCE_RED_STOP | FORCE_YELLOW_CAUTION | ECO_SLEEP | NONE
bool   buzzerEnabled = true;
int    pwmDuty       = 128;      // 0-255, used if buzzer is PWM-driven

// ─────────────────────────────────────────────
//  Timing
// ─────────────────────────────────────────────
unsigned long lastTelemetryMs = 0;
unsigned long lastConfigMs    = 0;
unsigned long lastBlinkMs     = 0;
unsigned long bootMs          = 0;
#define TELEMETRY_INTERVAL_MS  650
#define CONFIG_INTERVAL_MS    1500
#define BLINK_2HZ_MS          250   // 500 ms period → 250 ms half-period
#define BLINK_5HZ_MS          100   // 200 ms period → 100 ms half-period

bool blinkState = false;

// ─────────────────────────────────────────────
//  Sensor Objects
// ─────────────────────────────────────────────
#ifndef USE_PWM_MODE
Adafruit_VL53L0X sensorA;
Adafruit_VL53L0X sensorB;
#endif

bool sensorA_ok = false;
bool sensorB_ok = false;

float dA = 9999.0f;
float dB = 9999.0f;

// ─────────────────────────────────────────────
//  Prototypes
// ─────────────────────────────────────────────
void   initSensors();
void   calibrate();
float  readSensor(bool isA);
void   updateEMA();
void   updateStateMachine();
void   driveOutputs();
void   pushTelemetry();
void   fetchConfig();
void   allLEDsOff();
float  readBattery();

// =============================================================
//  SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  bootMs = millis();

  // ── GPIO init ──
  pinMode(XSHUT_A,   OUTPUT);
  pinMode(XSHUT_B,   OUTPUT);
  pinMode(LED_YEL_A, OUTPUT);
  pinMode(LED_RED_A, OUTPUT);
  pinMode(LED_YEL_B, OUTPUT);
  pinMode(LED_RED_B, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  allLEDsOff();
  digitalWrite(BUZZER_PIN, LOW);

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

  // ── Drive LEDs / Buzzer ──
  driveOutputs();

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

  delay(10);
}

// =============================================================
//  SENSOR INIT — dual I2C address assignment
// =============================================================
void initSensors() {
#ifndef USE_PWM_MODE
  // Both sensors default to 0x29 — bring them up one at a time
  digitalWrite(XSHUT_A, LOW);
  digitalWrite(XSHUT_B, LOW);
  delay(10);

  // ── Sensor A → 0x30 ──
  digitalWrite(XSHUT_A, HIGH);
  delay(10);
  sensorA_ok = sensorA.begin(ADDR_A, false, &Wire);
  if (sensorA_ok) {
    sensorA.setMeasurementTimingBudgetMicroSeconds(33000);
    Serial.println("[ToF] Sensor A OK at 0x30");
  } else {
    Serial.println("[ToF] Sensor A FAIL");
  }

  // ── Sensor B → 0x31 ──
  digitalWrite(XSHUT_B, HIGH);
  delay(10);
  sensorB_ok = sensorB.begin(ADDR_B, false, &Wire);
  if (sensorB_ok) {
    sensorB.setMeasurementTimingBudgetMicroSeconds(33000);
    Serial.println("[ToF] Sensor B OK at 0x31");
  } else {
    Serial.println("[ToF] Sensor B FAIL");
  }

#else
  // PWM fallback — configure pins as inputs to read pulse width
  pinMode(PWM_A_PIN, INPUT);
  pinMode(PWM_B_PIN, INPUT);
  sensorA_ok = true;
  sensorB_ok = true;
  Serial.println("[ToF] PWM mode active");
#endif
}

// =============================================================
//  READ SENSOR (mm)
// =============================================================
float readSensor(bool isA) {
#ifndef USE_PWM_MODE
  VL53L0X_RangingMeasurementData_t measure;
  if (isA && sensorA_ok) {
    sensorA.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) return (float)measure.RangeMilliMeter;
    return 9999.0f;
  } else if (!isA && sensorB_ok) {
    sensorB.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) return (float)measure.RangeMilliMeter;
    return 9999.0f;
  }
  return 9999.0f;
#else
  // PWM: measure pulse width in microseconds; 10 µs → 1 mm
  unsigned long pw = pulseIn(isA ? PWM_A_PIN : PWM_B_PIN, HIGH, 25000UL);
  if (pw == 0) return 9999.0f;
  return (float)(pw / US_PER_MM);
#endif
}

// =============================================================
//  EMA SMOOTHING
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
    emaA = base;
  }
  if (cntB > 0) {
    float base = sumB / cntB;
    threshB = constrain(base * THRESH_RATIO, THRESH_MIN_MM, THRESH_MAX_MM);
    emaB = base;
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
    // Both occupied
    if (stallStartMs == 0) stallStartMs = millis();
    stallSec = (millis() - stallStartMs) / 1000.0f;
    systemState = (stallSec >= stallTimeoutSec) ? 3 : 2;
  } else {
    // One side only — State 1
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
  bool occ_A = (emaA < threshA);
  bool occ_B = (emaB < threshB);

  switch (systemState) {
    // ── State 0 — All clear ──
    case 0:
      allLEDsOff();
      if (buzzerEnabled) digitalWrite(BUZZER_PIN, LOW);
      break;

    // ── State 1 — One side occupied: blink OPPOSITE yellow at 2 Hz ──
    case 1: {
      if (now - lastBlinkMs >= BLINK_2HZ_MS) {
        lastBlinkMs = now;
        blinkState  = !blinkState;
      }
      allLEDsOff();
      if (occ_A) {
        // Vehicle on A side → warn Panel B with yellow
        digitalWrite(LED_YEL_B, blinkState ? HIGH : LOW);
      } else {
        // Vehicle on B side → warn Panel A with yellow
        digitalWrite(LED_YEL_A, blinkState ? HIGH : LOW);
      }
      if (buzzerEnabled) digitalWrite(BUZZER_PIN, LOW);
      break;
    }

    // ── State 2 — Both occupied: strobe red + buzzer at 5 Hz ──
    case 2: {
      if (now - lastBlinkMs >= BLINK_5HZ_MS) {
        lastBlinkMs = now;
        blinkState  = !blinkState;
      }
      bool s = blinkState;
      digitalWrite(LED_RED_A, s ? HIGH : LOW);
      digitalWrite(LED_RED_B, s ? HIGH : LOW);
      digitalWrite(LED_YEL_A, LOW);
      digitalWrite(LED_YEL_B, LOW);
      if (buzzerEnabled) digitalWrite(BUZZER_PIN, s ? HIGH : LOW);
      break;
    }

    // ── State 3 — Stall hazard: alternating Red+Yellow ──
    case 3: {
      if (now - lastBlinkMs >= BLINK_5HZ_MS) {
        lastBlinkMs = now;
        blinkState  = !blinkState;
      }
      // Alternate: even tick → Red A + Yellow B; odd tick → Yellow A + Red B
      digitalWrite(LED_RED_A,  blinkState ? HIGH : LOW);
      digitalWrite(LED_YEL_B,  blinkState ? HIGH : LOW);
      digitalWrite(LED_YEL_A,  blinkState ? LOW  : HIGH);
      digitalWrite(LED_RED_B,  blinkState ? LOW  : HIGH);
      if (buzzerEnabled) digitalWrite(BUZZER_PIN, HIGH);
      break;
    }
  }
}

// =============================================================
//  ALL LEDs OFF helper
// =============================================================
void allLEDsOff() {
  digitalWrite(LED_YEL_A, LOW);
  digitalWrite(LED_RED_A, LOW);
  digitalWrite(LED_YEL_B, LOW);
  digitalWrite(LED_RED_B, LOW);
}

// =============================================================
//  BATTERY VOLTAGE
// =============================================================
float readBattery() {
  int raw = analogRead(BAT_ADC_PIN);
  return ((float)raw / ADC_BITS) * ADC_VREF * BAT_DIVIDER;
}

// =============================================================
//  PUSH TELEMETRY  PUT /nodes/node_01/telemetry.json
// =============================================================
void pushTelemetry() {
  WiFiClientSecure client;
  client.setInsecure();  // Test Mode — skip cert validation

  HTTPClient http;
  String url = String(FB_BASE) + "/nodes/" + NODE_ID + "/telemetry.json";
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<512> doc;
  doc["dA"]          = (int)emaA;
  doc["dB"]          = (int)emaB;
  doc["threshA"]     = (int)threshA;
  doc["threshB"]     = (int)threshB;
  doc["state"]       = systemState;
  doc["stallSec"]    = stallSec;
  doc["modeOverride"]= modeOverride;
  doc["vBat"]        = readBattery();
  doc["rssi"]        = WiFi.RSSI();
  doc["uptimeSec"]   = (millis() - bootMs) / 1000;
  doc["sensorA_ok"]  = sensorA_ok;
  doc["sensorB_ok"]  = sensorB_ok;

  String body;
  serializeJson(doc, body);

  int code = http.PUT(body);
  if (code < 0) Serial.printf("[FB] Telemetry PUT error: %s\n", http.errorToString(code).c_str());
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
      if (!doc["modeOverride"].isNull())    modeOverride    = doc["modeOverride"].as<String>();
      if (!doc["threshA"].isNull())         threshA         = constrain((float)doc["threshA"], THRESH_MIN_MM, THRESH_MAX_MM);
      if (!doc["threshB"].isNull())         threshB         = constrain((float)doc["threshB"], THRESH_MIN_MM, THRESH_MAX_MM);
      if (!doc["stallTimeoutSec"].isNull()) stallTimeoutSec = (float)doc["stallTimeoutSec"];
      if (!doc["buzzerEnabled"].isNull())   buzzerEnabled   = (bool)doc["buzzerEnabled"];
      if (!doc["pwmDuty"].isNull())         pwmDuty         = (int)doc["pwmDuty"];

      // Remote recalibrate trigger
      if (doc["recalibrate"] == true) {
        calibrate();
        // PATCH recalibrate: false
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
