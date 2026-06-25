/*
  Peltier PID Controller — ItsyBitsy M0 Express
  ===============================================
  Sterownik Cytron MDD10A REV2.0 (tryb DIR+PWM)

  POLACZENIA:
  DIR1  pin 3  | PWM1 pin 4   (Peltier)
  DIR2  pin 5  | PWM2 pin 11  (Wentylator)
  GND   G

  MAX31856:
  SCK->SCK | MISO->SDO | MOSI->SDI
  CS1->pin9 | CS2->pin10 | GND->G | VCC->3V

  BIBLIOTEKI (Library Manager):
  - Adafruit MAX31856 library
  - Adafruit BusIO
  - ArduinoJson by Benoit Blanchon wersja 6.x

  PLYTKA w Arduino IDE:
  Tools > Board > Adafruit SAMD > Adafruit ItsyBitsy M0 Express
  Board Manager URL:
  https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
*/

#include <SPI.h>
#include <Adafruit_MAX31856.h>
#include <ArduinoJson.h>

#define PIN_DIR1   3
#define PIN_PWM1   4
#define PIN_DIR2   5
#define PIN_PWM2   11
#define PIN_CS1    9
#define PIN_CS2    10

Adafruit_MAX31856 sensor1 = Adafruit_MAX31856(PIN_CS1);
Adafruit_MAX31856 sensor2 = Adafruit_MAX31856(PIN_CS2);

struct PIDState {
  float setpoint  = 25.0f;
  float kp        = 5.0f;
  float ki        = 0.1f;
  float kd        = 1.0f;
  bool  enabled   = false;
  bool  heatMode  = true;
  float integral  = 0.0f;
  float prevError = 0.0f;
  unsigned long prevTime = 0;
} pid;

struct FanState {
  bool  autoMode = true;
  float manual   = 0.0f;
} fan;

void setPeltier(float pct, bool heat) {
  pct = constrain(pct, 0.0f, 100.0f);
  digitalWrite(PIN_DIR1, heat ? HIGH : LOW);
  analogWrite(PIN_PWM1, (int)(pct / 100.0f * 255));
}

void setFan(float pct) {
  pct = constrain(pct, 0.0f, 100.0f);
  digitalWrite(PIN_DIR2, HIGH);
  analogWrite(PIN_PWM2, (int)(pct / 100.0f * 255));
}

float computePID(float temp, unsigned long now) {
  float dt = (now - pid.prevTime) / 1000.0f;
  if (dt <= 0.0f) return 0.0f;
  float error = pid.setpoint - temp;
  pid.integral += error * dt;
  pid.integral = constrain(pid.integral, -100.0f, 100.0f);
  float derivative = (error - pid.prevError) / dt;
  float output = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
  pid.prevError = error;
  pid.prevTime  = now;
  return constrain(output, 0.0f, 100.0f);
}

void processCommand(const String& raw) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, raw) != DeserializationError::Ok) return;
  if (doc.containsKey("setpoint"))   pid.setpoint = doc["setpoint"].as<float>();
  if (doc.containsKey("kp"))         pid.kp       = doc["kp"].as<float>();
  if (doc.containsKey("ki"))         pid.ki       = doc["ki"].as<float>();
  if (doc.containsKey("kd"))         pid.kd       = doc["kd"].as<float>();
  if (doc.containsKey("heat_mode"))  pid.heatMode = doc["heat_mode"].as<bool>();
  if (doc.containsKey("fan_auto"))   fan.autoMode = doc["fan_auto"].as<bool>();
  if (doc.containsKey("fan_manual")) fan.manual   = doc["fan_manual"].as<float>();
  if (doc.containsKey("pid_enabled")) {
    pid.enabled = doc["pid_enabled"].as<bool>();
    if (!pid.enabled) {
      setPeltier(0, pid.heatMode);
      pid.integral = 0;
      pid.prevError = 0;
    }
  }
  if (doc.containsKey("reset_pid") && doc["reset_pid"].as<bool>()) {
    pid.integral = 0;
    pid.prevError = 0;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_DIR1, OUTPUT);
  pinMode(PIN_PWM1, OUTPUT);
  pinMode(PIN_DIR2, OUTPUT);
  pinMode(PIN_PWM2, OUTPUT);
  setPeltier(0, true);
  setFan(0);
  if (!sensor1.begin()) Serial.println("{\"error\":\"MAX31856 CS9 brak\"}");
  else sensor1.setThermocoupleType(MAX31856_TCTYPE_K);
  if (!sensor2.begin()) Serial.println("{\"error\":\"MAX31856 CS10 brak\"}");
  else sensor2.setThermocoupleType(MAX31856_TCTYPE_K);
  pid.prevTime = millis();
}

unsigned long lastReport = 0;
String serialBuf = "";

void loop() {
  unsigned long now = millis();

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      serialBuf.trim();
      if (serialBuf.length() > 0) processCommand(serialBuf);
      serialBuf = "";
    } else {
      serialBuf += c;
    }
  }

  if (now - lastReport >= 500) {
    lastReport = now;

    float t1 = NAN, t2 = NAN;
    uint8_t f1 = sensor1.readFault();
    uint8_t f2 = sensor2.readFault();
    if (!f1) t1 = sensor1.readThermocoupleTemperature();
    if (!f2) t2 = sensor2.readThermocoupleTemperature();

    float tempCtrl = NAN;
    if (!isnan(t1) && !isnan(t2))  tempCtrl = (t1 + t2) / 2.0f;
    else if (!isnan(t1))           tempCtrl = t1;
    else if (!isnan(t2))           tempCtrl = t2;

    float peltierPct = 0.0f;
    if (pid.enabled && !isnan(tempCtrl)) {
      if (pid.prevTime == 0) pid.prevTime = now;
      peltierPct = computePID(tempCtrl, now);
      setPeltier(peltierPct, pid.heatMode);
    } else if (!pid.enabled) {
      setPeltier(0, pid.heatMode);
    }

    float fanPct = 0.0f;
    if (fan.autoMode && !isnan(tempCtrl)) {
      fanPct = constrain(abs(tempCtrl - pid.setpoint) * 5.0f, 0.0f, 100.0f);
    } else {
      fanPct = fan.manual;
    }
    setFan(fanPct);

    StaticJsonDocument<256> doc;
    if (isnan(t1)) doc["t1"] = nullptr; else doc["t1"] = round(t1 * 10) / 10.0;
    if (isnan(t2)) doc["t2"] = nullptr; else doc["t2"] = round(t2 * 10) / 10.0;
    doc["setpoint"]    = pid.setpoint;
    doc["peltier_pct"] = round(peltierPct * 10) / 10.0;
    doc["fan_pct"]     = round(fanPct * 10) / 10.0;
    doc["pid_on"]      = pid.enabled;
    doc["heat_mode"]   = pid.heatMode;
    doc["ts"]          = now;
    serializeJson(doc, Serial);
    Serial.println();
  }
}
