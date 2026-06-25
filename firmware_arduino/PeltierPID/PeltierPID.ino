/*
  Peltier PID Controller — ItsyBitsy M0 Express
  ===============================================
  Sterownik Cytron MDD10A REV2.0 (tryb DIR+PWM Sign-Magnitude)

  POLACZENIA STEROWNIK -> ItsyBitsy:
    DIR1  -> pin 3   (kierunek Peltiera)
    PWM1  -> pin 4   (moc Peltiera, PWM)
    DIR2  -> pin 5   (kierunek wentylatora)
    PWM2  -> pin 11  (moc wentylatora, PWM)
    GND   -> G

  POLACZENIA MAX31856 -> ItsyBitsy:
    SCK  -> SCK
    SDO  -> MISO
    SDI  -> MOSI
    CS1  -> pin 9   (czujnik 1 - regulacja PID)
    CS2  -> pin 10  (czujnik 2 - pomiar dodatkowy)
    GND  -> G
    VCC  -> 3V

  BIBLIOTEKI:
    Adafruit MAX31856 library
    Adafruit BusIO
    ArduinoJson (Benoit Blanchon, wersja 6.x)

  PLYTKA:
    Tools > Board > Adafruit SAMD > Adafruit ItsyBitsy M0 Express
*/

#include <SPI.h>
#include <Adafruit_MAX31856.h>
#include <ArduinoJson.h>

// ─── PINY ─────────────────────────────────────────────────────────────────────
#define PIN_DIR1   3
#define PIN_PWM1   4
#define PIN_DIR2   5
#define PIN_PWM2   11
#define PIN_CS1    9
#define PIN_CS2    10

// ─── STALE ────────────────────────────────────────────────────────────────────
#define PWM_MAX        255
#define PID_DT_MS      100      // okres petli PID [ms]
#define INTEGRAL_MAX   400.0f   // anti-windup
#define TEMP_MIN_C     -15.0f   // zabezpieczenie dolne
#define TEMP_MAX_DEF   80.0f    // zabezpieczenie gorne
#define REPORT_MS      500      // co ile ms wysylac JSON
#define TF_SIZE        4        // rozmiar bufora filtru temperatury
#define SPIKE_THR      8.0f     // prog odrzucenia skoku [C]

// ─── CZUJNIKI ─────────────────────────────────────────────────────────────────
Adafruit_MAX31856 sensor1 = Adafruit_MAX31856(PIN_CS1);
Adafruit_MAX31856 sensor2 = Adafruit_MAX31856(PIN_CS2);

// ─── STAN PID ─────────────────────────────────────────────────────────────────
struct {
  float setpoint   = 25.0f;
  float kp         = 10.0f;
  float ki         = 0.3f;
  float kd         = 0.8f;
  bool  enabled    = false;
  bool  heatMode   = true;    // true=grzanie, false=chlodzenie
  float integral   = 0.0f;
  float prevError  = 0.0f;
  float dFilt      = 0.0f;    // filtrowana pochodna (tlumi szum)
  float pwmFilt    = 0.0f;    // filtrowane wyjscie PWM (gladka moc)
  unsigned long prevTime = 0;
} pid;

// ─── WENTYLATOR ───────────────────────────────────────────────────────────────
struct {
  bool  autoMode = true;
  float manual   = 0.0f;  // 0-100%
  float speed    = 0.0f;  // aktualna predkosc
} fan;

// ─── FILTR TEMPERATURY (jak w referencji: bufor + spike rejection) ─────────────
float tfBuf[TF_SIZE] = {25, 25, 25, 25};
int   tfIdx = 0;
float lastTemp = 25.0f;
bool  sensorError = false;
float calOffset = 0.0f;   // offset kalibracji termopary

// Temperatura czujnika 2 (pomiar dodatkowy)
float temp2 = NAN;

// ─── SERIAL ───────────────────────────────────────────────────────────────────
String cmdBuf = "";
unsigned long lastReport = 0;

// ─── FUNKCJE TEMPERATURY ──────────────────────────────────────────────────────

// Odczyt z filtrem (jak w kodzie referencyjnym):
// - odrzuca skoki > SPIKE_THR (szum/zaklucenia)
// - usrednia ostatnie TF_SIZE probki
// - dodaje offset kalibracji
float readTemp() {
  uint8_t fault = sensor1.readFault();
  if (fault) {
    sensorError = true;
    return lastTemp;
  }
  sensorError = false;
  float raw = sensor1.readThermocoupleTemperature();

  // Odrzuc NaN i wartosci poza fizycznym zakresem
  if (isnan(raw) || raw < -50.0f || raw > 200.0f) {
    sensorError = true;
    return lastTemp;
  }

  // Spike rejection: jesli skok > SPIKE_THR, uzyj poprzedniej probki
  float prev = tfBuf[(tfIdx - 1 + TF_SIZE) % TF_SIZE];
  if (abs(raw - prev) > SPIKE_THR) raw = prev;

  // Zapisz do bufora i usrednij
  tfBuf[tfIdx] = raw;
  tfIdx = (tfIdx + 1) % TF_SIZE;
  float sum = 0;
  for (int i = 0; i < TF_SIZE; i++) sum += tfBuf[i];
  lastTemp = (sum / TF_SIZE) + calOffset;
  return lastTemp;
}

// Odczyt czujnika 2 (pomiar dodatkowy, nie wplywa na PID)
void readTemp2() {
  float r = sensor2.readThermocoupleTemperature();
  if (!isnan(r) && r > -50.0f && r < 200.0f) temp2 = r;
  else temp2 = NAN;
}

// ─── FUNKCJE STEROWANIA ───────────────────────────────────────────────────────

void setPeltier(float pct, bool heat) {
  pct = constrain(pct, 0.0f, 100.0f);
  digitalWrite(PIN_DIR1, heat ? HIGH : LOW);
  analogWrite(PIN_PWM1, (int)(pct / 100.0f * PWM_MAX));
}

void stopPeltier() {
  analogWrite(PIN_PWM1, 0);
  analogWrite(PIN_DIR1, LOW);
  pid.integral = 0;
  pid.prevError = 0;
  pid.dFilt = 0;
  pid.pwmFilt = 0;
}

void setFan(float pct) {
  pct = constrain(pct, 0.0f, 100.0f);
  fan.speed = pct;
  digitalWrite(PIN_DIR2, HIGH);
  analogWrite(PIN_PWM2, (int)(pct / 100.0f * PWM_MAX));
}

// ─── PID (wzorowany na kodzie referencyjnym) ──────────────────────────────────
// Kluczowe cechy:
//   - filtr pochodnej (EMA alfa=0.3) -> brak drgania PWM od szumu termopary
//   - filtr wyjscia PWM (EMA alfa=0.4) -> gladka zmiana mocy = prosta linia T
//   - anti-windup (constrain integratora)
//   - jednokierunkowosc: grzanie=tylko+PWM, chlodzenie=tylko-PWM
float computePID(float temp, unsigned long now) {
  float dt = (now - pid.prevTime) / 1000.0f;
  if (dt <= 0.001f) return pid.pwmFilt;
  pid.prevTime = now;

  float error = pid.setpoint - temp;

  // Anti-windup: przy duzym bledzie ograniczony zakres integratora
  float igLim = (fabs(error) < 2.0f) ? INTEGRAL_MAX : INTEGRAL_MAX * 0.5f;
  pid.integral = constrain(pid.integral + error * dt, -igLim, igLim);

  // Filtr pochodnej (EMA alfa=0.3) - tlumi szum termopary
  float dRaw = (error - pid.prevError) / dt;
  pid.dFilt = pid.dFilt + 0.3f * (dRaw - pid.dFilt);
  pid.prevError = error;

  float out = pid.kp * error + pid.ki * pid.integral + pid.kd * pid.dFilt;

  // Jednokierunkowosc (grzanie XOR chlodzenie)
  if (pid.heatMode) out = constrain(out, 0.0f, (float)PWM_MAX);
  else              out = constrain(out, -(float)PWM_MAX, 0.0f);

  // Filtr wyjscia PWM (EMA alfa=0.4) - gladka zmiana mocy
  pid.pwmFilt = pid.pwmFilt + 0.4f * (out - pid.pwmFilt);

  return pid.pwmFilt;
}

// ─── WYSYLANIE CFG DO PC ──────────────────────────────────────────────────────
void sendCfg() {
  StaticJsonDocument<256> doc;
  doc["type"]    = "cfg";
  doc["sp"]      = pid.setpoint;
  doc["kp"]      = pid.kp;
  doc["ki"]      = pid.ki;
  doc["kd"]      = pid.kd;
  doc["pid_on"]  = pid.enabled;
  doc["heat"]    = pid.heatMode;
  doc["fan_auto"]= fan.autoMode;
  doc["fan_man"] = fan.manual;
  doc["offset"]  = calOffset;
  serializeJson(doc, Serial);
  Serial.println();
}

// ─── PARSER KOMEND Z PC ───────────────────────────────────────────────────────
// Format: KLUCZ:wartosc\n   np. "SP:30.0"
void processCommand(String c) {
  c.trim();
  if (c.length() == 0) return;

  int colon = c.indexOf(':');
  String key = (colon >= 0) ? c.substring(0, colon) : c;
  String val = (colon >= 0) ? c.substring(colon + 1) : "";
  key.toUpperCase();
  float fv = val.toFloat();

  if      (key == "SP")     { pid.setpoint = constrain(fv, -15.0f, 100.0f); }
  else if (key == "KP")     { pid.kp = constrain(fv, 1.0f, 30.0f); }
  else if (key == "KI")     { pid.ki = constrain(fv, 0.0f, 1.5f); }
  else if (key == "KD")     { pid.kd = constrain(fv, 0.0f, 3.0f); }
  else if (key == "HEAT")   { pid.heatMode = (fv > 0.5f); }
  else if (key == "FANAUTO"){ fan.autoMode = (fv > 0.5f); }
  else if (key == "FAN")    { fan.manual = constrain(fv, 0.0f, 100.0f); if (!fan.autoMode) setFan(fan.manual); }
  else if (key == "OFFSET") { calOffset = constrain(fv, -20.0f, 20.0f); }
  else if (key == "START")  {
    pid.enabled = true;
    pid.integral = 0; pid.prevError = 0;
    pid.dFilt = 0;    pid.pwmFilt = 0;
    pid.prevTime = millis();
    Serial.println("{\"type\":\"status\",\"msg\":\"ON\"}");
  }
  else if (key == "STOP") {
    pid.enabled = false;
    stopPeltier();
    setFan(100);  // chlodz radiator po zatrzymaniu
    Serial.println("{\"type\":\"status\",\"msg\":\"STOP\"}");
  }
  else if (key == "FANOFF") { setFan(0); }
  else if (key == "GET")    { sendCfg(); return; }
  else if (key == "RESET")  {
    pid.kp=10; pid.ki=0.3f; pid.kd=0.8f;
    pid.integral=0; pid.prevError=0;
    Serial.println("{\"type\":\"status\",\"msg\":\"RESET\"}");
  }

  sendCfg();  // potwierdz zmiane
}

void readSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (cmdBuf.length() > 0) { processCommand(cmdBuf); cmdBuf = ""; }
    } else {
      cmdBuf += ch;
      if (cmdBuf.length() > 80) cmdBuf = "";  // ochrona przed przepelnieniem
    }
  }
}

// ─── SETUP ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(PIN_DIR1, OUTPUT); analogWrite(PIN_PWM1, 0); digitalWrite(PIN_DIR1, LOW);
  pinMode(PIN_DIR2, OUTPUT); analogWrite(PIN_PWM2, 0); digitalWrite(PIN_DIR2, LOW);
  pinMode(PIN_PWM1, OUTPUT);
  pinMode(PIN_PWM2, OUTPUT);

  // Czujnik 1 (PID)
  if (!sensor1.begin()) {
    Serial.println("{\"type\":\"error\",\"msg\":\"MAX31856 CS9 brak\"}");
  } else {
    sensor1.setThermocoupleType(MAX31856_TCTYPE_K);
  }

  // Czujnik 2 (pomiar dodatkowy)
  sensor2.begin();
  sensor2.setThermocoupleType(MAX31856_TCTYPE_K);

  // Inicjalizuj bufor filtra aktualną temperaturą
  delay(200);
  float initT = sensor1.readThermocoupleTemperature();
  if (!isnan(initT) && initT > -50 && initT < 150) {
    for (int i = 0; i < TF_SIZE; i++) tfBuf[i] = initT;
    lastTemp = initT;
  }
  pid.setpoint = lastTemp;
  pid.prevTime = millis();

  // Powiadom PC ze jestesmy gotowi
  sendCfg();
  Serial.println("{\"type\":\"status\",\"msg\":\"READY\"}");
}

// ─── LOOP ─────────────────────────────────────────────────────────────────────
unsigned long lastPid = 0;

void loop() {
  unsigned long now = millis();

  readSerial();

  // ── Petla PID (co PID_DT_MS) ──
  if (now - lastPid >= PID_DT_MS) {
    lastPid = now;

    float temp = readTemp();
    readTemp2();

    // Zabezpieczenie termiczne
    if (temp > TEMP_MAX_DEF && pid.enabled) {
      pid.enabled = false;
      stopPeltier();
      Serial.println("{\"type\":\"error\",\"msg\":\"TEMP MAX\"}");
    }

    // Sterowanie
    float peltierPct = 0.0f;
    if (pid.enabled) {
      float raw = computePID(temp, now);
      peltierPct = fabs(raw) / PWM_MAX * 100.0f;
      setPeltier(peltierPct, pid.heatMode);
    } else {
      stopPeltier();
    }

    // Wentylator
    float fanPct = 0.0f;
    if (fan.autoMode) {
      float delta = fabs(temp - pid.setpoint);
      fanPct = constrain(delta * 5.0f, 0.0f, 100.0f);
    } else {
      fanPct = fan.manual;
    }
    setFan(fanPct);

    // ── Raport JSON do PC (co REPORT_MS) ──
    if (now - lastReport >= REPORT_MS) {
      lastReport = now;

      StaticJsonDocument<256> doc;
      doc["type"] = "data";
      doc["ts"]   = now;

      if (sensorError) doc["t1"] = nullptr;
      else             doc["t1"] = round(temp * 10) / 10.0;

      if (isnan(temp2)) doc["t2"] = nullptr;
      else              doc["t2"] = round(temp2 * 10) / 10.0;

      doc["sp"]      = pid.setpoint;
      doc["pct"]     = round(peltierPct * 10) / 10.0;
      doc["fan"]     = round(fanPct * 10) / 10.0;
      doc["pid_on"]  = pid.enabled;
      doc["heat"]    = pid.heatMode;
      doc["kp"]      = pid.kp;
      doc["ki"]      = pid.ki;
      doc["kd"]      = pid.kd;
      doc["err_sen"] = sensorError;

      serializeJson(doc, Serial);
      Serial.println();
    }
  }
}
