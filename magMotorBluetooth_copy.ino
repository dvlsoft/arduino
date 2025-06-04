#include <Wire.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

// --- RM3100 ---
#define RM3100_ADDR 0x20
#define REG_POLL 0x00
#define REG_CMM 0x01
#define REG_STATUS 0x34
#define REG_CCX1 0x04
#define REG_CCX0 0x05
#define DRDY_PIN 7
#define CYCLE_COUNT 200
#define GAIN (0.3671 * CYCLE_COUNT + 1.5)

// --- Stepper Motor ---
#define STEP_PIN 4
#define DIR_PIN 3
#define ENABLE_PIN 2
#define STEPS_PER_REV 200
#define MICROSTEPS 16
#define STEP_DELAY_US 800

// --- Bluetooth ---
SoftwareSerial BT(9, 10);  // RX, TX

// --- EEPROM ---
#define EEPROM_ADDR 0

// --- Control ---
#define BUTTON_PIN 8
#define MIN_DELTA 5.0
bool trackingEnabled = false;

float prevAzimuth = 0.0;
float targetAzimuth = 0.0;

unsigned long lastButtonTime = 0;

void setup() {
  pinMode(DRDY_PIN, INPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.begin(9600);
  BT.begin(9600);
  Wire.begin();

  log("Initializing...");
  initRM3100();
  prevAzimuth = readAzimuthFromEEPROM();
  log("Ready. Stored Azimuth: " + String(prevAzimuth, 1));
}

void loop() {
  if (digitalRead(BUTTON_PIN) == LOW && millis() - lastButtonTime > 500) {
    trackingEnabled = !trackingEnabled;
    lastButtonTime = millis();
    log(trackingEnabled ? "Tracking ENABLED (by button)" : "Tracking DISABLED");
  }

  float az = getAzimuth();
  String dir = getCompassDirection(az);
  log("Azimuth: " + String(az, 1) + "° - " + dir);

  if (trackingEnabled) {
    float delta = az - prevAzimuth;
    if (delta > 180) delta -= 360;
    if (delta < -180) delta += 360;

    if (abs(delta) >= MIN_DELTA) {
      rotateByAngle(delta);
      prevAzimuth = normalizeAngle(az);
      writeAzimuthToEEPROM(prevAzimuth);
    }
  }

  if (BT.available()) {
    String cmd = BT.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    handleCommand(cmd, az, dir);
  }

  delay(300);
}

// --- RM3100 ---
void initRM3100() {
  writeReg(REG_CMM, 0x00);
  delay(10);
  writeReg(REG_POLL, 0x70);
  delay(10);
  setCycleCount(CYCLE_COUNT);
  writeReg(REG_CMM, 0x79);
}

void setCycleCount(uint16_t cc) {
  Wire.beginTransmission(RM3100_ADDR);
  Wire.write(REG_CCX1);
  for (int i = 0; i < 3; i++) {
    Wire.write(cc >> 8);
    Wire.write(cc & 0xFF);
  }
  Wire.endTransmission();
}

float getAzimuth() {
  if (digitalRead(DRDY_PIN) == LOW) return prevAzimuth;

  Wire.beginTransmission(RM3100_ADDR);
  Wire.write(0x24);
  Wire.endTransmission();
  Wire.requestFrom(RM3100_ADDR, 6);
  if (Wire.available() < 6) return prevAzimuth;

  long x = 0, y = 0;
  byte x2 = Wire.read(), x1 = Wire.read(), x0 = Wire.read();
  byte y2 = Wire.read(), y1 = Wire.read(), y0 = Wire.read();

  if (x2 & 0x80) x = 0xFF000000;
  if (y2 & 0x80) y = 0xFF000000;

  x |= ((long)x2 << 16) | ((long)x1 << 8) | x0;
  y |= ((long)y2 << 16) | ((long)y1 << 8) | y0;

  float xf = (float)x / GAIN;
  float yf = (float)y / GAIN;
  float az = atan2(yf, xf) * 180.0 / PI;
  if (az < 0) az += 360.0;
  return az;
}

void writeReg(byte addr, byte val) {
  Wire.beginTransmission(RM3100_ADDR);
  Wire.write(addr);
  Wire.write(val);
  Wire.endTransmission();
}

// --- Stepper ---
void rotateByAngle(float delta) {
  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(DIR_PIN, delta > 0 ? HIGH : LOW);
  int steps = abs(delta) * STEPS_PER_REV * MICROSTEPS / 360.0;
  log("Rotating " + String(delta, 1) + "° (" + String(steps) + " steps)");

  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }

  digitalWrite(ENABLE_PIN, HIGH);
  log("Rotation complete");
}

// --- EEPROM ---
void writeAzimuthToEEPROM(float az) {
  EEPROM.put(EEPROM_ADDR, az);
}

float readAzimuthFromEEPROM() {
  float val = 0.0;
  EEPROM.get(EEPROM_ADDR, val);
  if (isnan(val) || val < 0 || val > 360) return 0.0;
  return val;
}

// --- Commands ---
void handleCommand(String cmd, float az, String dir) {
  log("Command: " + cmd);
  if (cmd.startsWith("SET ")) {
    String angleStr = cmd.substring(4);
    float val = angleStr.toFloat();
    if (val >= 0 && val <= 360) {
      targetAzimuth = val;
      trackingEnabled = true;
      log("Target set to " + String(targetAzimuth, 1) + "°");
    } else {
      log("Invalid angle");
    }
  } else if (cmd == "NORTH") {
    targetAzimuth = 0;
    trackingEnabled = true;
    log("Turning to NORTH");
  } else if (cmd == "START") {
    trackingEnabled = true;
    log("Tracking started");
  } else if (cmd == "STOP") {
    trackingEnabled = false;
    log("Tracking stopped");
  } else if (cmd == "GET") {
    log("Azimuth: " + String(az, 1) + "° - " + dir);
  } else if (cmd == "STATUS") {
    log("Tracking: " + String(trackingEnabled ? "ON" : "OFF"));
    log("Current: " + String(az, 1) + "°, Target: " + String(targetAzimuth, 1));
  } else if (cmd == "RESET") {
    prevAzimuth = 0.0;
    writeAzimuthToEEPROM(0.0);
    log("Azimuth reset");
  } else {
    log("Unknown command");
  }
}

// --- Utility ---
void log(String msg) {
  Serial.println(msg);
  BT.println(msg);
}

String getCompassDirection(float az) {
  if (az < 22.5 || az >= 337.5) return "North";
  if (az < 67.5) return "Northeast";
  if (az < 112.5) return "East";
  if (az < 157.5) return "Southeast";
  if (az < 202.5) return "South";
  if (az < 247.5) return "Southwest";
  if (az < 292.5) return "West";
  return "Northwest";
}

float normalizeAngle(float a) {
  while (a < 0) a += 360;
  while (a >= 360) a -= 360;
  return a;
}
