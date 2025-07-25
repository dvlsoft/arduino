#include <SoftwareSerial.h>
#include <Wire.h>

// Setup compass --- RM3100 ---
#define RM3100_ADDR 0x20
#define REG_POLL 0x00
#define REG_CMM 0x01
#define REG_STATUS 0x34
#define REG_CCX1 0x04
#define REG_CCX0 0x05
#define DRDY_PIN 7
#define CYCLE_COUNT 200
#define GAIN (0.3671 * CYCLE_COUNT + 1.5)

// Setup motor --- NEMA17 ---
#define STEP_PIN    4
#define DIR_PIN     3
#define ENABLE_PIN  2
#define MS1_PIN 7
#define MS2_PIN 8

const int stepsPerCommand = 10;
const int minDelayUs = 600;     // минимальная задержка (макс. скорость)
const int maxDelayUs = 1500;    // начальная задержка (разгон)
const int frequency = 2250;
bool busy = false;

// Setup connection--- Waveshare ---
SoftwareSerial ethSerial(10, 11);  // RX, TX

float prevAzimuth = 0.0;
float targetAzimuth = 0.0;

// ======= переменные таймера ========
int currentStep = 0;
int totalSteps = 0;
bool stepDir = true;
bool stepActive = false;

unsigned long lastStepTime = 0;
int stepDelayUs = maxDelayUs;

void setup() {
  pinMode(DRDY_PIN, INPUT);
  Wire.begin();
 
  Serial.begin(9600);
  ethSerial.begin(9600);
  Serial.println("Initializing compass...");
  initRM3100();

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, LOW);  // удержание
  Serial.println("Stepper controller with timer ready.");
  
  //Setup steps for motor
  digitalWrite(MS1_PIN, HIGH);
  digitalWrite(MS2_PIN, LOW);
  // Microstep  MS1   MS2
  // 1/2        High  Low
  // 1/4        Low   High
  // 1/8        Low   Low
  // 1/16       High  High
  printCompassValue(); //Initial data from compass
}

void loop() {
  // обработка команд
  if (ethSerial.available() && !stepActive) {
  String cmd = ethSerial.readStringUntil('\n');
  cmd.trim();

  if (cmd == "CW" || cmd == "CCW") {
    stepDir = (cmd == "CW");
    digitalWrite(DIR_PIN, stepDir ? HIGH : LOW);
    digitalWrite(ENABLE_PIN, LOW);
    Serial.println("Moving: " + cmd);

    totalSteps = stepsPerCommand;
    currentStep = 0;
    stepActive = true;
    lastStepTime = micros();
  } else {
    Serial.println("Invalid or idle cmd: " + cmd);
  }

  // сразу передаём азимут в ответ
  float az = getAzimuth();
  ethSerial.println(String(az, 1));  // отправка значения азимута обратно
  
  //Print azimuth to console
  Serial.print("Azimuth: ");
  Serial.println(az, 1);  // только значение, без направления
  flushSerial();
}


  // шагание по таймеру
  if (stepActive) {
    unsigned long now = micros();
    if (now - lastStepTime >= stepDelayUs) {
      lastStepTime = now;

      // вычисляем задержку для текущего шага (ускорение/торможение)
      int accelSteps = min(totalSteps / 2, 5);
      if (currentStep < accelSteps) {
        stepDelayUs = map(currentStep, 0, accelSteps, maxDelayUs, minDelayUs);
      } else if (currentStep >= totalSteps - accelSteps) {
        stepDelayUs = map(currentStep, totalSteps - accelSteps, totalSteps - 1, minDelayUs, maxDelayUs);
      } else {
        stepDelayUs = minDelayUs;
      }

      // сделать шаг
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(frequency);  // короткий импульс
      digitalWrite(STEP_PIN, LOW);

      currentStep++;
      if (currentStep >= totalSteps) {
        stepActive = false;
        Serial.println("Step sequence complete.");
      }
    }
  }
}

// Очистка приёмного буфера
void flushSerial() {
  while (ethSerial.available()) ethSerial.read();
}

// --- RM3100 и расчёт азимута ---
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

void printCompassValue(){
  // float az = getAzimuth();

  //   float delta = az - prevAzimuth;
  //   if (delta > 180) delta -= 360;
  //   if (delta < -180) delta += 360;

  //   float virtualAzimuth = az;
  //   if (delta < 0) {
  //     virtualAzimuth = 360.0 - az;
  //     if (virtualAzimuth == 360.0) virtualAzimuth = 0.0;
  //   }

  //   if (abs(virtualAzimuth - targetAzimuth) >= 1.0) {
  //     Serial.print("Azimuth: ");
  //     Serial.print(virtualAzimuth, 1);
  //     Serial.print("° - ");
  //     Serial.println(getCompassDirection(virtualAzimuth));
  //     targetAzimuth = virtualAzimuth;
  //   }

  //   prevAzimuth = az;
  float az = getAzimuth();

  if (abs(az - targetAzimuth) >= 1.0) {
    Serial.print("Azimuth: ");
    Serial.println(az, 1);  // только значение, без направления
    targetAzimuth = az;
  }

  prevAzimuth = az;
}

