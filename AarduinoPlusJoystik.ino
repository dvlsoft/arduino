#include <SoftwareSerial.h>

//Joystick setup
#define JOY_X A0 //Input pin
#define DEADZONE 30

// Setup connection--- Waveshare ---
SoftwareSerial ethSerial(10, 11);  // RX, TX

void setup() {
  Serial.begin(9600);
  ethSerial.begin(9600);
  Serial.println("Joystick sender ready.");
}

void loop() {
  int x = analogRead(JOY_X) - 512;

  String cmd = "0";
  if (x > DEADZONE) {
    cmd = "CW";
  } else if (x < -DEADZONE) {
    cmd = "CCW";
  }

  if (cmd != "0") {
    ethSerial.println(cmd);        // отправка команды
    Serial.println("Sent: " + cmd);

    // ждём ответ — азимут
    unsigned long start = millis();
    while (!ethSerial.available() && millis() - start < 500);  // таймаут 500мс

    if (ethSerial.available()) {
      String response = ethSerial.readStringUntil('\n');
      response.trim();
      Serial.println("Azimuth: " + response);
    } else {
      Serial.println("No azimuth response");
    }
  }

  delay(100);  // задержка между циклами
}

