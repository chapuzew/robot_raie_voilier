#include <Arduino.h>
#include <Servo.h>

Servo monServo;

void setup() {
  monServo.attach(10, 500, 2500); // 500–2500 µs sur broche 10 (jaune) et alimenter le servo en 12V en direct
  monServo.writeMicroseconds(500);
  delay(1000);
}

void loop() {
  // Balayage de 0 à 360 degrés
  for (int angle = 0; angle <= 360; angle++) {
    int pulseWidth = map(angle, 0, 360, 500, 2500);
    monServo.writeMicroseconds(pulseWidth);
    delay(10);
  }

  delay(600); // Pause à 360°

  // Retour de 360 à 0
  for (int angle = 360; angle >= 0; angle--) {
    int pulseWidth = map(angle, 0, 360, 500, 2500);
    monServo.writeMicroseconds(pulseWidth);
    delay(10);
  }

  delay(600); // Pause à 0°
}
