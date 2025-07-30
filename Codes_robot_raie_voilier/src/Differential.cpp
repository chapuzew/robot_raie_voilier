#include "Differential.h"

Differential::Differential(int pin1, int pin2, int angleMin, int angleMax)
    : pin1(pin1), pin2(pin2), angleMin(angleMin), angleMax(angleMax),
      positionRotation(0), positionTranslation((angleMin + angleMax) / 2){
}

void Differential::setup(){
    Serial.begin(9600); // Initialise la communication série

    servo1.attach(pin1, 500, 2500);
    servo2.attach(pin2, 500, 2500);
    servo1.writeMicroseconds(1500);
    servo2.writeMicroseconds(1500);
    delay(1000);
}

void Differential::voilier() {
    int startPos1 = 1500;
    int startPos2 = 1500;
    int endPos1 = 2000;
    int endPos2 = 1000;
    int step = 10;         // taille du pas de déplacement
    int delayTime = 20;    // temps en ms entre chaque incrément

    // Montée progressive
    for (int pos = 0; pos <= (endPos1 - startPos1); pos += step) {
        servo1.writeMicroseconds(startPos1 + pos);
        servo2.writeMicroseconds(startPos2 - pos);
        delay(delayTime);
    }
    // Correction finale pour s'assurer d'être pile à la position finale
    servo1.writeMicroseconds(endPos1);
    servo2.writeMicroseconds(endPos2);

}

void Differential::voilierdescente() {
    int startPos1 = 1500;
    int startPos2 = 1500;
    int endPos1 = 2000;
    int endPos2 = 1000;
    int step = 10;         // taille du pas de déplacement
    int delayTime = 20;  

    // Descente progressive (inverse de la montée)
    for (int pos = 0; pos <= (endPos1 - startPos1); pos += step) {
        servo1.writeMicroseconds(endPos1 - pos);
        servo2.writeMicroseconds(endPos2 + pos);
        delay(delayTime);
    }
    // Correction finale pour être sûr d'être pile à la position de départ
    servo1.writeMicroseconds(startPos1);
    servo2.writeMicroseconds(startPos2);


}


void Differential::tournerAxeSurLuiMeme(int deltaAngle) {
    const float rapportReduction = 2;  // Ajuste selon ton système mécanique

    // Conversion de l'angle d'axe en angle servo
    int deltaServo = (int)(deltaAngle * rapportReduction);

    // Propositions de nouvelles positions servo
    int newPos1 = positionServo1 + deltaServo;
    int newPos2 = positionServo2 + deltaServo;

    // Contraintes limites servo
    int constrained1 = constrain(newPos1, angleMin, angleMax);
    int constrained2 = constrain(newPos2, angleMin, angleMax);

    // Calcul des deltas réellement possibles
    int delta1 = constrained1 - positionServo1;
    int delta2 = constrained2 - positionServo2;

    // On prend le delta minimal possible pour respecter les limites
    int actualDelta = min(abs(delta1), abs(delta2));
    if (deltaServo < 0) actualDelta = -actualDelta;

    // Mise à jour des positions servo
    positionServo1 += actualDelta;
    positionServo2 += actualDelta;

    setServos(positionServo1, positionServo2);

    // // debug
    // Serial.println("----- Rotation de l'axe -----");
    // Serial.print("Demande de deltaAngle (axe) : ");
    // Serial.println(deltaAngle);
    // Serial.print("Delta servo appliqué : ");
    // Serial.println(actualDelta);
    // Serial.print("positionServo1 : ");
    // Serial.println(positionServo1);
    // Serial.print("positionServo2 : ");
    // Serial.println(positionServo2);
    // Serial.println("--------------------------------");
}


void Differential::translaterAxe(int deltaAngle) {
    // Propositions de nouvelles positions
    int newPos1 = positionServo1 + deltaAngle;
    int newPos2 = positionServo2 - deltaAngle;

    // Contraintes
    int constrained1 = constrain(newPos1, angleMin, angleMax);
    int constrained2 = constrain(newPos2, angleMin, angleMax);

    // Calculs des deltas réellement possibles
    int delta1 = constrained1 - positionServo1;
    int delta2 = positionServo2 - constrained2; // signe inversé car on soustrait


    int actualDelta = min(abs(delta1), abs(delta2));
    if (deltaAngle < 0) actualDelta = -actualDelta;

    // Mise à jour des nouvelles positions valides
    positionServo1 += actualDelta;
    positionServo2 -= actualDelta;

    setServos(positionServo1, positionServo2);

    // // débug
    // Serial.println("----- Translation de l'axe -----");
    // Serial.print("Demande de deltaAngle : ");
    // Serial.println(deltaAngle);
    // Serial.print("Delta effectivement appliqué : ");
    // Serial.println(actualDelta);
    // Serial.print("positionServo1 : ");
    // Serial.println(positionServo1);
    // Serial.print("positionServo2 : ");
    // Serial.println(positionServo2);
    // Serial.println("--------------------------------");
}


void Differential::sinusoideTranslationVitesseLisse(float amplitudeDeg, float vitesseDegPerSec, int nbCycles) {
    float amplitudeServo = amplitudeDeg * 2;
    float omega = vitesseDegPerSec / amplitudeDeg;
    float period = 2 * M_PI / omega;
    float totalTime = nbCycles * period;

    const unsigned long dt_ms = 5;  // 200 Hz
    unsigned long startMillis = millis();

    while ((millis() - startMillis) / 1000.0 < totalTime) {
        float t = (millis() - startMillis) / 1000.0;
        float angle = amplitudeServo * sin(omega * t);

        int base1 = (angleMin + angleMax) / 2 + angle / 2;
        int base2 = (angleMin + angleMax) / 2 - angle / 2;

        base1 = constrain(base1, angleMin, angleMax);
        base2 = constrain(base2, angleMin, angleMax);

        setServos(base1, base2);

        delay(dt_ms);
    }
}

void Differential::sinusoideRotationVitesseLisse(float amplitudeDeg, float vitesseDegPerSec, int nbCycles) {
    float amplitudeServo = amplitudeDeg * 4;  // réduction 2:1
    float omega = vitesseDegPerSec / amplitudeDeg;
    float period = 2 * M_PI / omega;
    float totalTime = nbCycles * period;

    const unsigned long dt_ms = 5;  // 200 Hz
    unsigned long startMillis = millis();

    while ((millis() - startMillis) / 1000.0 < totalTime) {
        float t = (millis() - startMillis) / 1000.0;
        float angle = amplitudeServo * sin(omega * t);

        // Mouvement parallèle (rotation pure)
        int base1 = (angleMin + angleMax) / 2 + angle / 2;
        int base2 = (angleMin + angleMax) / 2 + angle / 2;

        base1 = constrain(base1, angleMin, angleMax);
        base2 = constrain(base2, angleMin, angleMax);

        setServos(base1, base2);

        delay(dt_ms);
    }
}


void Differential::sinusoideCombineeVitesseLisse(
    float amplitudeTranslationDeg,
    float amplitudeRotationDeg,
    float vitesseDegPerSec,
    int nbCycles) {
    // Rapport de réduction pour la rotation (ajuster si besoin)
    const float rapportReductionRotation = 4.0;

    // Calcul des amplitudes servo
    float amplitudeServoTranslation = amplitudeTranslationDeg * 2;          // translation servo = 2 * translation axe
    float amplitudeServoRotation = amplitudeRotationDeg * rapportReductionRotation;  // rotation servo = rapport * rotation axe

    // Calcul de la pulsation (on peut prendre la même vitesse pour rotation et translation)
    float omega = vitesseDegPerSec / max(amplitudeTranslationDeg, amplitudeRotationDeg);
    float period = 2 * M_PI / omega;
    float totalTime = nbCycles * period;

    const unsigned long dt_ms = 5;  // fréquence de mise à jour (~200 Hz)
    unsigned long startMillis = millis();

    while ((millis() - startMillis) / 1000.0 < totalTime) {
        float t = (millis() - startMillis) / 1000.0;

        // Calcul angles servo translation et rotation (sinusoïdes)
        float angleTranslation = amplitudeServoTranslation * sin(omega * t);
        float angleRotation = amplitudeServoRotation * sin(omega * t);

        // Calcul positions servo combinées
        // Translation = servos en opposition (+angle, -angle)
        // Rotation = servos en parallèle (+angle, +angle)

        int posServo1 = (angleMin + angleMax) / 2 + (angleRotation / 2) + (angleTranslation / 2);
        int posServo2 = (angleMin + angleMax) / 2 + (angleRotation / 2) - (angleTranslation / 2);

        posServo1 = constrain(posServo1, angleMin, angleMax);
        posServo2 = constrain(posServo2, angleMin, angleMax);

        setServos(posServo1, posServo2);

        delay(dt_ms);
    }
}









void Differential::setServos(int angle1, int angle2) {
    Serial.flush();
    servo1.writeMicroseconds(angleToPulse(angle1));
    delay(10);
    servo2.writeMicroseconds(angleToPulse(angle2));
    // delay(300); // petit délai pour laisser les servos s'ajuster
}

int Differential::angleToPulse(int angle) const {
    return map(angle, 0, 360, 500, 2500);
}

int Differential::getPositionRotation() const {
    return positionRotation;
}

int Differential::getPositionTranslation() const {
    return positionTranslation;
}

ServoPositions Differential::getPositionServos() const {
    return {positionServo1, positionServo2};
}





void Differential::setAmplitudeMaxTranslation(float amplitudeDeg) {
    amplitudeMaxTranslation = amplitudeDeg;
}
