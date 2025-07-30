#ifndef DIFFERENTIAL_H
#define DIFFERENTIAL_H

#include <Servo.h>
#include <Arduino.h>
#include <math.h> 

struct ServoPositions {
    int pos1;
    int pos2;
};


class Differential {
public:
    Differential(int pin1, int pin2, int angleMin, int angleMax);

    void setup();
    void tournerAxeSurLuiMeme(int deltaAngle); // Rotation sur place
    void translaterAxe(int deltaAngle);        // Mouvement longitudinal

    int getPositionRotation() const;
    int getPositionTranslation() const;
    ServoPositions getPositionServos() const ;

    void setAmplitudeMaxTranslation(float amplitudeDeg);
    void sinusoideTranslationVitesseLisse(float amplitudeDeg, float vitesseDegPerSec, int nbCycles);
    void sinusoideRotationVitesseLisse(float amplitudeDeg, float vitesseDegPerSec, int nbCycles);
    void sinusoideCombineeVitesseLisse(
    float amplitudeTranslationDeg, float amplitudeRotationDeg, float vitesseDegPerSec, int nbCycles);
    
    void voilier();
    void voilierdescente();

    // protected:
    Servo servo1, servo2;
    int pin1, pin2;
    int angleMin, angleMax;

    int positionRotation;
    int positionTranslation;
    int positionServo1 = 180;
    int positionServo2 = 180;

    void setServos(int angle1, int angle2);
    int angleToPulse(int angle) const;

    float amplitudeMaxTranslation = 45.0;  // amplitude max en degrés
    float lastSinAngle = 0;
};

#endif
