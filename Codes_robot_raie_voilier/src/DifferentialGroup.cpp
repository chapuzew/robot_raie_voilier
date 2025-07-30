#include "DifferentialGroup.h"

DifferentialGroup::DifferentialGroup(Differential& d1, Differential& d2)
    : diff1(d1), diff2(d2) {}

void DifferentialGroup::sinusoideTranslationSynchronisee(float amplitudeDeg, float vitesseDegPerSec, float phaseShiftRad) {
    float amplitudeServo = amplitudeDeg * 2;
    float omega = vitesseDegPerSec / amplitudeDeg;
    float period = 2 * M_PI / omega;

    const unsigned long dt_ms = 5;
    unsigned long startMillis = millis();

    while ((millis() - startMillis) / 1000.0 < period) {
        float t = (millis() - startMillis) / 1000.0;
        float angle1 = amplitudeServo * sin(omega * t);
        float angle2 = amplitudeServo * sin(omega * t + phaseShiftRad);

        int base1a = (diff1.angleMin + diff1.angleMax) / 2 + angle1 / 2;
        int base1b = (diff1.angleMin + diff1.angleMax) / 2 - angle1 / 2;
        int base2a = (diff2.angleMin + diff2.angleMax) / 2 + angle2 / 2;
        int base2b = (diff2.angleMin + diff2.angleMax) / 2 - angle2 / 2;

        diff1.setServos(constrain(base1a, diff1.angleMin, diff1.angleMax),
                        constrain(base1b, diff1.angleMin, diff1.angleMax));
        diff2.setServos(constrain(base2a, diff2.angleMin, diff2.angleMax),
                        constrain(base2b, diff2.angleMin, diff2.angleMax));

        delay(dt_ms);
    }
}
    

void DifferentialGroup::sinusoideRotationSynchronisee(float amplitudeDeg, float vitesseDegPerSec, float phaseShiftRad) {
    const float rapportReductionRotation = 4.0;
    float amplitudeServo = amplitudeDeg * rapportReductionRotation;
    float omega = vitesseDegPerSec / amplitudeDeg;
    float period = 2 * M_PI / omega;

    const unsigned long dt_ms = 5;
    unsigned long startMillis = millis();

    while ((millis() - startMillis) / 1000.0 < period) {
        float t = (millis() - startMillis) / 1000.0;
        float angle1 = amplitudeServo * sin(omega * t);
        float angle2 = -amplitudeServo * sin(omega * t + phaseShiftRad);  // Inversion du signal ici

        int base1 = (diff1.angleMin + diff1.angleMax) / 2 + angle1 / 2;
        int base2 = (diff2.angleMin + diff2.angleMax) / 2 + angle2 / 2;

        diff1.setServos(constrain(base1, diff1.angleMin, diff1.angleMax),
                        constrain(base1, diff1.angleMin, diff1.angleMax));
        diff2.setServos(constrain(base2, diff2.angleMin, diff2.angleMax),
                        constrain(base2, diff2.angleMin, diff2.angleMax));

        delay(dt_ms);
    }
}


void DifferentialGroup::sinusoideCombineeSynchronisee(float amplitudeTranslationDeg, float amplitudeRotationDeg, float vitesseDegPerSec, float phaseShiftRad) {
    const float rapportReductionRotation = 4.0;

    float amplitudeServoTranslation = amplitudeTranslationDeg * 2;
    float amplitudeServoRotation = amplitudeRotationDeg * rapportReductionRotation;
    float omega = vitesseDegPerSec / max(amplitudeTranslationDeg, amplitudeRotationDeg);
    float period = 2 * M_PI / omega;

    const unsigned long dt_ms = 5;
    unsigned long startMillis = millis();

    while ((millis() - startMillis) / 1000.0 < period) {
        float t = (millis() - startMillis) / 1000.0;

        float angleTranslation1 = amplitudeServoTranslation * sin(omega * t);
        float angleRotation1 = -amplitudeServoRotation * sin(omega * t);  // inversion ici

        float angleTranslation2 = amplitudeServoTranslation * sin(omega * t + phaseShiftRad);
        float angleRotation2 = amplitudeServoRotation * sin(omega * t + phaseShiftRad);

        int pos1a = (diff1.angleMin + diff1.angleMax) / 2 + (angleRotation1 / 2) + (angleTranslation1 / 2);
        int pos1b = (diff1.angleMin + diff1.angleMax) / 2 + (angleRotation1 / 2) - (angleTranslation1 / 2);
        int pos2a = (diff2.angleMin + diff2.angleMax) / 2 + (angleRotation2 / 2) + (angleTranslation2 / 2);
        int pos2b = (diff2.angleMin + diff2.angleMax) / 2 + (angleRotation2 / 2) - (angleTranslation2 / 2);

        diff1.setServos(constrain(pos1a, diff1.angleMin, diff1.angleMax),
                        constrain(pos1b, diff1.angleMin, diff1.angleMax));
        diff2.setServos(constrain(pos2a, diff2.angleMin, diff2.angleMax),
                        constrain(pos2b, diff2.angleMin, diff2.angleMax));

        delay(dt_ms);
    }
}




void DifferentialGroup::carreLisserotationMontDescIndependants(float amplitudeDeg, float vitesseDegPerSec, float phaseShiftRad, float dureeMontee, float dureeDescente) {
    const float rapportReductionRotation = 4.0;
    float amplitudeServo = amplitudeDeg * rapportReductionRotation;

    float omega = vitesseDegPerSec / amplitudeDeg;
    float period = 2 * M_PI / omega;
    float demiPeriode = period / 2.0;

    float plateauHaut = demiPeriode - dureeMontee;
    float plateauBas  = demiPeriode - dureeDescente;

    const unsigned long dt_ms = 5;
    unsigned long startMillis = millis();

    auto signalLisse = [&](float tCycle, float plateau, float dureeTransition, bool isMontee) -> float {
        if (tCycle < plateau) {
            return isMontee ? 1.0 : -1.0;
        } else if (tCycle < plateau + dureeTransition) {
            float ratio = (tCycle - plateau) / dureeTransition;
            return isMontee ? (1.0 - 2.0 * ratio) : (-1.0 + 2.0 * ratio);
        } else {
            return isMontee ? -1.0 : 1.0;
        }
    };

    while ((millis() - startMillis) / 1000.0 < period) {
        float t = (millis() - startMillis) / 1000.0;

        float t1 = fmod(t, period);
        float t2 = fmod(t + phaseShiftRad / omega, period); 

        float signal1, signal2;

        if (t1 < demiPeriode) {
            signal1 = signalLisse(t1, plateauHaut, dureeMontee, true);
        } else {
            signal1 = signalLisse(t1 - demiPeriode, plateauBas, dureeDescente, false);
        }

        if (t2 < demiPeriode) {
            signal2 = signalLisse(t2, plateauHaut, dureeMontee, true);
        } else {
            signal2 = signalLisse(t2 - demiPeriode, plateauBas, dureeDescente, false);
        }

        signal2 = -signal2;

        float angle1 = amplitudeServo * signal1;
        float angle2 = amplitudeServo * signal2;

        int base1 = (diff1.angleMin + diff1.angleMax) / 2 + angle1 / 2;
        int base2 = (diff2.angleMin + diff2.angleMax) / 2 + angle2 / 2;

        diff1.setServos(constrain(base1, diff1.angleMin, diff1.angleMax),
                        constrain(base1, diff1.angleMin, diff1.angleMax));
        diff2.setServos(constrain(base2, diff2.angleMin, diff2.angleMax),
                        constrain(base2, diff2.angleMin, diff2.angleMax));

        delay(dt_ms);
    }
}

void DifferentialGroup::carreLisseTranslationMontDescIndependants(float amplitudeDeg, float vitesseDegPerSec, float phaseShiftRad, float dureeMontee, float dureeDescente) {
    float amplitudeServo = amplitudeDeg * 2;  // facteur pour translation
    float omega = vitesseDegPerSec / amplitudeDeg;
    float period = 2 * M_PI / omega;
    float demiPeriode = period / 2.0;

    float plateauHaut = demiPeriode - dureeMontee;
    float plateauBas  = demiPeriode - dureeDescente;

    const unsigned long dt_ms = 5;
    unsigned long startMillis = millis();

    auto signalLisse = [&](float tCycle, float plateau, float dureeTransition, bool isMontee) -> float {
        if (tCycle < plateau) {
            return isMontee ? 1.0 : -1.0;
        } else if (tCycle < plateau + dureeTransition) {
            float ratio = (tCycle - plateau) / dureeTransition;
            return isMontee ? (1.0 - 2.0 * ratio) : (-1.0 + 2.0 * ratio);
        } else {
            return isMontee ? -1.0 : 1.0;
        }
    };

    while ((millis() - startMillis) / 1000.0 < period) {
        float t = (millis() - startMillis) / 1000.0;

        float t1 = fmod(t, period);
        float t2 = fmod(t + phaseShiftRad / omega, period);

        float signal1, signal2;

        if (t1 < demiPeriode) {
            signal1 = signalLisse(t1, plateauHaut, dureeMontee, true);
        } else {
            signal1 = signalLisse(t1 - demiPeriode, plateauBas, dureeDescente, false);
        }

        if (t2 < demiPeriode) {
            signal2 = signalLisse(t2, plateauHaut, dureeMontee, true);
        } else {
            signal2 = signalLisse(t2 - demiPeriode, plateauBas, dureeDescente, false);
        }

        float angle1 = amplitudeServo * signal1;
        float angle2 = amplitudeServo * signal2;

        int base1a = (diff1.angleMin + diff1.angleMax) / 2 + angle1 / 2;
        int base1b = (diff1.angleMin + diff1.angleMax) / 2 - angle1 / 2;
        int base2a = (diff2.angleMin + diff2.angleMax) / 2 + angle2 / 2;
        int base2b = (diff2.angleMin + diff2.angleMax) / 2 - angle2 / 2;

        diff1.setServos(constrain(base1a, diff1.angleMin, diff1.angleMax),
                        constrain(base1b, diff1.angleMin, diff1.angleMax));
        diff2.setServos(constrain(base2a, diff2.angleMin, diff2.angleMax),
                        constrain(base2b, diff2.angleMin, diff2.angleMax));

        delay(dt_ms);
    }
}

void DifferentialGroup::carreLisseTranslationMontDescIndependantsAdoucis(float amplitudeDeg, float vitesseDegPerSec, float phaseShiftRad, float dureeMontee, float dureeDescente) {
    float amplitudeServo = amplitudeDeg * 2;  // facteur pour translation
    float omega = vitesseDegPerSec / amplitudeDeg;
    float period = 2 * M_PI / omega;
    float demiPeriode = period / 2.0;

    float plateauHaut = demiPeriode - dureeMontee;
    float plateauBas  = demiPeriode - dureeDescente;

    const unsigned long dt_ms = 5;
    unsigned long startMillis = millis();

    // Transition sinusoïdale (cos pour adoucir les fronts)
    auto signalLisse = [&](float tCycle, float plateau, float dureeTransition, bool isMontee) -> float {
        if (tCycle < plateau) {
            return isMontee ? 1.0 : -1.0;
        } else if (tCycle < plateau + dureeTransition) {
            float ratio = (tCycle - plateau) / dureeTransition;
            float smooth = 0.5 * (1.0 - cos(ratio * M_PI)); 
            return isMontee ? (1.0 - 2.0 * smooth) : (-1.0 + 2.0 * smooth);
        } else {
            return isMontee ? -1.0 : 1.0;
        }
    };

    while ((millis() - startMillis) / 1000.0 < period) {
        float t = (millis() - startMillis) / 1000.0;

        float t1 = fmod(t, period);
        float t2 = fmod(t + phaseShiftRad / omega, period);

        float signal1, signal2;

        if (t1 < demiPeriode) {
            signal1 = signalLisse(t1, plateauHaut, dureeMontee, true);
        } else {
            signal1 = signalLisse(t1 - demiPeriode, plateauBas, dureeDescente, false);
        }

        if (t2 < demiPeriode) {
            signal2 = signalLisse(t2, plateauHaut, dureeMontee, true);
        } else {
            signal2 = signalLisse(t2 - demiPeriode, plateauBas, dureeDescente, false);
        }

        float angle1 = amplitudeServo * signal1;
        float angle2 = amplitudeServo * signal2;

        int base1a = (diff1.angleMin + diff1.angleMax) / 2 + angle1 / 2;
        int base1b = (diff1.angleMin + diff1.angleMax) / 2 - angle1 / 2;
        int base2a = (diff2.angleMin + diff2.angleMax) / 2 + angle2 / 2;
        int base2b = (diff2.angleMin + diff2.angleMax) / 2 - angle2 / 2;

        diff1.setServos(constrain(base1a, diff1.angleMin, diff1.angleMax),
                        constrain(base1b, diff1.angleMin, diff1.angleMax));
        diff2.setServos(constrain(base2a, diff2.angleMin, diff2.angleMax),
                        constrain(base2b, diff2.angleMin, diff2.angleMax));

        delay(dt_ms);
    }
}



void DifferentialGroup::voilierSynchronise() {
    int startPos1 = 1500;
    int startPos2 = 1500;
    int endPos1 = 2000;
    int endPos2 = 1000;
    int step = 10;
    int delayTime = 20;

    // Montée progressive translation : diff1 va "avant", diff2 va "arrière"
    for (int pos = 0; pos <= (endPos1 - startPos1); pos += step) {
        // diff1 : servo1 monte, servo2 descend
        diff1.setServos(startPos1 + pos, startPos2 - pos);
        // diff2 : servo1 descend, servo2 monte (inverse de diff1)
        diff2.setServos(endPos1 + pos, endPos2 - pos);
        delay(delayTime);
    }

    // Correction finale
    diff1.setServos(endPos1, endPos2);
    diff2.setServos(startPos1, startPos2);

    delay(5000); // pause en haut

    // Descente progressive translation (inverse de la montée)
    for (int pos = 0; pos <= (endPos1 - startPos1); pos += step) {
        // diff1 : descente servo1, montée servo2
        diff1.setServos(endPos1 - pos, endPos2 - pos);
        // diff2 : montée servo1, descente servo2 (inverse de diff1)
        diff2.setServos(startPos1 + pos, startPos2 + pos);
        delay(delayTime);
    }

    // Correction finale position départ
    diff1.setServos(startPos1, startPos2);
    diff2.setServos(endPos1, endPos2);

    // Blocage indéfini
    while (true) {
        delay(1000);
    }
}
    
    
    
