#ifndef DIFFERENTIAL_GROUP_H
#define DIFFERENTIAL_GROUP_H

#include "Differential.h"

class DifferentialGroup {
public:
    DifferentialGroup(Differential& d1, Differential& d2);

    void sinusoideTranslationSynchronisee(float amplitudeDeg, float vitesseDegPerSec, float phaseShiftRad);
    void sinusoideRotationSynchronisee(float amplitudeDeg, float vitesseDegPerSec, float phaseShiftRad);
    void sinusoideCombineeSynchronisee(float amplitudeTranslationDeg, float amplitudeRotationDeg, float vitesseDegPerSec, float phaseShiftRad);
    void carreLisserotationMontDescIndependants(float amplitudeDeg, float vitesseDegPerSec, float phaseShiftRad, float dureeMontee,float dureeDescente);
    void carreLisseTranslationMontDescIndependants(float amplitudeDeg, float vitesseDegPerSec, float phaseShiftRad, float dureeMontee, float dureeDescente);
    void carreLisseTranslationMontDescIndependantsAdoucis(float amplitudeDeg, float vitesseDegPerSec, float phaseShiftRad, float dureeMontee, float dureeDescente);
    
    void voilierSynchronise();

private:
    Differential& diff1;
    Differential& diff2;
};

#endif
