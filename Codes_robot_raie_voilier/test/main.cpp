#include "Differential.h"

Differential diff1(9, 11, 0, 360); // broches 9 et 11, angles limités entre 0° et 180°
Differential diff2(5, 6, 0, 360); // broches 9 et 11, angles limités entre 0° et 180°


void setup() {
    diff1.setup();
    diff2.setup();
  }
  
  
void loop() {
// diff1.tournerAxeSurLuiMeme(90);

// diff1.tournerAxeSurLuiMeme(45);
// delay(1000);

// diff1.tournerAxeSurLuiMeme(-45);
// delay(1000);

// diff1.tournerAxeSurLuiMeme(-45);
// delay(1000);

// diff1.tournerAxeSurLuiMeme(45);
// delay(1000);

// oscillerServoParPas();
// servoOscillationRelative(45.0, 4000);  // Oscille en 4 secondes (aller-retour)
// diff1.sinusoideTranslationVitesseLisse(45, 70, 3);  // 3 cycles à 5°/s avec amplitude 20°
// diff1.sinusoideRotationVitesseLisse(45, 40, 2); // amplitude 20°, vitesse 10°/s, 2 cycles
// diff1.sinusoideCombineeVitesseLisse(45, 45, 150, 3);

}