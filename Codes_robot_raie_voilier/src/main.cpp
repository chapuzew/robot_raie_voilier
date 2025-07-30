#include "Differential.h"
#include "DifferentialGroup.h"
#include "DifferentialServer.h"

Differential diff1(9, 6, 0, 360); // Configuration de la premiére epaule sur les pin 9 et 6
Differential diff2(3, 5, 0, 360); // Configuration de la seconde epaule sur les pin 3 et 5
DifferentialGroup group(diff1, diff2);
DifferentialServer server(80);


void setup() {
    Serial.begin(9600);
    Serial.println("Démarrage OK");

    server.begin();

    // Initialisation des quatres servo a la position 180
    diff1.setup();
    diff2.setup();
}

void loop() {

    int a, b, c, d, e, f, mode =0;

    while (true) {

        if (server.update(a, b, c, d, e, f, mode)) { // a = angle translation (degrés), b = angle rotation (degrés), c = periode en radiant/seconde, d = dephasage entre les deux ailes, e/f = pourcentage de la periode que prend le temps de monté/descante 
            Serial.print("Reçu : ");
            // Serial.print(a); Serial.print(", ");
            // Serial.print(b); Serial.print(", ");
            // Serial.print(c); Serial.print(", ");
            // Serial.print(d); Serial.print(", ");
            // Serial.print(e); Serial.print(", ");
            // Serial.print(f); Serial.print(", ");
            // Serial.println(mode);*
        }
        
        if (mode == 0) ;

        else if (mode == 1) group.sinusoideTranslationSynchronisee(a, c, d);

        else if (mode == 2) group.sinusoideRotationSynchronisee(a, c, d);

        else if (mode == 3) group.sinusoideCombineeSynchronisee(a, b, c, d);

        else if (mode == 4) group.carreLisserotationMontDescIndependants(a, c, d, e, f);
        
        else if (mode == 5) group.carreLisseTranslationMontDescIndependants(a, c, d, e, f);

        else if (mode == 6) group.carreLisseTranslationMontDescIndependantsAdoucis(a, c, d, e, f);

        else if (mode == 7) {
            diff1.voilier();
            diff2.voilier();
            delay(7000);
            diff1.voilierdescente();
            diff2.voilierdescente();

            delay(100000);

        }

    }
// tests sans la comuunication

//   group.sinusoideTranslationSynchronisee(45, 120, 0);
//   group.sinusoideRotationSynchronisee(45, 40, 0);
//   group.sinusoideCombineeSynchronisee(45, 45, 120, 0);
//   group.carreLisserotationMontDescIndependants(30, 60, 0, 0.2, 0.2);
//   group.carreLisseTranslationMontDescIndependants(30.0, 60.0, 0.0, 0.1, 0.1);
//   group.carreLisseTranslationMontDescIndependants(30.0, 60.0, 0.0, 0.3, 0.3);

}











// import requests

// def envoyer_valeurs(v1, v2, v3, v4, v5):
//     url = f"http://192.168.1.2/cmd/{v1}/{v2}/{v3}/{v4}/{v5}"
//     try:
//         r = requests.get(url, timeout=2)
//         print("Réponse Arduino :", r.text.strip())
//     except requests.exceptions.RequestException as e:
//         print("Erreur :", e)

// envoyer_valeurs(45, 45, 40, 3, 0)

