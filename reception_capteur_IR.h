#ifndef RECEPTION_CAPTEUR_IR_H
#define RECEPTION_CAPTEUR_IR_H

#define CAPTEUR_HAUT_DROITE 	0
#define CAPTEUR_HAUT_DROITE_45	1
#define CAPTEUR_DROITE			2
#define CAPTEUR_BAS_DROITE 		3
#define CAPTEUR_BAS_GAUCHE 		4
#define CAPTEUR_GAUCHE 			5
#define CAPTEUR_HAUT_GAUCHE_45	6
#define CAPTEUR_HAUT_GAUCHE 	7

//les capteurs sont numerotées dans le sens horaire en commencant par celui en haut a droite.
// Les mesures sont faites à des frequences de 800Hz soit 100Hz par capteur
// Les mesures sont faites par paires opposées afin de pas avoir des interferences : IR0 et IR4 puis IR1 et IR5 puis IR2 et IR6 puis IR3 et IR7


void valeurs_ambiantes(void);

void valeurs_calibrees(void);

void valeurs_absolues(void);

#endif
