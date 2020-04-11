#ifndef RECEPTION_CAPTEUR_IR_H
#define RECEPTION_CAPTEUR_IR_H

#include <stdint.h>

//num�rotation des capteurs
#define CAPTEUR_HAUT_DROITE 	0
#define CAPTEUR_HAUT_DROITE_45	1
#define CAPTEUR_DROITE			2
#define CAPTEUR_BAS_DROITE 		3
#define CAPTEUR_BAS_GAUCHE 		4
#define CAPTEUR_GAUCHE 			5
#define CAPTEUR_HAUT_GAUCHE_45	6
#define CAPTEUR_HAUT_GAUCHE 	7
#define PAS_DE_CAPTEUR			8

//en vue des diff�rentes valeurs al�atoires qui pouvait arriver sur les capteurs
//nous avons d�cid� d'interpreter l'arriv�e d'un obstacle comme 3 valeurs d'affil�e sur un capteur d�passant les valeurs seuil
#define MESURES_AFFILE			3
#define OBSTACLE_ENV_5CM		150
#define OBSTACLE_ENV_2CM		600
#define OBSTACLE_EN_CONTACT 	2000


//valeurs mesur�es avec un fond blanc en s'approchant des capteurs


//les capteurs sont numerot�es dans le sens horaire en commencant par celui en haut a droite.
// Les mesures sont faites � des frequences de 800Hz soit 100Hz par capteur
// Les mesures sont faites par paires oppos�es afin de pas avoir des interferences : IR0 et IR4 puis IR1 et IR5 puis IR2 et IR6 puis IR3 et IR7


void valeurs_ambiantes(void);

void valeurs_calibrees(void);

void valeurs_absolues(void);

int32_t check_environnement(void);

#endif
