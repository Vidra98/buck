#ifndef RECEPTION_CAPTEUR_IR_H
#define RECEPTION_CAPTEUR_IR_H

#include <stdint.h>
#include <stdbool.h>

//numérotation des capteurs
#define CAPTEUR_HAUT_DROITE 	0
#define CAPTEUR_HAUT_DROITE_45	1
#define CAPTEUR_DROITE			2
#define CAPTEUR_BAS_DROITE 		3
#define CAPTEUR_BAS_GAUCHE 		4
#define CAPTEUR_GAUCHE 			5
#define CAPTEUR_HAUT_GAUCHE_45	6
#define CAPTEUR_HAUT_GAUCHE 	7

//en vue des différentes valeurs aléatoires qui pouvait arriver sur les capteurs
//nous avons décidé d'interpreter l'arrivée d'un obstacle comme 3 valeurs d'affilée sur un capteur dépassant les valeurs seuil
#define MESURES_AFFILE			3
#define OBSTACLE_ENV_5CM		150
#define OBSTACLE_ENV_2CM		600
#define OBSTACLE_EN_CONTACT 	2000


//valeurs mesurées avec un fond blanc en s'approchant des capteurs


//les capteurs sont numerotées dans le sens horaire en commencant par celui en haut a droite.
// Les mesures sont faites à des frequences de 800Hz soit 100Hz par capteur
// Les mesures sont faites par paires opposées afin de pas avoir des interferences : IR0 et IR4 puis IR1 et IR5 puis IR2 et IR6 puis IR3 et IR7


/* Fonction qui détermine le chemin à prendre (contournement par la droite ou la gauche)
 * J'ai mis par défaut (ie pas d'obstacle sur les cotés) le contournement par la gauche
 */
int16_t check_chemin(void);

// Fonction qui renvoie une valeur calibrée d'un capteur, pour le PI
// Est appelée uniquement si nécessaire
uint32_t valeur_sur_capteur(int32_t num_capteur);

// Fonction qui determine si le capteur concerné voit un obstacle ou non
// Est appelée dans le threads pour définir l'environnement
bool get_obstacle_condition(int32_t num_capteur);

//Focntion qui remet tous les bools à 0 et refresh les données des capteurs
//passe les bools à 1 si obstacle en vue
// Est appelée dans le thread pour refresh l'environnement
void valeurs_calibrees(void);

void valeurs_ambiantes(void);

void valeurs_absolues(void);


#endif
