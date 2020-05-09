#ifndef RECEPTION_CAPTEUR_IR_H
#define RECEPTION_CAPTEUR_IR_H

#include <stdint.h>
#include <stdbool.h>


typedef enum{
	CAPTEUR_HAUT_DROITE=0,
	CAPTEUR_HAUT_DROITE_45,
	CAPTEUR_DROITE,
	CAPTEUR_BAS_DROITE,
	CAPTEUR_BAS_GAUCHE,
	CAPTEUR_GAUCHE,
	CAPTEUR_HAUT_GAUCHE,
	CAPTEUR_HAUT_GAUCHE_45
}CAPTEUR_NUMEROTATION;

//les capteurs sont numerot�es dans le sens horaire en commencant par celui en haut a droite.
// Les mesures sont faites � des frequences de 800Hz soit 100Hz par capteur
// Les mesures sont faites par paires oppos�es afin de pas avoir des interferences : IR0 et IR4 puis IR1 et IR5 puis IR2 et IR6 puis IR3 et IR7


/* Fonction qui d�termine le chemin � prendre (contournement par la droite ou la gauche)
 * J'ai mis par d�faut (ie pas d'obstacle sur les cot�s) le contournement par la gauche
 */
bool check_chemin(void);


// Fonction qui determine si le capteur concern� voit un obstacle ou non
// Est appel�e dans le threads pour d�finir l'environnement
bool get_obstacle_condition(int32_t num_capteur);

//Focntion qui remet tous les bools � 0 et refresh les donn�es des capteurs
//passe les bools � 1 si obstacle en vue
// Est appel�e dans le thread pour refresh l'environnement
void valeurs_calibrees(void);

#endif
