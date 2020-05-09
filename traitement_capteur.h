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
	CAPTEUR_HAUT_GAUCHE_45,
	CAPTEUR_HAUT_GAUCHE
}CAPTEUR_NUMEROTATION;


/* Ordonne la direction à prendre pour contourner l'obstacle
 * La direction fixée par défaut est la gauche
 */
bool check_chemin(void);


/* Indique si le capteur concerné voit un obstacle devant lui ou non
 *
 */
bool get_obstacle_condition(CAPTEUR_NUMEROTATION numero_capteur);

/*
 * Reactualise l'environnement de notre robot
 */
void actualisation_capteurs(void);

#endif
