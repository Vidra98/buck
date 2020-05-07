#ifndef ANIMATIONS_H
#define ANIMATIONS_H

#include <stdint.h>
#include <stdbool.h>
#define PI					3.14159265358979f
#define SEC					1000
#define TOGGLE				2
#define VALIDATION			4

#define ON					1
#define OFF					0


void set_nouvelle_commande(void);

// Allume une des 8 leds sur le haut de buck selon la direction du son incident
void set_tracking_leds(float angle);

bool get_animations_commande_validee(void);
//demarage du thread d'animation
void animations_start(void);

#endif
