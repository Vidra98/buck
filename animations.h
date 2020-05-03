#ifndef ANIMATIONS_H
#define ANIMATIONS_H

#include <stdint.h>
#include <stdbool.h>
#define PI					3.14159265358979f
#define SEC					1000
#define TEMPS_SON			200
#define TOGGLE				2
#define VALIDATION			4

#define NB_RGB_LED			4
#define RGB_LED2	   		0
#define RGB_LED4			1
#define RGB_LED6			2
#define RGB_LED8			3
/* Allume une des 8 leds sur le haut de buck selon la direction du son incident
 *
 */
void reset_leds(void);

void set_direction_led(float angle);

void animation_leds(uint8_t etat_parcours);

void validation_commande(void);

bool get_animations_commande_validee(void);

void animations_start(void);

#endif
