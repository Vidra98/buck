#ifndef PARCOURS_H
#define PARCOURS_H
#include <stdint.h>


//pour une vitesse lineaire de 10cm/s avec la conversion de 1 tour/s = 1000 steps/s et r = 13/(2pi)
#define VITESSE_LIN 		769

//buck a un perimetre de 16.8cm donc un quart de tour correspond à 4.2 cm donc on veut une vitesse de 2.1 cm/s (tourne pendant 2 secs)
// on fait la conversion en step
#define ROTATION_45_DEG 	81
#define ROTATION_90_DEG		162




int16_t vitesse_moteurs(int32_t capteur);

int16_t vitesse_moteur_gauche (int16_t right_speed);

void parcours_start(void);



#endif
