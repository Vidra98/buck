#ifndef PARCOURS_H
#define PARCOURS_H

#include <stdint.h>

//etats pour definir la progression du contournement d'obstacles
typedef enum{
	MVT_IDLE = 0,
	CONTOURNEMENT,
	LONGEMENT,
	RETOUR_TRAJECTOIRE,
	PREM_LIGNE_DROITE,
	PREM_VIRAGE,
	SEC_LIGNE_DROITE,
	SEC_VIRAGE,
	TRACKING,
	PARCOURS_INFINI
} BUCK_STATE;

//retourne l'état du robot (BUCK_STATE)
uint8_t get_parcours_etat(void);

//initialisation du thread des déplacement
void parcours_start(void);



#endif
