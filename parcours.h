#ifndef PARCOURS_H
#define PARCOURS_H
#include <stdint.h>
#include <stdbool.h>

//temps d'attente avant le parcours en infini [seconde]
#define TEMPS_IDLE					5

//pour une vitesse lineaire de 10cm/s avec la conversion de 1 tour/s = 1000 steps/s et r = 13/(2pi)
#define VITESSE_LIN 				769

// pour une vitesse d'environ 7.5 cm/s qui correspond à l'idle
#define VITESSE_IDLE 				577

//etapes pour definir la progression du contournement d'obstacles
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

//coefficient du regulateur pour l'esquive d'obstacle
#define VITESSE_LIM					400
#define KP 							5
#define KI							0.1
#define MAX_SUM_ERROR 				VITESSE_LIM/KI
#define MAX_ERROR 					VITESSE_LIM/KP
#define VITESSE_MIN_PI				10

//coefficient du regulateur pour le tracking sonore
#define VITESSE_LIM_PI_SON			900
#define KP_SON						400
#define KI_SON						5
#define MAX_SUM_ERROR_SON 			VITESSE_LIM/6
#define MAX_ERROR_SON 				VITESSE_LIM
#define KA_SON						800

#define PI							3.14159265358979f
#define COS_AVANT					1.0f
#define COS_ARRIERE					-1.0f
#define COS_MARGE					0.05f

#define ERREURS_STEPS				10
#define LIMITE_STEPS_DROITE			2310
#define LIMITE_STEPS_PETIT_VIRAGE	1613
#define LIMITE_STEPS_GRAND_VIRAGE 	3322
#define VITESSE_PETIT_VIRAGE 		373


uint8_t get_parcours_etat(void);

bool get_parcours_nouvelle_commande(void);

//initialisation du thread
void parcours_start(void);



#endif
