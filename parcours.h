#ifndef PARCOURS_H
#define PARCOURS_H
#include <stdint.h>
#include <stdbool.h>


#define TEMPS_IDLE					5

//pour une vitesse lineaire de 10cm/s avec la conversion de 1 tour/s = 1000 steps/s et r = 13/(2pi)
#define VITESSE_LIN 				769

// pour une vitesse d'environ 7.5 cm/s qui correspond à l'idle
//(tant que pas d'obstacle, buck avance à une vitesse de 7.5cm/s)
#define VITESSE_IDLE 				577

//etapes pour definir la progression du contournement d'obstacles
#define MVT_IDLE 					0
#define CONTOURNEMENT				1
#define LONGEMENT					2
#define RETOUR_TRAJECTOIRE		 	3
#define PREM_LIGNE_DROITE			4
#define PREM_VIRAGE 				5
#define SEC_LIGNE_DROITE 			6
#define SEC_VIRAGE 					7

/*comme la vitesse du régulateur s'ajoute à celle de l'idle (à savoir 7.5 cm/s) et que l'on souhaite pas dépasser la vitesse max
de 13 cm/s on définit une vitesse limite sur le regulateur que l'on pourra jamais dépasser. ceci évite ainsi que le régulateur
s'emballe avec des valeurs d'erreur qui sont trop élevées (surtout quand on voit les valeurs des capteurs)
*/

#define VITESSE_LIM					400
#define KP 							5
#define KI							0.1
#define MAX_SUM_ERROR 				VITESSE_LIM/KI
#define MAX_ERROR 					VITESSE_LIM/KP
#define VITESSE_MIN_PI				10

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



bool commande_recu(float freq);

uint8_t get_parcours_etat(void);

bool get_parcours_nouvelle_commande(void);

float get_parcours_angle(void);
/* Définit un parcours de buck selon le symbôle infini (un 8 couché)
 * Parcours composé de 2 lignes droites de 30 cm et de 2 virages d'un rayon de 21cm
 * (15.7cm pour la roue intérieure et 26.3cm pour la roue extérieure)
 *
 */
void parcours_en_infini(void);

/* Fonction qui déclenche un controlleur PI pour le retour sur trajectoire (une fois l'obstacle contourné)
 * reçoit en paramètre la position des moteurs enregistrées lors du contournement
 * et la position actuelle lors du retour en trajectoire
 */
int16_t pi_controller(int32_t objectif, int32_t pos_actuelle);


/* La fonction qui définit nos vitesses de moteurs selon notre environnement (correspond aux booléens qui indiquent la présence
 * d'obstacle sur les cotés et devant). Dans cette fonction se trouve l'appel du PI.
 * Ne renvoit rien car les vitesses sont des variables globales (donc la fonction y a directement accès).
 */
void parcours_obstacle(bool obstacle1, bool obstacle2, bool obstacle3);

//initialisation du thread
void parcours_start(void);



#endif
