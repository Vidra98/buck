#ifndef PARCOURS_H
#define PARCOURS_H
#include <stdint.h>
#include <stdbool.h>


//pour une vitesse lineaire de 10cm/s avec la conversion de 1 tour/s = 1000 steps/s et r = 13/(2pi)
#define VITESSE_LIN 				769
//buck a un perimetre de 16.8cm donc un quart de tour correspond � 4.2 cm donc on veut une vitesse de 2.1 cm/s (tourne pendant 2 secs)
// on fait la conversion en step
//#define ROTATION_45_DEG 			81
//#define ROTATION_90_DEG			162

//les 3 1ers defines ne devraient normalement plus servir pour la suite du projet

// pour une vitesse d'environ 7.5 cm/s qui correspond � l'idle (tant que pas d'obstacle, buck avance � une vitesse de 7.5cm/s)
#define VITESSE_IDLE 				577

//etapes pour definir la progression du contournement d'obstacles
#define MVT_IDLE 					0
#define MVT_CONTOURNEMENT_DROITE	1
#define MVT_CONTOURNEMENT_GAUCHE 	2
#define LIM_CONTOURNEMENT 			50

/*comme la vitesse du r�gulateur s'ajoute � celle de l'idle (� savoir 7.5 cm/s) et que l'on souhaite pas d�passer la vitesse max
de 13 cm/s on d�finit une vitesse limite sur le regulateur que l'on pourra jamais d�passer. ceci �vite ainsi que le r�gulateur
s'emballe avec des valeurs d'erreur qui sont trop �lev�es (surtout quand on voit les valeurs des capteurs)
Tu peux essayer avec plusieurs valeurs de KP et KI pour voir ce que �a donne*/

#define VITESSE_LIM					230
#define KP 							5
#define KI							0.1
#define MAX_SUM_ERROR 				VITESSE_LIM/KI
#define MAX_ERROR 					VITESSE_LIM/KP

#define PREM_LIGNE_DROITE			1
#define PREM_VIRAGE 				2
#define SEC_LIGNE_DROITE 			3
#define SEC_VIRAGE 					4
#define ERREURS_STEPS				10
#define LIMITE_STEPS_DROITE			2310
#define LIMITE_STEPS_PETIT_VIRAGE	3796
#define LIMITE_STEPS_GRAND_VIRAGE 	6360
#define VITESSE_PETIT_VIRAGE 		459


//On aura tr�s probablement plus besoin de ces 2 fonctions

//int16_t vitesse_moteurs(int32_t capteur, bool obstacle_capteur);

//int16_t vitesse_moteur_gauche (int16_t right_speed);


/* D�finit un parcours de buck selon le symb�le infini (un 8 couch�)
 * Parcours compos� de 2 lignes droites de 30 cm et de 2 virages d'un rayon de 21cm
 * (15.7cm pour la roue int�rieure et 26.3cm pour la roue ext�rieure)
 *
 */
void parcours_en_infini(void);

/* Le controleur PI, qui s'active lorsqu'on detecte un obstacle devant ou sur les cot�s (et qu'on sort alors de l'idle)
 * Recoit en param�tre les valeurs des capteurs de devant et du cot� ainsi qu'une valeur correspondant � un obstacle en contact
 * (suite aux mesures). Sort une vitesse � appliquer sur les moteurs (qui s'ajoute � la vitesse de l'idle)
 */
int16_t pi_controller(uint32_t ecart_devant, uint32_t ecart_cote, uint32_t limite_contact);


/* La fonction qui d�finit nos vitesses de moteurs selon notre environnement (correspond aux bool�ens qui indiquent la pr�sence
 * d'obstacle sur les cot�s et devant). Dans cette fonction se trouve l'appel du PI.
 * Ne renvoit rien car les vitesses sont des variables globales (donc la fonction y a directement acc�s).
 */
void definir_vitesse(bool obstacle1, bool obstacle2, bool obstacle3);

//initialisation du thread
void parcours_start(void);



#endif
