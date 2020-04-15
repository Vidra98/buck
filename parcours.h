#ifndef PARCOURS_H
#define PARCOURS_H
#include <stdint.h>
#include <stdbool.h>


//pour une vitesse lineaire de 10cm/s avec la conversion de 1 tour/s = 1000 steps/s et r = 13/(2pi)
#define VITESSE_LIN 				769

//buck a un perimetre de 16.8cm donc un quart de tour correspond à 4.2 cm donc on veut une vitesse de 2.1 cm/s (tourne pendant 2 secs)
// on fait la conversion en step
#define ROTATION_45_DEG 			81
#define ROTATION_90_DEG				162

//les 3 1ers defines ne devraient normalement plus servir pour la suite du projet

// pour une vitesse d'environ 7.5 cm/s qui correspond à l'idle (tant que pas d'obstacle, buck avance à une vitesse de 7.5cm/s)
#define VITESSE_IDLE 				577

//etapes pour definir la progression du contournement d'obstacles
#define MVT_IDLE 					0
#define MVT_CONTOURNEMENT_DROITE	1
#define MVT_CONTOURNEMENT_GAUCHE 	2

/*comme la vitesse du régulateur s'ajoute à celle de l'idle (à savoir 7.5 cm/s) et que l'on souhaite pas dépasser la vitesse max
de 13 cm/s on définit une vitesse limite sur le regulateur que l'on pourra jamais dépasser. ceci évite ainsi que le régulateur
s'emballe avec des valeurs d'erreur qui sont trop élevées (surtout quand on voit les valeurs des capteurs)
Tu peux essayer avec plusieurs valeurs de KP et KI pour voir ce que ça donne*/

#define VITESSE_LIM					350
#define KP 							1
#define KI							0.1
#define MAX_SUM_ERROR 				VITESSE_LIM/KI
#define MAX_ERROR 					VITESSE_LIM/KP

//On aura très probablement plus besoin de ces 2 fonctions

//int16_t vitesse_moteurs(int32_t capteur, bool obstacle_capteur);

//int16_t vitesse_moteur_gauche (int16_t right_speed);


/* Le controleur PI, qui s'active lorsqu'on detecte un obstacle devant ou sur les cotés (et qu'on sort alors de l'idle)
 * Recoit en paramètre les valeurs des capteurs de devant et du coté ainsi qu'une valeur correspondant à un obstacle en contact
 * (suite aux mesures). Sort une vitesse à appliquer sur les moteurs (qui s'ajoute à la vitesse de l'idle)
 */
int16_t pi_controller(uint32_t ecart_devant, uint32_t ecart_cote, uint32_t limite_contact);


/* La fonction qui définit nos vitesses de moteurs selon notre environnement (correspond aux booléens qui indiquent la présence
 * d'obstacle sur les cotés et devant). Dans cette fonction se trouve l'appel du PI.
 * Ne renvoit rien car les vitesses sont des variables globales (donc la fonction y a directement accès).
 */
void definir_vitesse(bool obstacle1, bool obstacle2, bool obstacle3);

//initialisation du thread
void parcours_start(void);



#endif
