#include "parcours.h"
#include <main.h>
#include <chprintf.h>
#include "ch.h"
#include "hal.h"
#include <motors.h>
#include "reception_capteur_IR.h"
#include <sensors/proximity.h>


static int16_t vitesse_droite;
static int16_t vitesse_gauche;
static int16_t etat_contournement = MVT_IDLE;

// les 2 1eres fonctions ne devraient plus servir, je les laisse en comm au cas où le contournement ne marche pas
// ça fera au moins un truc qui marche :)

/*int16_t vitesse_moteurs(int32_t capteur, bool obstacle_capteur)
{
	int16_t speed = 0;
	if (obstacle_capteur)
	{
		chprintf((BaseSequentialStream *)&SD3, "Il ya un obstacle \n");
		switch(capteur)
		{
			case CAPTEUR_HAUT_DROITE :
			chprintf((BaseSequentialStream *)&SD3, "l'obstacle est devant moi\n");
			speed = -VITESSE_LIN;
			break;

			case CAPTEUR_HAUT_DROITE_45 :
			chprintf((BaseSequentialStream *)&SD3, "l'obstacle est sur ma droite à 45 degrés \n");
			speed = -ROTATION_45_DEG;
			break;

			case CAPTEUR_DROITE :
			chprintf((BaseSequentialStream *)&SD3, "l'obstacle est sur ma droite \n");
			speed = -ROTATION_90_DEG;
			break;

			case CAPTEUR_BAS_DROITE	:
			chprintf((BaseSequentialStream *)&SD3, "l'obstacle est derrière moi \n");
			speed = VITESSE_LIN;
			break;

			case CAPTEUR_BAS_GAUCHE	:
			chprintf((BaseSequentialStream *)&SD3, "l'obstacle est derrière moi \n");
			speed = VITESSE_LIN;
			break;

			case CAPTEUR_GAUCHE :
			chprintf((BaseSequentialStream *)&SD3, "l'obstacle est sur ma gauche \n");
			speed = ROTATION_90_DEG;
			break;

			case CAPTEUR_HAUT_GAUCHE_45 :
			chprintf((BaseSequentialStream *)&SD3, "l'obstacle est sur ma gauche à 45 degrés \n");
			speed = ROTATION_45_DEG;
			break;

			case CAPTEUR_HAUT_GAUCHE :
			chprintf((BaseSequentialStream *)&SD3, "l'obstacle est devant moi \n");
			speed = -VITESSE_LIN;
			break;

		}
	}

	return speed;
}*/

/*int16_t vitesse_moteur_gauche (int16_t right_speed)
{
	int16_t left_speed = 0;
	if (right_speed != 0)
	{
	switch (right_speed)
	{
		case VITESSE_LIN :
		left_speed = VITESSE_LIN;
		break;

		case (-VITESSE_LIN) :
		left_speed = -VITESSE_LIN;
		break;

		case ROTATION_45_DEG :
		left_speed = -ROTATION_45_DEG;
		break;

		case (-ROTATION_45_DEG) :
		left_speed = ROTATION_45_DEG;
		break;

		case ROTATION_90_DEG :
		left_speed = -ROTATION_90_DEG;
		break;

		case (-ROTATION_90_DEG) :
		left_speed = ROTATION_90_DEG;
		break;
	}
	}
	return left_speed;
}*/


int16_t pi_controller(uint32_t ecart_devant, uint32_t ecart_cote, uint32_t limite_contact)
{
	float error = 0;
	float speed = 0;
	static float sum_error = 0;

	error = ecart_devant + ecart_cote - limite_contact;
	//pour eviter des emballements
	if (error > MAX_ERROR)
	{
		error = MAX_ERROR;
	}

	else if (error < -MAX_ERROR)
	{
		error = -MAX_ERROR;
	}


	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}


	speed = KP * error + KI * sum_error;

	return (int16_t)speed;

}




void definir_vitesse(bool obstacle1, bool obstacle2, bool obstacle3)
{
	int16_t vitesse_pi = 0;
	uint32_t valeur_capteur_devant = 0;
	uint32_t valeur_capteur_cote = 0;
	if ((!obstacle1) && (!obstacle2) && (!obstacle3))
	{
		vitesse_droite = VITESSE_IDLE;
		vitesse_gauche = VITESSE_IDLE;
		etat_contournement = MVT_IDLE;
	}

	else
	{

		etat_contournement = check_chemin();
		switch(etat_contournement)
		{

			case MVT_CONTOURNEMENT_GAUCHE :
			valeur_capteur_devant = get_valeur_capteur(CAPTEUR_HAUT_DROITE);
			valeur_capteur_cote = get_valeur_capteur(CAPTEUR_HAUT_DROITE_45);
			vitesse_pi = pi_controller(valeur_capteur_devant, valeur_capteur_cote, OBSTACLE_EN_CONTACT);
			vitesse_droite = VITESSE_IDLE + vitesse_pi;
			vitesse_gauche = VITESSE_IDLE - vitesse_pi;
			break;

			case MVT_CONTOURNEMENT_DROITE :
			valeur_capteur_devant = get_valeur_capteur(CAPTEUR_HAUT_GAUCHE);
			valeur_capteur_cote = get_valeur_capteur(CAPTEUR_HAUT_GAUCHE_45);
			vitesse_pi = pi_controller(valeur_capteur_devant, valeur_capteur_cote, OBSTACLE_EN_CONTACT);
			vitesse_droite = VITESSE_IDLE - vitesse_pi;
			vitesse_gauche = VITESSE_IDLE + vitesse_pi;
			break;
		}
	}
}



static THD_WORKING_AREA(waParcours, 256);
static THD_FUNCTION(Parcours, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;
	bool obstacle_devant = false;
	bool obstacle_cote_droit = false;
	bool obstacle_cote_gauche = false;

	while (1)
	{
		time = chVTGetSystemTime();
		//chprintf((BaseSequentialStream *)&SD3, "je rentre dans la fonction capteur \n");

		/* Enregistrement des valeurs issues des 8 capteurs (même si on travaillera qu'avec les 4 de devant)
		 * Les 3 booléens nous permettra de sortir de l'idle (car obstacle dans les alentours)
		 * J'estime que pas besoin de vérifier les capteurs sur les côtés
		 */
		valeurs_calibrees();
		obstacle_devant = get_obstacle_condition(CAPTEUR_HAUT_DROITE);
		obstacle_cote_droit = get_obstacle_condition(CAPTEUR_HAUT_DROITE_45);
		obstacle_cote_gauche = get_obstacle_condition(CAPTEUR_HAUT_GAUCHE_45);


		definir_vitesse(obstacle_devant, obstacle_cote_droit, obstacle_cote_gauche);
		right_motor_set_speed(vitesse_droite);
		left_motor_set_speed(vitesse_gauche);

		chThdSleepUntilWindowed(time, time + MS2ST(10)); //chgt : mise à une freq de 100Hz
		//baisser la fréquence du thread (ie augmenter le temps de sleep) si karnel panic
	}
}

void parcours_start(void){
	chThdCreateStatic(waParcours, sizeof(waParcours), NORMALPRIO, Parcours, NULL);
}
