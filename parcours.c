#include "parcours.h"
#include <main.h>
#include <chprintf.h>
#include "ch.h"
#include "hal.h"
#include <motors.h>
#include "reception_capteur_IR.h"
#include <sensors/proximity.h>
#include <math.h>


static int16_t vitesse_droite;
static int16_t vitesse_gauche;
//static int16_t etat_contournement = MVT_IDLE;
static uint16_t etat_parcours = PREM_LIGNE_DROITE;



void parcours_en_infini(void)
{
	int32_t compteur_droit = 0;
	int32_t compteur_gauche = 0;
	compteur_droit = right_motor_get_pos();
	compteur_gauche = left_motor_get_pos();

	//séquence ligne droite - virage - ligne droite - virage que l'on répète en boucle
	// le passage d'une étape à l'autre s'effectue avec l'actualisation de la position des moteurs
	switch(etat_parcours)
	{
		case PREM_LIGNE_DROITE :
		if ((abs(compteur_droit - LIMITE_STEPS_DROITE) < ERREURS_STEPS) ||
				(abs(compteur_gauche - LIMITE_STEPS_DROITE) < ERREURS_STEPS))
		{
			etat_parcours = PREM_VIRAGE;
			right_motor_set_pos(0);
			left_motor_set_pos(0);
			break;
		}
		vitesse_droite = VITESSE_LIN;
		vitesse_gauche = VITESSE_LIN;
		break;


	case PREM_VIRAGE :
	if ((abs(compteur_droit - LIMITE_STEPS_PETIT_VIRAGE) < ERREURS_STEPS) ||
			(abs(compteur_gauche - LIMITE_STEPS_GRAND_VIRAGE) < ERREURS_STEPS))
	{
		etat_parcours = SEC_LIGNE_DROITE;
		right_motor_set_pos(0);
		left_motor_set_pos(0);
		break;
	}
	vitesse_droite = VITESSE_PETIT_VIRAGE;
	vitesse_gauche = VITESSE_LIN;
	break;

	case SEC_LIGNE_DROITE :
		if ((abs(compteur_droit - LIMITE_STEPS_DROITE) < ERREURS_STEPS) ||
				(abs(compteur_gauche - LIMITE_STEPS_DROITE) < ERREURS_STEPS))
		{
			etat_parcours = SEC_VIRAGE;
			right_motor_set_pos(0);
			left_motor_set_pos(0);
			break;
		}
		vitesse_droite = VITESSE_LIN;
		vitesse_gauche = VITESSE_LIN;
		break;

	case SEC_VIRAGE :
	if ((abs(compteur_gauche - LIMITE_STEPS_PETIT_VIRAGE) < ERREURS_STEPS) ||
			(abs(compteur_droit - LIMITE_STEPS_GRAND_VIRAGE) < ERREURS_STEPS))
	{
		etat_parcours = PREM_LIGNE_DROITE;
		right_motor_set_pos(0);
		left_motor_set_pos(0);
		break;
	}
	vitesse_droite = VITESSE_LIN;
	vitesse_gauche = VITESSE_PETIT_VIRAGE;
	break;
	}

}

/*
int16_t pi_controller(uint32_t ecart_devant, uint32_t ecart_cote, uint32_t limite_contact)
{
	float error = 0;
	float speed = 0;
	static float sum_error = 0;

	error = 2*ecart_devant + ecart_cote - limite_contact;
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

}*/




void definir_vitesse(bool obstacle1, bool obstacle2, bool obstacle3)
{
	//int16_t vitesse_pi = 0;
	//uint32_t valeur_capteur_devant = 0;
	//uint32_t valeur_capteur_cote = 0;
	static int16_t etat_contournement = MVT_IDLE;
	static int32_t compteur_droite = 0;
	static int32_t compteur_gauche = 0;
	static bool tourner_a_gauche = false;
	static bool obstacle_cote_90 = false;

	switch(etat_contournement)
	{
		case MVT_IDLE :
		if ((!obstacle1) && (!obstacle2) && (!obstacle3))
		{
			vitesse_droite = VITESSE_IDLE;
			vitesse_gauche = VITESSE_IDLE;
		}

		else
		{
			etat_contournement = CONTOURNEMENT;
		}
		break;

		case CONTOURNEMENT :
		if ((!obstacle1) && (!obstacle2) && (!obstacle3))
		{
			etat_contournement = LONGEMENT;
		}
		else
		{
			tourner_a_gauche = check_chemin();
			if (tourner_a_gauche)
			{
				//valeur_capteur_devant = get_valeur_capteur(CAPTEUR_HAUT_DROITE);
				//valeur_capteur_cote = get_valeur_capteur(CAPTEUR_HAUT_DROITE_45);
				//vitesse_pi = pi_controller(valeur_capteur_devant, valeur_capteur_cote, OBSTACLE_EN_CONTACT);
				vitesse_droite = VITESSE_IDLE;
				vitesse_gauche = -VITESSE_IDLE;
			}

			else
			{
				//valeur_capteur_devant = get_valeur_capteur(CAPTEUR_HAUT_GAUCHE);
				//valeur_capteur_cote = get_valeur_capteur(CAPTEUR_HAUT_GAUCHE_45);
				//vitesse_pi = pi_controller(valeur_capteur_devant, valeur_capteur_cote, OBSTACLE_EN_CONTACT);
				vitesse_droite = -VITESSE_IDLE;
				vitesse_gauche = VITESSE_IDLE;
			}
			compteur_droite = right_motor_get_pos();
			compteur_gauche = left_motor_get_pos();

		}
		break;

		case LONGEMENT:
		if (tourner_a_gauche)
		{
			obstacle_cote_90 = get_obstacle_condition(CAPTEUR_DROITE);
		}
		else
		{
			obstacle_cote_90 = get_obstacle_condition(CAPTEUR_GAUCHE);
		}
		if (obstacle_cote_90)
		{
			vitesse_droite = VITESSE_IDLE;
			vitesse_gauche = VITESSE_IDLE;
		}
		else
		{
			etat_contournement = RETOUR_TRAJECTOIRE;
			right_motor_set_pos(0);
			left_motor_set_pos(0);

		}
		break;

		case RETOUR_TRAJECTOIRE:
		if (tourner_a_gauche)
		{
			vitesse_droite = -VITESSE_IDLE;
			vitesse_gauche = VITESSE_IDLE;
		}
		else
		{
			vitesse_droite = VITESSE_IDLE;
			vitesse_gauche = -VITESSE_IDLE;
		}
		if (abs(right_motor_get_pos() - compteur_droite) < ERREURS_STEPS ||
				abs(left_motor_get_pos() - compteur_gauche) < ERREURS_STEPS)
		{
			tourner_a_gauche = false;
			obstacle_cote_90 = false;
			etat_contournement = MVT_IDLE;
			compteur_droite = 0;
			compteur_gauche = 0;
		}
		break;
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
