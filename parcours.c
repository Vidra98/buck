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



void parcours_en_infini(void)
{
	int32_t compteur_droit = right_motor_get_pos();
	int32_t compteur_gauche = left_motor_get_pos();
	static uint8_t etat_parcours = PREM_LIGNE_DROITE;
	//s�quence ligne droite - virage - ligne droite - virage que l'on r�p�te en boucle
	// le passage d'une �tape � l'autre s'effectue avec l'actualisation de la position des moteurs
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


int16_t pi_controller(int32_t objectif, int32_t pos_actuelle)
{
	float error = 0;
	float speed = 0;
	static float sum_error = 0;

	error = abs(pos_actuelle) - abs(objectif);
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




void parcours_obstacle(bool obstacle_devant, bool obstacle_droite_45, bool obstacle_gauche_45)
{
	int16_t vitesse_pi_r = 0;
	int16_t vitesse_pi_l = 0;
	static uint8_t etat_contournement = MVT_IDLE;
	static int32_t compteur_droite = 0;
	static int32_t compteur_gauche = 0;
	static bool tourner_a_gauche = false;
	static bool obstacle_cote_90 = false;

	switch(etat_contournement)
	{
		case MVT_IDLE :
		if ((!obstacle_devant) && (!obstacle_droite_45) && (!obstacle_gauche_45))
		{
			vitesse_droite = VITESSE_IDLE;
			vitesse_gauche = VITESSE_IDLE;
		}

		else
		{
			etat_contournement = CONTOURNEMENT;
			right_motor_set_pos(0);
			left_motor_set_pos(0);
		}
		break;

		case CONTOURNEMENT :
		if ((!obstacle_devant) && (!obstacle_droite_45) && (!obstacle_gauche_45))
		{
			etat_contournement = LONGEMENT;
		}
		else
		{
			if (compteur_droite == 0 && compteur_gauche == 0)
			{
				tourner_a_gauche = check_chemin();
			}
			if (tourner_a_gauche)
			{
				vitesse_droite = VITESSE_IDLE;
				vitesse_gauche = -VITESSE_IDLE;
			}

			else
			{
				vitesse_droite = -VITESSE_IDLE;
				vitesse_gauche = VITESSE_IDLE;
			}
			compteur_droite = right_motor_get_pos();
			compteur_gauche = left_motor_get_pos();
		}
		break;

		case LONGEMENT:
		if (!obstacle_devant)
		{
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
		}
		else
		{
			etat_contournement = CONTOURNEMENT;
			right_motor_set_pos(0);
			left_motor_set_pos(0);
			compteur_droite = 0;
			compteur_gauche = 0;
		}
		break;

		case RETOUR_TRAJECTOIRE:
		vitesse_pi_r = pi_controller(compteur_droite, right_motor_get_pos());
		vitesse_pi_l = pi_controller(compteur_gauche, left_motor_get_pos());
		if (tourner_a_gauche)
		{
			vitesse_droite = VITESSE_IDLE + vitesse_pi_r;
			vitesse_gauche = VITESSE_IDLE - vitesse_pi_l;
		}
		else
		{
			vitesse_droite = VITESSE_IDLE - vitesse_pi_r;
			vitesse_gauche = VITESSE_IDLE + vitesse_pi_l;
		}
		if (abs(vitesse_pi_r) < 10 || abs(vitesse_pi_l) < 10)
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
	bool obstacle_devant_droit = false;
	bool obstacle_devant_gauche = false;

	while (1)
	{
		time = chVTGetSystemTime();
		valeurs_calibrees();
		obstacle_devant = get_obstacle_condition(CAPTEUR_HAUT_DROITE);
		obstacle_devant_droit = get_obstacle_condition(CAPTEUR_HAUT_DROITE_45);
		obstacle_devant_gauche = get_obstacle_condition(CAPTEUR_HAUT_GAUCHE_45);


		parcours_obstacle(obstacle_devant, obstacle_devant_droit, obstacle_devant_gauche);
		right_motor_set_speed(vitesse_droite);
		left_motor_set_speed(vitesse_gauche);

		chThdSleepUntilWindowed(time, time + MS2ST(10)); //chgt : mise � une freq de 100Hz
		//baisser la fr�quence du thread (ie augmenter le temps de sleep) si karnel panic
	}
}

void parcours_start(void){
	chThdCreateStatic(waParcours, sizeof(waParcours), NORMALPRIO, Parcours, NULL);
}
