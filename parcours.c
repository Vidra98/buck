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
static int16_t etat_contournement = MVT_IDLE;
static uint16_t etat_parcours = PREM_LIGNE_DROITE;
static bool contournement_en_cours = false;


void parcours_en_infini(void)
{
	int32_t compteur_droit = 0;
	int32_t compteur_gauche = 0;
	compteur_droit = right_motor_get_pos();
	compteur_gauche = left_motor_get_pos();

	//s�quence ligne droite - virage - ligne droite - virage que l'on r�p�te en boucle
	// le passage d'une �tape � l'autre s'effectue avec l'actualisation de la position des moteurs
	switch(etat_parcours)
	{
		case PREM_LIGNE_DROITE :
		if ((abs(compteur_droit - LIMITE_STEPS_DROITE) > ERREURS_STEPS) &&
				(abs(compteur_gauche - LIMITE_STEPS_DROITE) > ERREURS_STEPS))
		{
			etat_parcours = PREM_VIRAGE;
			right_motor_set_pos(0);
			left_motor_set_pos(0);
			vitesse_droite = 0;
			vitesse_gauche = 0;
			break;
		}
		vitesse_droite = VITESSE_LIN;
		vitesse_gauche = VITESSE_LIN;
		break;


	case PREM_VIRAGE :
	if ((abs(compteur_droit - LIMITE_STEPS_PETIT_VIRAGE) > ERREURS_STEPS) &&
			(abs(compteur_gauche - LIMITE_STEPS_GRAND_VIRAGE) > ERREURS_STEPS))
	{
		etat_parcours = SEC_LIGNE_DROITE;
		right_motor_set_pos(0);
		left_motor_set_pos(0);
		vitesse_droite = 0;
		vitesse_gauche = 0;
		break;
	}
	vitesse_droite = VITESSE_PETIT_VIRAGE;
	vitesse_gauche = VITESSE_LIN;
	break;

	case SEC_LIGNE_DROITE :
		if ((abs(compteur_droit - LIMITE_STEPS_DROITE) > ERREURS_STEPS) &&
				(abs(compteur_gauche - LIMITE_STEPS_DROITE) > ERREURS_STEPS))
		{
			etat_parcours = SEC_VIRAGE;
			right_motor_set_pos(0);
			left_motor_set_pos(0);
			vitesse_droite = 0;
			vitesse_gauche = 0;
			break;
		}
		vitesse_droite = VITESSE_LIN;
		vitesse_gauche = VITESSE_LIN;
		break;

	case SEC_VIRAGE :
	if ((abs(compteur_gauche - LIMITE_STEPS_PETIT_VIRAGE) > ERREURS_STEPS) &&
			(abs(compteur_droit - LIMITE_STEPS_GRAND_VIRAGE) > ERREURS_STEPS))
	{
		etat_parcours = PREM_LIGNE_DROITE;
		right_motor_set_pos(0);
		left_motor_set_pos(0);
		vitesse_droite = 0;
		vitesse_gauche = 0;
		break;
	}
	vitesse_droite = VITESSE_LIN;
	vitesse_gauche = VITESSE_PETIT_VIRAGE;
	break;
	}
}


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

}




void definir_vitesse(bool obstacle1, bool obstacle2, bool obstacle3)
{
	int16_t vitesse_pi = 0;
	uint32_t valeur_capteur_devant = 0;
	uint32_t valeur_capteur_cote = 0;
	static int16_t compteur = 0;

   //1er cas : pas d'obstacle on est dans le cas de l'idle (on avance � 7.5 cm/s)
	if ((!obstacle1) && (!obstacle2) && (!obstacle3) && (!contournement_en_cours))
	{
		vitesse_droite = VITESSE_IDLE;
		vitesse_gauche = VITESSE_IDLE;
		etat_contournement = MVT_IDLE;
	}

	//2eme cas : un obstacle mais on l'a contourn� et on souhaite retourner dans la trajectoire initiale
	// il y a plus rien devant ou sur les c�t�s � 45 deg
	else if ((!obstacle1) && (!obstacle2) && (!obstacle3) && (contournement_en_cours))
	{
		switch(etat_contournement)
		{
			case MVT_CONTOURNEMENT_GAUCHE :
			vitesse_droite = VITESSE_LIN - VITESSE_LIM; //env 7.5cm/s
			vitesse_gauche = VITESSE_LIN + VITESSE_LIM; //env 12 cm/s
			break;

			case MVT_CONTOURNEMENT_DROITE :
			vitesse_droite = VITESSE_LIN + VITESSE_LIM;
			vitesse_gauche = VITESSE_LIN - VITESSE_LIM;
			break;
		}

	// ce compteur indique combien de temps on reste en situation de retour de trajectoire
	// comme le thread est � 100 Hz, une incr�mentation correspond � id�alement 10 ms
	// on souhaite faire ce retour de trajectoire � une demi seconde (fix� arbitrairement et peut �tre modifi�)
		compteur++;
		if (compteur > LIM_CONTOURNEMENT)
		{
			etat_contournement = MVT_IDLE;
			compteur = 0;
			contournement_en_cours = false;
		}
	}

	//3eme cas : il y a un obstacle en approche, on active la r�gulation pour s'en �loigner
	else
	{
		contournement_en_cours = true;
		etat_contournement = check_chemin();
		switch(etat_contournement)
		{

			case MVT_CONTOURNEMENT_GAUCHE :
			valeur_capteur_devant = get_valeur_capteur(CAPTEUR_HAUT_DROITE);
			valeur_capteur_cote = get_valeur_capteur(CAPTEUR_HAUT_DROITE_45);
			vitesse_pi = pi_controller(valeur_capteur_devant, valeur_capteur_cote, OBSTACLE_EN_CONTACT);
			vitesse_droite = VITESSE_LIN + vitesse_pi;
			vitesse_gauche = VITESSE_LIN - 2*vitesse_pi;
			break;

			case MVT_CONTOURNEMENT_DROITE :
			valeur_capteur_devant = get_valeur_capteur(CAPTEUR_HAUT_GAUCHE);
			valeur_capteur_cote = get_valeur_capteur(CAPTEUR_HAUT_GAUCHE_45);
			vitesse_pi = pi_controller(valeur_capteur_devant, valeur_capteur_cote, OBSTACLE_EN_CONTACT);
			vitesse_droite = VITESSE_LIN - 2*vitesse_pi;
			vitesse_gauche = VITESSE_LIN + vitesse_pi;
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

		/* Enregistrement des valeurs issues des 8 capteurs (m�me si on travaillera qu'avec les 4 de devant)
		 * Les 3 bool�ens nous permettra de sortir de l'idle (car obstacle dans les alentours)
		 * J'estime que pas besoin de v�rifier les capteurs sur les c�t�s
		 */
		valeurs_calibrees();
		obstacle_devant = get_obstacle_condition(CAPTEUR_HAUT_DROITE);
		obstacle_cote_droit = get_obstacle_condition(CAPTEUR_HAUT_DROITE_45);
		obstacle_cote_gauche = get_obstacle_condition(CAPTEUR_HAUT_GAUCHE_45);


		definir_vitesse(obstacle_devant, obstacle_cote_droit, obstacle_cote_gauche);
		right_motor_set_speed(vitesse_droite);
		left_motor_set_speed(vitesse_gauche);

		chThdSleepUntilWindowed(time, time + MS2ST(10)); //chgt : mise � une freq de 100Hz
		//baisser la fr�quence du thread (ie augmenter le temps de sleep) si karnel panic
	}
}

void parcours_start(void){
	chThdCreateStatic(waParcours, sizeof(waParcours), NORMALPRIO, Parcours, NULL);
}
