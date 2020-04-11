#include "parcours.h"
#include <main.h>
#include <chprintf.h>
#include "ch.h"
#include "hal.h"
#include <motors.h>
#include "reception_capteur_IR.h"


int16_t vitesse_moteurs(int32_t capteur)
{
	int16_t speed = 0.;
		switch(capteur)
		{
			case CAPTEUR_HAUT_DROITE :
			//chprintf((BaseSequentialStream *)&SD3, "l'obstacle est devant moi\n");
			speed = -VITESSE_LIN;
			break;

			case CAPTEUR_HAUT_DROITE_45 :
			//chprintf((BaseSequentialStream *)&SD3, "l'obstacle est sur ma droite à 45 degrés \n");
			speed = -ROTATION_45_DEG;
			break;

			case CAPTEUR_DROITE :
			//chprintf((BaseSequentialStream *)&SD3, "l'obstacle est sur ma droite \n");
			speed = -ROTATION_90_DEG;
			break;

			case CAPTEUR_BAS_DROITE	:
			//chprintf((BaseSequentialStream *)&SD3, "l'obstacle est derrière moi \n");
			speed = VITESSE_LIN;
			break;

			case CAPTEUR_BAS_GAUCHE	:
			//chprintf((BaseSequentialStream *)&SD3, "l'obstacle est derrière moi \n");
			speed = VITESSE_LIN;
			break;

			case CAPTEUR_GAUCHE :
			//chprintf((BaseSequentialStream *)&SD3, "l'obstacle est sur ma gauche \n");
			speed = ROTATION_90_DEG;
			break;

			case CAPTEUR_HAUT_GAUCHE_45 :
			//chprintf((BaseSequentialStream *)&SD3, "l'obstacle est sur ma gauche à 45 degrés \n");
			speed = ROTATION_45_DEG;
			break;

			case CAPTEUR_HAUT_GAUCHE :
			//chprintf((BaseSequentialStream *)&SD3, "l'obstacle est devant moi \n");
			speed = -VITESSE_LIN;
			break;

			case PAS_DE_CAPTEUR :
			//chprintf((BaseSequentialStream *)&SD3, "pas d'obstacle à signaler \n");
			speed = 0;

		}

		return speed;
}

int16_t vitesse_moteur_gauche (int16_t right_speed)
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
}

static THD_WORKING_AREA(waParcours, 256);
static THD_FUNCTION(Parcours, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;

	int16_t vitesse_droite = 0;
	int16_t vitesse_gauche = 0;
	//bool obstacle_capteur = false;
	int32_t numero_capteur = PAS_DE_CAPTEUR;

	while (1)
	{
		time = chVTGetSystemTime();
		numero_capteur = check_environnement();
		//obstacle_capteur = get_obstacle_condition();
		vitesse_droite = vitesse_moteurs(numero_capteur);
		vitesse_gauche = vitesse_moteur_gauche(vitesse_droite);
		right_motor_set_speed(vitesse_droite);
		left_motor_set_speed(vitesse_gauche);

		chThdSleepUntilWindowed(time, time + MS2ST(2000));
		//définir une vitesse pendant 2 secondes puis l'arrêter (normalement)
	}
}

void parcours_start(void){
	chThdCreateStatic(waParcours, sizeof(waParcours), NORMALPRIO, Parcours, NULL);
}
