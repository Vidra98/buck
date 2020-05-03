#include "parcours.h"
#include <main.h>
#include <chprintf.h>
#include "ch.h"
#include "hal.h"
#include <motors.h>
#include "reception_capteur_IR.h"
#include <sensors/proximity.h>
#include <math.h>
#include <traitement_son.h>
#include <leds.h>
#include "animations.h"


static int16_t vitesse_droite;
static int16_t vitesse_gauche;
static uint8_t etat_contournement = MVT_IDLE;
static bool nouvelle_commande = false;
static bool aller_en_avant = false;
static bool aller_en_arriere = false;
static float angle;

/*Oriente le robot dans la direction du son
 *a revoir quelque modif
 */

//modifs de guigui : j'ai mis l'angle en variable globale car le fichier animations en a besoin pour les leds
// j'ai retiré sound_index pour le mettre dans une autre fonction (qui nous permet de dire si on a reçu une commande)
// du coup j'ai remplacé les conditions "si la freq est dans l'intervalle" par des booléens qui sont définies dans la même fonction
// vas checker la fonction commande_recu
void reponse_sonore(void){


	int16_t vitesse_d,vitesse_g;
	//float angle, sound_index,c_angle,s_angle;
	float c_angle, s_angle;
	static int16_t sum_error;
	angle = get_angle();
	//sound_index = get_freq();
	//chprintf((BaseSequentialStream *) &SD3, " parcours :  angle %f freq %f    ", angle, sound_index*AUDIO_RESOLUTION );
	c_angle=cosf(angle);
	s_angle=sinf(angle);
	// dirige le robot vers la source de son pour les fréquences [260,400]Hz
	//if ((sound_index>AVANT_FREQ_MIN)&&(sound_index<AVANT_FREQ_MAX))
	if(aller_en_avant)
	{
		aller_en_arriere = false;
// avant d'avancer ou de s'éloigner du son, buck devrait bipper et allumer ses body leds
//routine appelée dans le thread d'animations
//ça permet de dire qu'il a compris la commande
		if(nouvelle_commande && get_animation_commande_validee())
		{
			nouvelle_commande = false;
		}
		else{
		nouvelle_commande = false;
		sum_error += (COS_AVANT-c_angle);
		if (abs(sum_error)> MAX_SUM_ERROR_SON) sum_error=MAX_SUM_ERROR_SON;
		vitesse_d=(COS_AVANT-c_angle)*KP_SON+(sum_error)*KI_SON+KA_SON*c_angle;
		vitesse_g=-((COS_AVANT-c_angle)*KP_SON+(sum_error)*KI_SON)+KA_SON*c_angle;
	    if (s_angle > NUL){
			vitesse_droite=vitesse_d;
			vitesse_gauche=vitesse_g;
		}
		else {
			vitesse_droite=vitesse_g;
			vitesse_gauche=vitesse_d;
		}
	    set_direction_led(angle);
	    set_front_led(1);
		}
	}
	//dirige le robot à l'inverse de la source de son pour les frequence [400,555]Hz
	//else if ((sound_index>ARRIERE_FREQ_MIN)&&(sound_index<ARRIERE_FREQ_MAX))
	else if (aller_en_arriere && get_animation_commande_validee())
	{
		aller_en_avant = false;
		if (nouvelle_commande)
		{
			nouvelle_commande = false;
		}
		else {
		nouvelle_commande = false;
		sum_error += (COS_ARRIERE-c_angle);
		if (abs(sum_error)> MAX_SUM_ERROR_SON) sum_error=MAX_SUM_ERROR_SON;
		vitesse_d=(COS_ARRIERE-c_angle)*KP_SON+(sum_error)*KI_SON-KA_SON*c_angle;
		vitesse_g=-((COS_ARRIERE-c_angle)*KP_SON+(sum_error)*KI_SON)-KA_SON*c_angle;
		if (s_angle > NUL){
			vitesse_droite=vitesse_d;
			vitesse_gauche=vitesse_g;
		}
		else {
			vitesse_droite=vitesse_g;
			vitesse_gauche=vitesse_d;
		}
		set_direction_led(angle);
		set_front_led(1);
		}
	}
	//pas de mouvement sinon
	else {
		nouvelle_commande = false;
		vitesse_droite=0;
		vitesse_gauche=0;
	}
}

void parcours_en_infini(void)
{
	int32_t compteur_droit = right_motor_get_pos();
	int32_t compteur_gauche = left_motor_get_pos();
	static uint8_t etat_parcours = PREM_LIGNE_DROITE;
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



//les fonctions d'animations sont animation_leds (clignote des leds de couleur différente selon le parcours)
// set_direction_leds( indique avec les leds rouges d'où provient le son)
// et validation_commande (bippe pour indiquer qu'il a reçu une nouvelle commande et qu'il l'a comprise)
// les 2 dernières fonctions sont appelées dans reponse_sonore
void parcours_obstacle(bool obstacle_devant, bool obstacle_droite_45, bool obstacle_gauche_45)
{
	int16_t vitesse_pi_r = 0;
	int16_t vitesse_pi_l = 0;
	static int32_t compteur_droite = 0;
	static int32_t compteur_gauche = 0;
	static bool tourner_a_gauche = false;
	static bool obstacle_cote_90 = false;

	switch(etat_contournement)
	{
		//si pas d'obstacle notre epuck suit ou s'éloigne du son, sinon il l'évite
		case MVT_IDLE :
		if ((!obstacle_devant) && (!obstacle_droite_45) && (!obstacle_gauche_45))
		{
			reponse_sonore();
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
		if (abs(vitesse_pi_r) < 25 || abs(vitesse_pi_l) < 25)
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

//j'ai un peu changé ta fonction reponse sonore et j'ai mis les conditions ici
//ça me permettait d'interpréter les receptions de commandes selon les fréquences
// et donc de pouvoir aussi lancer l'idle au cas où
bool commande_recu(float freq)
{
	if ((freq>AVANT_FREQ_MIN)&&(freq<AVANT_FREQ_MAX))
	{
		if (!aller_en_avant)
		{
			aller_en_avant = true;
			nouvelle_commande = true;
		}
		return true;
	}

	else if ((freq>ARRIERE_FREQ_MIN)&&(freq<ARRIERE_FREQ_MAX))
	{
		if(!aller_en_arriere)
		{
			aller_en_arriere = true;
			nouvelle_commande = true;
		}
		return true;
	}

	else
	{
		aller_en_avant = false;
		aller_en_arriere = false;
		return false;
	}
}

bool lancement_idle(systime_t temps, float indice_freq, bool idle)
{
	static systime_t delta_t = 0;
	systime_t temps_prime = chVTGetSystemTime();
	//buck ne bouge pas quand il reçoit pas de commande
	if (!commande_recu(indice_freq))
	{
		if (!idle)
		{
			delta_t = temps_prime - temps;
			if (ST2S(delta_t) >= TEMPS_IDLE)
			{
				delta_t = 0;
				return true;
			}
			else return false;
		}
		else return true;
	}
	else return false;
}

//focntions qui sont appelées par animations pour faire les différentes animations
uint8_t get_parcours_etat(void)
{
	return etat_contournement;
}

bool get_parcours_nouvelle_commande(void)
{
	return nouvelle_commande;
}

float get_parcours_angle(void)
{
	return angle;
}


// Thread avec les routines controlant les moteurs
static THD_WORKING_AREA(waParcours, 256);
static THD_FUNCTION(Parcours, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;
	bool obstacle_devant = false;
	bool obstacle_devant_droit = false;
	bool obstacle_devant_gauche = false;
	static bool idle_lance = false;
	float freq_indice = 0.;
	while (1)
	{
		time = chVTGetSystemTime();
		freq_indice = get_freq();
		valeurs_calibrees();
		obstacle_devant = get_obstacle_condition(CAPTEUR_HAUT_DROITE);
		obstacle_devant_droit = get_obstacle_condition(CAPTEUR_HAUT_DROITE_45);
		obstacle_devant_gauche = get_obstacle_condition(CAPTEUR_HAUT_GAUCHE_45);

//implémentation de l'idle si les conditions sont respectées (attente de 5 secondes et pas de commande reçue)
		idle_lance = lancement_idle(time, freq_indice, idle_lance);
		if (idle_lance)
		{
			parcours_en_infini();
		}
		else
		{
		//les routines d'animations sont appelées dans cette fonction
			parcours_obstacle(obstacle_devant, obstacle_devant_droit, obstacle_devant_gauche);
		}
		right_motor_set_speed(vitesse_droite);
		left_motor_set_speed(vitesse_gauche);

		chThdSleepUntilWindowed(time, time + MS2ST(10)); //chgt : mise à une freq de 100Hz
		//baisser la fréquence du thread (ie augmenter le temps de sleep) si karnel panic
	}
}

void parcours_start(void){
	chThdCreateStatic(waParcours, sizeof(waParcours), NORMALPRIO+3, Parcours, NULL);
}
