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
static bool nouvelle_commande = false, aller_en_avant = false, aller_en_arriere = false, parcours_infini =false;
static systime_t time_start;

/*
 *  Défini le comportement du robot en fonction de l'intensité et frequence du son reçu
 */
void commande_recu(void);

/* Définit un parcours de buck selon le symbôle infini
 * Parcours composé de 2 lignes droites de 30 cm et de 2 virages d'un rayon de 21cm
 * (15.7cm pour la roue intérieure et 26.3cm pour la roue extérieure)
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

void reponse_sonore(void){
	int16_t vitesse_d,vitesse_g;
	float c_angle, s_angle, angle;
	static int16_t sum_error;
	angle = get_angle();
	c_angle=cosf(angle);
	s_angle=sinf(angle);
	// dirige le robot vers la source de son pour les fréquences [260,400]Hz
	if(aller_en_avant)
	{
		time_start = chVTGetSystemTime();
		aller_en_arriere = false;
		sum_error += (COS_AVANT-c_angle);
		if (abs(sum_error)> MAX_SUM_ERROR_SON) sum_error=MAX_SUM_ERROR_SON;

		vitesse_d=(COS_AVANT-c_angle)*KP_SON+(sum_error)*KI_SON+KA_SON*c_angle;
		vitesse_g=-((COS_AVANT-c_angle)*KP_SON+(sum_error)*KI_SON)+KA_SON*c_angle;

		if (s_angle > NUL){
			vitesse_droite=vitesse_d;
			vitesse_gauche=vitesse_g;
		}
		//on inverse les vitesses gauche et droite pour un retour plus efficace a la position d'equilibre
		else {
			vitesse_droite=vitesse_g;
			vitesse_gauche=vitesse_d;
		}

		//set_tracking_leds est apellé ici pour pouvoir réagir plus vite que dans le thread animation qui est de basse frequence.
		set_tracking_leds(angle);
	}
	//dirige le robot à l'inverse de la source de son pour les frequence [400,555]Hz
	else if (aller_en_arriere)
	{
		time_start = chVTGetSystemTime();
		aller_en_avant = false;


		sum_error += (COS_ARRIERE-c_angle);
		if (abs(sum_error)> MAX_SUM_ERROR_SON) sum_error=MAX_SUM_ERROR_SON;

		vitesse_d=(COS_ARRIERE-c_angle)*KP_SON+(sum_error)*KI_SON-KA_SON*c_angle;
		vitesse_g=-((COS_ARRIERE-c_angle)*KP_SON+(sum_error)*KI_SON)-KA_SON*c_angle;

		if (s_angle > NUL){
			vitesse_droite=vitesse_d;
			vitesse_gauche=vitesse_g;
		}
		//on inverse les vitesses gauche et droite pour un retour plus efficace a la position d'equilibre

		else {
			vitesse_droite=vitesse_g;
			vitesse_gauche=vitesse_d;
		}

		set_tracking_leds(angle);

	}
	//pas de mouvement sinon
	else {
		vitesse_droite=0;
		vitesse_gauche=0;
	}
}
/*
 * j'ai supprimé ça, c'était utile?
 * 			right_motor_set_pos(0);
			left_motor_set_pos(0);
 */
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


int16_t pi_retour_trajectoire(int32_t objectif, int32_t pos_actuelle)
{
	float error = 0;
	float speed = 0;
	static float sum_error = 0;

	error = abs(pos_actuelle) - abs(objectif);
	//pour eviter des emballements
	if (error > MAX_ERROR)	error = MAX_ERROR;
	else if (error < -MAX_ERROR) error = -MAX_ERROR;

	sum_error += error;
	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR) sum_error = MAX_SUM_ERROR;
	else if(sum_error < -MAX_SUM_ERROR) sum_error = -MAX_SUM_ERROR;

	speed = KP * error + KI * sum_error;

	return (int16_t)speed;
}

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
		vitesse_pi_r = pi_retour_trajectoire(compteur_droite, right_motor_get_pos());
		vitesse_pi_l = pi_retour_trajectoire(compteur_gauche, left_motor_get_pos());

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
			time_start = chVTGetSystemTime();
		}
		break;
	}
}

void commande_recu(void)
{
	float freq=get_freq(), amp=get_amp();
	if ((freq>AVANT_FREQ_MIN)&&(freq<AVANT_FREQ_MAX)&&(amp>MIN_INTENSITY_TRACKING))
	{
		if (!aller_en_avant)
		{
			aller_en_avant = true;
			set_nouvelle_commande();
		}
	}

	else if ((freq>ARRIERE_FREQ_MIN)&&(freq<ARRIERE_FREQ_MAX)&&(amp>MIN_INTENSITY_TRACKING))
	{
		if(!aller_en_arriere)
		{
			aller_en_arriere = true;
			set_nouvelle_commande();
		}
	}
	else
	{
		aller_en_avant = false;
		aller_en_arriere = false;
	}
}

bool lancement_idle(void)
{
	systime_t time, delta_t;
	if (etat_contournement==MVT_IDLE)
	{
		if ((aller_en_avant==false)&&(aller_en_arriere==false))
		{
			time=chVTGetSystemTime();
			delta_t = time - time_start;
			if (ST2S(delta_t) >= TEMPS_IDLE)
			{
				parcours_infini =true;
				return true;
			}
		}
	}
	parcours_infini=false;
	return false;
}

//focntions qui sont appelées par animations pour faire les différentes animations
uint8_t get_parcours_etat(void)
{
	if (etat_contournement==MVT_IDLE){
		if ((aller_en_avant==true) || (aller_en_arriere==true)) return TRACKING;
		else if (parcours_infini) return PARCOURS_INFINI;
	}
	return etat_contournement;
}

bool get_idle_state(void){
	if ((aller_en_avant==true) || (aller_en_arriere==true)) return true;
	return false;
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
	while (1)
	{
		time = chVTGetSystemTime();
		commande_recu();
		valeurs_calibrees();
		obstacle_devant = get_obstacle_condition(CAPTEUR_HAUT_DROITE);
		obstacle_devant_droit = get_obstacle_condition(CAPTEUR_HAUT_DROITE_45);
		obstacle_devant_gauche = get_obstacle_condition(CAPTEUR_HAUT_GAUCHE_45);
		//les routines d'animations sont appelées dans cette fonction
		parcours_obstacle(obstacle_devant, obstacle_devant_droit, obstacle_devant_gauche);

		if (lancement_idle()) parcours_en_infini();

		right_motor_set_speed(vitesse_droite);
		left_motor_set_speed(vitesse_gauche);
		chThdSleepUntilWindowed(time, time + MS2ST(10)); //chgt : mise à une freq de 100Hz
		//baisser la fréquence du thread (ie augmenter le temps de sleep) si karnel panic
	}
												}

void parcours_start(void){
	chThdCreateStatic(waParcours, sizeof(waParcours), NORMALPRIO+1, Parcours, NULL);
}
