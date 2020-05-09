#include "parcours.h"
#include <main.h>
#include "ch.h"
#include <motors.h>
#include <sensors/proximity.h>
#include <math.h>
#include <traitement_son.h>
#include "animations.h"
#include <stdbool.h>
#include "traitement_capteur.h"

//Temps d'attente [seconde] avant d'entrer en état d'idle (avec le parcours)
#define TEMPS_IDLE							5

/*Pour une vitesse lineaire de 10cm/s avec la conversion de 1 tour/s = 1000 steps/s et v = r*Omega
 * Sachant que r = 13/(2*pi)
 */
#define VITESSE_IDLE						769

// Pour une vitesse d'environ 7.5 cm/s
#define VITESSE_LIN 						577



//coefficient du regulateur pour le retour de trajectoire (une fois l'obstacle contourné)
#define VITESSE_LIM							900
#define KP 									5
#define KI									0.1
#define MAX_SUM_ERROR 						VITESSE_LIM/KP
#define MAX_ERROR 							VITESSE_LIM/KI
#define MARGE_VITESSE						25

//coefficient du regulateur pour le tracking sonore
#define VITESSE_LIM_PI_SON					900
#define KP_SON								600
#define KI_SON								3
#define MAX_SUM_ERROR_SON 					VITESSE_LIM/6
#define MAX_ERROR_SON 						VITESSE_LIM
#define KA_SON								900

#define PI									3.14159265358979f
#define COS_AVANT							1.0f
#define COS_ARRIERE							-1.0f
#define COS_MARGE							0.05f

// Bandes passantes des commandes
#define AVANT_FREQ_MIN						10		//152hz
#define AVANT_FREQ_MAX						27		//411hz
#define ARRIERE_FREQ_MIN					27		//411hz
#define ARRIERE_FREQ_MAX					37		//563hz

//intensity min du son pour etre tracké
#define MIN_INTENSITY_TRACKING	    10000

//Constantes pour respecter les contraintes géométriques du parcours d'idle (en infini)
#define ERREURS_STEPS						10
#define LIMITE_STEPS_LIGNE_DROITE			2310
#define LIMITE_STEPS_PETIT_VIRAGE			1613
#define LIMITE_STEPS_GRAND_VIRAGE 			3322
#define VITESSE_PETIT_VIRAGE 				373


static int16_t vitesse_droite, vitesse_gauche;
static uint8_t etat_contournement = MVT_IDLE;
static bool aller_en_avant = false, aller_en_arriere = false, parcours_infini =false;
static systime_t time_start;

/*Définit le comportement du robot en fonction de l'intensité et frequence du son reçu
 *
 */
void commande_recu(void);

/*
 * Définit un parcours de buck selon le symbôle infini
 * Parcours composé de 2 lignes droites de 30 cm et de 2 virages d'un rayon de 7.65cm
 * (10.5cm pour le virage extérieur et 5cm pour le virage intérieur)
 */
void parcours_en_infini(void);


/* Fonction qui déclenche un controlleur PI pour le retour sur trajectoire (une fois l'obstacle contourné)
 * reçoit en paramètre la position des moteurs enregistrées lors du contournement
 * et la position actuelle lors du retour en trajectoire
 * IMPORTANT : ce PI n'est pas le même que celui utilisé pour réponse_sonore
 */
int16_t pi_controller(int32_t objectif, int32_t pos_actuelle);


/* Définit le chemin de notre robot avec la réaction au son ou bien l'évitement d'un obstacle
 *
 */
void parcours_obstacle(bool obstacle_devant_droite, bool obstacle_devant_gauche,
								bool obstacle_droite_45, bool obstacle_gauche_45);

/*
 * Pour suivre ou bien s'éloigner du son
 */
void reponse_sonore(void);

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
		//Tant que Buck n'a pas clignoté ses body leds, il ne bouge pas
		if(get_animations_commande_validee())
		{
			sum_error += (COS_AVANT-c_angle);
			if (abs(sum_error)> MAX_SUM_ERROR_SON) sum_error=MAX_SUM_ERROR_SON;

			vitesse_d=(COS_AVANT-c_angle)*KP_SON+(sum_error)*KI_SON+KA_SON*c_angle;
			vitesse_g=-((COS_AVANT-c_angle)*KP_SON+(sum_error)*KI_SON)+KA_SON*c_angle;

			if (s_angle > 0){
				vitesse_droite=vitesse_d;
				vitesse_gauche=vitesse_g;
			}
			//on inverse les vitesses gauche et droite pour un retour plus efficace a la position d'equilibre
			else {
				vitesse_droite=vitesse_g;
				vitesse_gauche=vitesse_d;
			}
			//Pour indiquer la direction du son
			set_tracking_leds(angle);
		}
	}
	//dirige le robot à l'inverse de la source de son pour les frequence [400,555]Hz
	else if (aller_en_arriere)
	{
		time_start = chVTGetSystemTime();

		aller_en_avant = false;
		if (get_animations_commande_validee())
		{
			sum_error += (COS_ARRIERE-c_angle);
			if (sum_error<-MAX_SUM_ERROR_SON) sum_error=-MAX_SUM_ERROR_SON;

			vitesse_d=(COS_ARRIERE-c_angle)*KP_SON+(sum_error)*KI_SON-KA_SON*c_angle;
			vitesse_g=-((COS_ARRIERE-c_angle)*KP_SON+(sum_error)*KI_SON)-KA_SON*c_angle;

			if (s_angle > 0){
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
	}
	else {
		vitesse_droite=0;
		vitesse_gauche=0;
	}
}

void parcours_en_infini(void)
{
	int32_t compteur_droit = right_motor_get_pos();
	int32_t compteur_gauche = left_motor_get_pos();
	static uint8_t etat_parcours = PREM_LIGNE_DROITE;
	/*Séquence ligne droite - virage - ligne droite - virage que l'on répète en boucle
	* Le passage d'une étape à l'autre s'effectue avec l'actualisation de la position des moteurs
	*/
	switch(etat_parcours)
	{
	case PREM_LIGNE_DROITE :
		if ((abs(compteur_droit - LIMITE_STEPS_LIGNE_DROITE) < ERREURS_STEPS) ||
				(abs(compteur_gauche - LIMITE_STEPS_LIGNE_DROITE) < ERREURS_STEPS))
		{
			etat_parcours = PREM_VIRAGE;
			right_motor_set_pos(0);
			left_motor_set_pos(0);
			break;
		}
		vitesse_droite = VITESSE_IDLE;
		vitesse_gauche = VITESSE_IDLE;
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
		vitesse_gauche = VITESSE_IDLE;

		break;

	case SEC_LIGNE_DROITE :
		if ((abs(compteur_droit - LIMITE_STEPS_LIGNE_DROITE) < ERREURS_STEPS) ||
				(abs(compteur_gauche - LIMITE_STEPS_LIGNE_DROITE) < ERREURS_STEPS))
		{
			etat_parcours = SEC_VIRAGE;
			right_motor_set_pos(0);
			left_motor_set_pos(0);
			break;
		}
		vitesse_droite = VITESSE_IDLE;
		vitesse_gauche = VITESSE_IDLE;
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
		vitesse_droite = VITESSE_IDLE;
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

void parcours_obstacle(bool obstacle_devant_droite, bool obstacle_devant_gauche,
								bool obstacle_droite_45	, bool obstacle_gauche_45)
{
	int16_t vitesse_pi_r = 0;
	int16_t vitesse_pi_l = 0;
	static int32_t compteur_droite = 0;
	static int32_t compteur_gauche = 0;
	static bool tourner_a_gauche = false;
	static bool obstacle_cote_90 = false;

	switch(etat_contournement)
	{
	case EN_REACTION :
		if ((!obstacle_devant_droite) && (!obstacle_devant_gauche)
					&&(!obstacle_droite_45)	&& (!obstacle_gauche_45))
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

		if ((!obstacle_devant_droite) && (!obstacle_devant_gauche)
					&&(!obstacle_droite_45)	&& (!obstacle_gauche_45))
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
				vitesse_droite = VITESSE_LIN;
				vitesse_gauche = -VITESSE_LIN;
			}

			else
			{
				vitesse_droite = -VITESSE_LIN;
				vitesse_gauche = VITESSE_LIN;
			}
			compteur_droite = right_motor_get_pos();
			compteur_gauche = left_motor_get_pos();
		}
		break;

	case LONGEMENT:
		//Si un nouvel obstacle se présente devant, on repasse dans contournement
		if ((!obstacle_devant_droite) && (!obstacle_devant_gauche))
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
				vitesse_droite = VITESSE_LIN;
				vitesse_gauche = VITESSE_LIN;
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

		/* Les vitesses du pi étant toujours négatives il faut comprendre les "+" et "-" dans le raisonnement inverse
		 * Ainsi pour diminuer la vitesse finale des moteurs (centrées à 7.5 cm/s), il faut "rajouter" de la vitesse négative
		 * Pour le retour en trajectoire, si on a contourné par la gauche il faut revenir sur la droite et inversement
		 */
		if (tourner_a_gauche)
		{
			vitesse_droite = VITESSE_LIN + vitesse_pi_r;
			vitesse_gauche = VITESSE_LIN - vitesse_pi_l;
		}
		else
		{
			vitesse_droite = VITESSE_LIN - vitesse_pi_r;
			vitesse_gauche = VITESSE_LIN + vitesse_pi_l;
		}
		/*Plutôt que d'éteindre le PI, on considère le retour en trajectoire effectué si la vitesse des PI passe en dessous d'un seuil
		 * Cela revient à le compteur actuel du moteur est proche de la valeur enregistrée
		 */
		if (abs(vitesse_pi_r) < MARGE_VITESSE || abs(vitesse_pi_l) < MARGE_VITESSE)
		{
			tourner_a_gauche = false;
			obstacle_cote_90 = false;
			etat_contournement = EN_REACTION;
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
	if (etat_contournement==EN_REACTION)
	{
		if ((!aller_en_avant) && (!aller_en_arriere))
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

uint8_t get_parcours_etat(void)
{
	if (etat_contournement==EN_REACTION){
		if ((aller_en_avant) || (aller_en_arriere)) return TRACKING;
		else if (parcours_infini) return MVT_IDLE;
	}
	return etat_contournement;
}

// Thread avec les routines controlant les moteurs
static THD_WORKING_AREA(waParcours, 128);
static THD_FUNCTION(Parcours, arg)
												{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;
	bool obstacle_devant_droite = false, obstacle_devant_gauche = false,
			obstacle_droite_45 = false, obstacle_gauche_45 = false;

	while (1)
	{
		time = chVTGetSystemTime();

		commande_recu();

		actualisation_capteurs();
		obstacle_devant_droite = get_obstacle_condition(CAPTEUR_HAUT_DROITE);
		obstacle_devant_gauche = get_obstacle_condition(CAPTEUR_HAUT_GAUCHE);
		obstacle_droite_45 = get_obstacle_condition(CAPTEUR_HAUT_DROITE_45);
		obstacle_gauche_45 = get_obstacle_condition(CAPTEUR_HAUT_GAUCHE_45);

		parcours_obstacle(obstacle_devant_droite, obstacle_devant_gauche,
									obstacle_droite_45, obstacle_gauche_45);

		//En appelant le parcours d'Idle avec le parcours obstacle, on peut gérer l'intérraction avec les obstacles en idle
		if (lancement_idle()) parcours_en_infini();

		right_motor_set_speed(vitesse_droite);
		left_motor_set_speed(vitesse_gauche);

		chThdSleepUntilWindowed(time, time + MS2ST(10)); //mise à une freq de 100Hz
	}
												}

void parcours_start(void){
	chThdCreateStatic(waParcours, sizeof(waParcours), NORMALPRIO, Parcours, NULL);
}
