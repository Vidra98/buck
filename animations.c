#include "animations.h"
#include <main.h>
#include <chprintf.h>
#include <spi_comm.h>
#include <leds.h>
#include "ch.h"
#include "parcours.h"
#include <stdbool.h>

#define PI					3.14159265358979f
#define SEC					1000
#define TOGGLE				2
#define VALIDATION			3
#define MAX_INTENSITY		255
#define MEDIUM_INTENSITY    128

#define ON					1
#define OFF					0

static bool commande_validee = false,nouvelle_commande = false;

void animation_leds(void);

int get_animations_commande_validee(void){
	return commande_validee;
}

void set_nouvelle_commande(void){
	nouvelle_commande=true;
}

void set_tracking_leds(float angle){
	clear_leds();

	if ((angle<2*PI/8)  || (angle>14*PI/8))   	set_led(LED1,ON);
	if ((PI/8<angle)    && (3*PI/8>angle))		set_rgb_led(LED8,MAX_INTENSITY,0,0);
	if ((2*PI/8<angle)  && (6*PI/8>angle))   	set_led(LED7,ON);
	if ((5*PI/8<angle)  && (7*PI/8>angle))		set_rgb_led(LED6,MAX_INTENSITY,0,0);
	if ((6*PI/8<angle)  && (10*PI/8>angle))		set_led(LED5,ON);
	if ((9*PI/8<angle)  && (11*PI/8>angle))		set_rgb_led(LED4,MAX_INTENSITY,0,0);
	if ((10*PI/8<angle) && (14*PI/8>angle)) 	set_led(LED3,ON);
	if ((13*PI/8<angle) && (15*PI/8>angle))		set_rgb_led(LED2,MAX_INTENSITY,0,0);

	set_front_led(ON);
}

void animation_leds(void)
{
	static uint8_t etat_parcours_buf = MVT_IDLE;

	uint8_t etat_parcours = get_parcours_etat();
	switch(etat_parcours){
	case TRACKING:
		clear_leds();
		break;
	case MVT_IDLE:
		//les leds rgb clignotent en vert uniqument si elles ne sont pas occupées pour donner la direction du son
		if ((etat_parcours != etat_parcours_buf)) clear_leds();
		toggle_rgb_led(LED2, GREEN_LED, MAX_INTENSITY);
		toggle_rgb_led(LED4, GREEN_LED, MAX_INTENSITY);
		toggle_rgb_led(LED6, GREEN_LED, MAX_INTENSITY);
		toggle_rgb_led(LED8, GREEN_LED, MAX_INTENSITY);
		break;

		// à partir ou buck rentre en routine contourne les obstacles, les leds sont réquisitionnées en priorité
		// elles indiquent l'état du contournement
	case CONTOURNEMENT:
		if (etat_parcours != etat_parcours_buf) clear_leds();
		toggle_rgb_led(LED2, RED_LED, MAX_INTENSITY); // rouge
		toggle_rgb_led(LED4, RED_LED, MAX_INTENSITY);
		toggle_rgb_led(LED6, RED_LED, MAX_INTENSITY);
		toggle_rgb_led(LED8, RED_LED, MAX_INTENSITY);
		break;

	case LONGEMENT:
		if (etat_parcours != etat_parcours_buf) clear_leds();
		toggle_rgb_led(LED2, GREEN_LED, MEDIUM_INTENSITY); //orange
		toggle_rgb_led(LED4, GREEN_LED, MEDIUM_INTENSITY);
		toggle_rgb_led(LED6, GREEN_LED, MEDIUM_INTENSITY);
		toggle_rgb_led(LED8, GREEN_LED, MEDIUM_INTENSITY);
		toggle_rgb_led(LED2, RED_LED, MAX_INTENSITY);
		toggle_rgb_led(LED4, RED_LED, MAX_INTENSITY);
		toggle_rgb_led(LED6, RED_LED, MAX_INTENSITY);
		toggle_rgb_led(LED8, RED_LED, MAX_INTENSITY);
		break;

	case RETOUR_TRAJECTOIRE:
		if (etat_parcours != etat_parcours_buf) clear_leds();
		toggle_rgb_led(LED2, GREEN_LED, MAX_INTENSITY); //jaune
		toggle_rgb_led(LED4, GREEN_LED, MAX_INTENSITY);
		toggle_rgb_led(LED6, GREEN_LED, MAX_INTENSITY);
		toggle_rgb_led(LED8, GREEN_LED, MAX_INTENSITY);
		toggle_rgb_led(LED2, RED_LED, MAX_INTENSITY);
		toggle_rgb_led(LED4, RED_LED, MAX_INTENSITY);
		toggle_rgb_led(LED6, RED_LED, MAX_INTENSITY);
		toggle_rgb_led(LED8, RED_LED, MAX_INTENSITY);
		break;

	case PARCOURS_INFINI :
		if (etat_parcours != etat_parcours_buf) clear_leds();
		toggle_rgb_led(LED2, BLUE_LED, MAX_INTENSITY); //bleu
		toggle_rgb_led(LED4, BLUE_LED, MAX_INTENSITY);
		toggle_rgb_led(LED6, BLUE_LED, MAX_INTENSITY);
		toggle_rgb_led(LED8, BLUE_LED, MAX_INTENSITY);
		break;
	}
	etat_parcours_buf=etat_parcours;
}



static THD_WORKING_AREA(waAnimations, 64);
static THD_FUNCTION(Animations, arg){
	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;
	static uint8_t compteur_validation = 0;

	while(1)
	{
		time = chVTGetSystemTime();
		//fais clignoter la body led si un son est tracké/fuis
		if(nouvelle_commande)
		{
			commande_validee=false;
			compteur_validation ++;
			set_body_led(TOGGLE);
			if(compteur_validation == VALIDATION)
			{
				set_body_led(OFF);
				nouvelle_commande=false;
				compteur_validation = 0;
				commande_validee=true;
			}
		}
		set_front_led(OFF);
		animation_leds();
		chThdSleepUntilWindowed(time, time + MS2ST(500)); //mise à une freq de 2Hz
	}
}

void animations_start(void){
	chThdCreateStatic(waAnimations, sizeof(waAnimations), LOWPRIO, Animations, NULL);
}
