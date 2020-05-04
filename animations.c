#include "animations.h"
#include <main.h>
#include <chprintf.h>
#include <spi_comm.h>
#include <leds.h>
#include <usbcfg.h>
#include "ch.h"
#include "hal.h"
#include <audio/play_melody.h>
#include "parcours.h"
#include <traitement_son.h>

#define TRACKING			1
#define REST				0

static bool commande_validee = false;
static bool rgb_occupee[NB_RGB_LED];



void reset_leds(void)
{
	set_led(LED1, 0);
	set_led(LED3, 0);
	set_led(LED5, 0);
	set_led(LED7, 0);
	set_body_led(0);
	set_front_led(0);
	for (int i = 0; i < NB_RGB_LED; i++)
	{
		rgb_occupee[i] = false;
	}
}
void set_tracking_leds(float angle){
	clear_leds();
	angle=get_angle();
	if ((angle<2*PI/8) || (angle>14*PI/8))   	set_led(LED1,1);
	if ((PI/8<angle) && (3*PI/8>angle))			set_rgb_led(LED8,255,0,0);
	if ((2*PI/8<angle) && (6*PI/8>angle))   	set_led(LED7,1);
	if ((5*PI/8<angle) && (7*PI/8>angle))		set_rgb_led(LED6,255,0,0);
	if ((6*PI/8<angle) && (10*PI/8>angle))		set_led(LED5,1);
	if ((9*PI/8<angle) && (11*PI/8>angle))		set_rgb_led(LED4,255,0,0);
	if ((10*PI/8<angle) && (14*PI/8>angle)) 	set_led(LED3,1);
	if ((13*PI/8<angle) && (15*PI/8>angle))		set_rgb_led(LED2,255,0,0);
	set_front_led(1);
}

void animation_leds(void)
{
	static uint8_t etat_parcours_buf = MVT_IDLE, idle_state_buf=REST;

	float angle;
	bool idle_state;
	uint8_t etat_parcours = get_parcours_etat();
	switch(etat_parcours)
	{
	case MVT_IDLE:
		//les leds rgb clignotent en vert uniqument si elles ne sont pas occupées pour donner la direction du son
		idle_state=get_idle_state();
		if (idle_state==TRACKING){
			idle_state_buf=idle_state;
		}
		else {
			if ((etat_parcours != etat_parcours_buf)||(idle_state_buf=TRACKING)) clear_leds();
			toggle_rgb_led(LED2, GREEN_LED, 255);
			toggle_rgb_led(LED4, GREEN_LED, 255);
			toggle_rgb_led(LED6, GREEN_LED, 255);
			toggle_rgb_led(LED8, GREEN_LED, 255);
			idle_state_buf=REST;
		}
		break;

		// à partir ou buck rentre en routine contourne les obstacles, les leds sont réquisitionnées en priorité
		// elles indiquent l'état du contournement
	case CONTOURNEMENT:
		if (etat_parcours != etat_parcours_buf) clear_leds();
		toggle_rgb_led(LED2, RED_LED, 255); // rouge
		toggle_rgb_led(LED4, RED_LED, 255);
		toggle_rgb_led(LED6, RED_LED, 255);
		toggle_rgb_led(LED8, RED_LED, 255);
		break;

	case LONGEMENT:
		if (etat_parcours != etat_parcours_buf) clear_leds();
		toggle_rgb_led(LED2, GREEN_LED, 255); //orange
		toggle_rgb_led(LED4, GREEN_LED, 255);
		toggle_rgb_led(LED6, GREEN_LED, 255);
		toggle_rgb_led(LED8, GREEN_LED, 255);
		toggle_rgb_led(LED2, RED_LED, 255);
		toggle_rgb_led(LED4, RED_LED, 255);
		toggle_rgb_led(LED6, RED_LED, 255);
		toggle_rgb_led(LED8, RED_LED, 255);
		break;

	case RETOUR_TRAJECTOIRE:
		if (etat_parcours != etat_parcours_buf) clear_leds();
		toggle_rgb_led(LED2, GREEN_LED, 128); //jaune
		toggle_rgb_led(LED4, GREEN_LED, 128);
		toggle_rgb_led(LED6, GREEN_LED, 128);
		toggle_rgb_led(LED8, GREEN_LED, 128);
		toggle_rgb_led(LED2, RED_LED, 255);
		toggle_rgb_led(LED4, RED_LED, 255);
		toggle_rgb_led(LED6, RED_LED, 255);
		toggle_rgb_led(LED8, RED_LED, 255);
		break;

		//correspond à l'idle
	default :
		if (etat_parcours != etat_parcours_buf) clear_leds();

		toggle_rgb_led(LED2, BLUE_LED, 255); //bleu
		toggle_rgb_led(LED4, BLUE_LED, 255);
		toggle_rgb_led(LED6, BLUE_LED, 255);
		toggle_rgb_led(LED8, BLUE_LED, 255);
	}
	etat_parcours_buf=etat_parcours;
}

bool get_animations_commande_validee(void)
{
	return commande_validee;
}

static THD_WORKING_AREA(waAnimations, 128);
static THD_FUNCTION(Animations, arg)
				{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;
	uint8_t etat_buck = 0;
	bool new_commande = false;
	float angle_son = 0.;
	static uint8_t compteur_validation = 1;

	while(1)
	{
		time = chVTGetSystemTime();
		bool new_commande = get_parcours_nouvelle_commande();
		float angle_son = get_angle();
		if(new_commande)
		{
			compteur_validation ++;
			set_body_led(ON);
			if(compteur_validation == VALIDATION)
			{
				set_body_led(OFF);
				commande_validee = true;
				compteur_validation = 1;
			}
		}
		animation_leds();
		chThdSleepUntilWindowed(time, time + MS2ST(500)); //mise à une freq de 2Hz
	}
				}

void animations_start(void){
	chThdCreateStatic(waAnimations, sizeof(waAnimations), NORMALPRIO+1, Animations, NULL);
}
