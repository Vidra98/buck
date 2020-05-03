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

static bool commande_validee = false;

void set_direction_led(float angle)
{
	if ((angle<2*PI/8) || (angle>14*PI/8))   set_led(LED1,1);
	if ((PI/8<angle) && (3*PI/8>angle))		 set_rgb_led(LED8,255,0,0);
	if ((2*PI/8<angle) && (6*PI/8>angle))    set_led(LED7,1);
	if ((5*PI/8<angle) && (7*PI/8>angle))	 set_rgb_led(LED6,255,0,0);
	if ((6*PI/8<angle) && (10*PI/8>angle))   set_led(LED5,1);
	if ((9*PI/8<angle) && (11*PI/8>angle))	 set_rgb_led(LED4,255,0,0);
	if ((10*PI/8<angle) && (14*PI/8>angle))  set_led(LED3,1);
	if ((13*PI/8<angle) && (15*PI/8>angle))	 set_rgb_led(LED2,255,0,0);
}


void animation_leds(uint8_t etat_parcours)
{

	switch(etat_parcours)
	{
	case 0:
		toggle_rgb_led(LED2, GREEN_LED, 255);
		toggle_rgb_led(LED4, GREEN_LED, 255);
		toggle_rgb_led(LED6, GREEN_LED, 255);
		toggle_rgb_led(LED8, GREEN_LED, 255);
		break;

	case 1:
		toggle_rgb_led(LED2, RED_LED, 255);
		toggle_rgb_led(LED4, RED_LED, 255);
		toggle_rgb_led(LED6, RED_LED, 255);
		toggle_rgb_led(LED8, RED_LED, 255);
		break;

	case 2:
		toggle_rgb_led(LED2, GREEN_LED, 255);
		toggle_rgb_led(LED4, GREEN_LED, 255);
		toggle_rgb_led(LED6, GREEN_LED, 255);
		toggle_rgb_led(LED8, GREEN_LED, 255);
		toggle_rgb_led(LED2, RED_LED, 255);
		toggle_rgb_led(LED4, RED_LED, 255);
		toggle_rgb_led(LED6, RED_LED, 255);
		toggle_rgb_led(LED8, RED_LED, 255);
		break;

	case 3:
		toggle_rgb_led(LED2, GREEN_LED, 128);
		toggle_rgb_led(LED4, GREEN_LED, 128);
		toggle_rgb_led(LED6, GREEN_LED, 128);
		toggle_rgb_led(LED8, GREEN_LED, 128);
		toggle_rgb_led(LED2, RED_LED, 255);
		toggle_rgb_led(LED4, RED_LED, 255);
		toggle_rgb_led(LED6, RED_LED, 255);
		toggle_rgb_led(LED8, RED_LED, 255);
		break;

	default :
		toggle_rgb_led(LED2, BLUE_LED, 255);
		toggle_rgb_led(LED4, BLUE_LED, 255);
		toggle_rgb_led(LED6, BLUE_LED, 255);
		toggle_rgb_led(LED8, BLUE_LED, 255);
	}
}

bool get_animation_commande_validee(void)
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
	bool commande = false;
	float angle_son = 0.;

	while(1)
	{
		time = chVTGetSystemTime();
		uint8_t etat_buck = get_parcours_etat();
		bool commande = get_parcours_nouvelle_commande();
		float angle_son = get_parcours_angle();
		if(commande)
		{
			commande_validee = false;
		}

		if(!commande_validee)
		{
			set_body_led(TOGGLE);
			playNote(NOTE_F5, TEMPS_SON);
			set_body_led(TOGGLE);
			commande_validee = true;
		}

		else
		{
			animation_leds(etat_buck);
			set_direction_led(angle_son);
		}
		chThdSleepUntilWindowed(time, time + MS2ST(500)); //mise à une freq de 2Hz
	}
}

void animations_start(void){
	chThdCreateStatic(waAnimations, sizeof(waAnimations), NORMALPRIO+3, Animations, NULL);
}
