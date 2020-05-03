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
void set_direction_led(float angle)
{
	if ((angle<2*PI/8) || (angle>14*PI/8))   set_led(LED1,1);
	if ((PI/8<angle) && (3*PI/8>angle))
	{
		set_rgb_led(LED8,255,0,0);
		rgb_occupee[RGB_LED8] = true;
	}
	if ((2*PI/8<angle) && (6*PI/8>angle))    set_led(LED7,1);
	if ((5*PI/8<angle) && (7*PI/8>angle))
	{
		set_rgb_led(LED6,255,0,0);
		rgb_occupee[RGB_LED6] = true;
	}
	if ((6*PI/8<angle) && (10*PI/8>angle))   set_led(LED5,1);
	if ((9*PI/8<angle) && (11*PI/8>angle))
	{
		set_rgb_led(LED4,255,0,0);
		rgb_occupee[RGB_LED4] = true;
	}
	if ((10*PI/8<angle) && (14*PI/8>angle))  set_led(LED3,1);
	if ((13*PI/8<angle) && (15*PI/8>angle))
	{
		set_rgb_led(LED2,255,0,0);
		rgb_occupee[RGB_LED2] = true;
	}
	 set_front_led(1);
}


void animation_leds(uint8_t etat_parcours)
{

	switch(etat_parcours)
	{
	case MVT_IDLE:
	//les leds rgb clignotent en vert uniqument si elles ne sont pas occupées pour donner la direction du son
	if(!rgb_occupee[RGB_LED2])	toggle_rgb_led(LED2, GREEN_LED, 255);
	if(!rgb_occupee[RGB_LED4])	toggle_rgb_led(LED4, GREEN_LED, 255);
	if(!rgb_occupee[RGB_LED6])	toggle_rgb_led(LED6, GREEN_LED, 255);
	if(!rgb_occupee[RGB_LED8])	toggle_rgb_led(LED8, GREEN_LED, 255);
		break;

// à partir ou buck rentre en routine contourne les obstacles, les leds sont réquisitionnées en priorité
// elles indiquent l'état du contournement
	case CONTOURNEMENT:
		toggle_rgb_led(LED2, RED_LED, 255); // rouge
		toggle_rgb_led(LED4, RED_LED, 255);
		toggle_rgb_led(LED6, RED_LED, 255);
		toggle_rgb_led(LED8, RED_LED, 255);
		break;

	case LONGEMENT:
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
		toggle_rgb_led(LED2, BLUE_LED, 255); //bleu
		toggle_rgb_led(LED4, BLUE_LED, 255);
		toggle_rgb_led(LED6, BLUE_LED, 255);
		toggle_rgb_led(LED8, BLUE_LED, 255);
	}
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
	bool commande = false;
	float angle_son = 0.;
	static uint8_t compteur_validation = 1;

	while(1)
	{
		time = chVTGetSystemTime();
		uint8_t etat_buck = get_parcours_etat();
		bool commande = get_parcours_nouvelle_commande();
		float angle_son = get_parcours_angle();
		if(commande)
		{
			compteur_validation ++;
			set_body_led(TOGGLE);
			if(compteur_validation == VALIDATION)
			{
				commande_validee = true;
				compteur_validation = 1;
			}
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
