#include "animations.h"
#include <main.h>
#include <chprintf.h>
#include <spi_comm.h>
#include <leds.h>
#include <usbcfg.h>
#include "ch.h"
#include "hal.h"
#include <audio/play_melody.h>


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

void validation_commande(void)
{
	set_body_led(TOGGLE);
	playNote(NOTE_F5, TEMPS_SON);
	set_body_led(TOGGLE);
}

void animation_leds(uint8_t etat_parcours, bool nvlle_action)
{
	static uint8_t compteur = 20;
	if(nvlle_action)
	{
		compteur = 0;
		clear_leds();
	}

	if (compteur != 0)
	{
		compteur --;
		return;
	}
	else
	{
		compteur = 20;
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
		}
	}
}
