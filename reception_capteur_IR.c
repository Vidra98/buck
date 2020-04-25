#include "reception_capteur_IR.h"
#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <sensors/proximity.h>
#include "parcours.h"

static uint32_t val_ambiantes[PROXIMITY_NB_CHANNELS];

static uint32_t val_absolues[PROXIMITY_NB_CHANNELS];

static uint32_t val_calibrees[PROXIMITY_NB_CHANNELS];

static bool obstacle[PROXIMITY_NB_CHANNELS];


bool check_chemin(void)
{
	if (!obstacle[CAPTEUR_GAUCHE])
	{
		return true;
	}

	return false;
}


bool get_obstacle_condition (int32_t numero_capteur)
{
	return obstacle[numero_capteur];
}

uint32_t get_valeur_capteur(int32_t numero_capteur)
{
	return val_calibrees[numero_capteur];
}


void valeurs_ambiantes(void)
{
	for (int i = 0; i < PROXIMITY_NB_CHANNELS; i++)
	{
		val_ambiantes[i] = get_ambient_light(i);
		//chprintf((BaseSequentialStream *)&SDU1, "lumiere ambiante du capteur %d : %d\n",i, val_ambiantes[i]);
	}
}

void valeurs_absolues(void)
{
	for (int i = 0; i < PROXIMITY_NB_CHANNELS; i++)
	{
		val_absolues[i] = get_prox(i);
		//chprintf((BaseSequentialStream *)&SDU1, "valeurs absolues du capteur %d : %d \n", i, val_absolues);
	}
}

void valeurs_calibrees(void)
{
	for (int i = 0; i < PROXIMITY_NB_CHANNELS; i++)
	{

		val_calibrees[i] = get_calibrated_prox(i);
		if (val_calibrees[i] >= OBSTACLE_ENV_5CM)
		{
			obstacle[i] = true;
		}

		else
		{
			obstacle[i] = false;
		}
		//chprintf((BaseSequentialStream *)&SDU1, "valeurs calibrees du capteur %d : %d \n", i, val_calibrees[i]);
	}
}

