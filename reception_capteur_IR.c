#include "reception_capteur_IR.h"
#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <sensors/proximity.h>

static uint32_t val_ambiantes[PROXIMITY_NB_CHANNELS];

static uint32_t val_absolues[PROXIMITY_NB_CHANNELS];

static uint32_t val_calibrees[PROXIMITY_NB_CHANNELS];

static int16_t valeurs_au_dessus_seuil = 0;

//static bool obstacle = false;

int32_t check_environnement(void)
{
	for (int i = 0; i < PROXIMITY_NB_CHANNELS; i++)
	{
		val_calibrees[i] = get_calibrated_prox(i);
		if (val_calibrees[i] > OBSTACLE_ENV_2CM)
		{
			valeurs_au_dessus_seuil++;
			if (valeurs_au_dessus_seuil == 3)
			{
				valeurs_au_dessus_seuil = 0;
				break;
				return i;
			}
		}
	}
	return PAS_DE_CAPTEUR;
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
		//chprintf((BaseSequentialStream *)&SDU1, "valeurs calibrees du capteur %d : %d \n", i, val_calibrees[i]);
	}
}

