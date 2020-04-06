#include "reception_capteur_IR.h"
#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <sensors/proximity.h>

static int val_ambiantes[PROXIMITY_NB_CHANNELS];

static int val_absolues[PROXIMITY_NB_CHANNELS];

static int val_calibrees[PROXIMITY_NB_CHANNELS];


void valeurs_ambiantes(void)
{
	for (int i = 0; i < PROXIMITY_NB_CHANNELS; i++)
	{
		val_ambiantes[i] = get_ambient_light(i);
		chprintf((BaseSequentialStream *)&SDU1, "lumiere ambiante du capteur %d : %d\n",i, val_ambiantes[i]);
	}
}

void valeurs_absolues(void)
{
	for (int i = 0; i < PROXIMITY_NB_CHANNELS; i++)
	{
		val_absolues[i] = get_prox(i);
		chprintf((BaseSequentialStream *)&SDU1, "valeurs absolues du capteur %d : %d \n", i, val_absolues);
	}
}

void valeurs_calibrees(void)
{
	for (int i = 0; i < PROXIMITY_NB_CHANNELS; i++)
	{
		val_calibrees[i] = get_calibrated_prox(i);
		chprintf((BaseSequentialStream *)&SDU1, "valeurs calibrees du capteur %d : %d \n", i, val_calibrees[i]);
	}
}
