#include "traitement_capteur.h"

#include <sensors/proximity.h>

/*
 * Valeur définie suite à des mesures avec une feuille blanche
 */

#define OBSTACLE_ENV_5CM		150


static bool obstacle[PROXIMITY_NB_CHANNELS];


bool check_chemin(void)
{
	return !obstacle[CAPTEUR_HAUT_GAUCHE_45];
}

bool get_obstacle_condition (CAPTEUR_NUMEROTATION numero_capteur)
{
	return obstacle[numero_capteur];
}


void actualisation_capteurs(void)
{
	uint32_t val_calibrees[PROXIMITY_NB_CHANNELS];

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
	}
}

