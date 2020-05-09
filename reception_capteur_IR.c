#include "reception_capteur_IR.h"
#include <sensors/proximity.h>

//en vue des différentes valeurs aléatoires qui pouvait arriver sur les capteurs
//nous avons décidé d'interpreter l'arrivée d'un obstacle comme 3 valeurs d'affilée sur un capteur dépassant les valeurs seuil
#define MESURES_AFFILE			3
#define OBSTACLE_ENV_5CM		150
#define OBSTACLE_ENV_2CM		600
#define OBSTACLE_EN_CONTACT 	2000

static uint32_t val_calibrees[PROXIMITY_NB_CHANNELS];

static bool obstacle[PROXIMITY_NB_CHANNELS];

// Fonction qui renvoie une valeur calibrée d'un capteur, pour le PI
// Est appelée uniquement si nécessaire
uint32_t valeur_sur_capteur(int32_t num_capteur);

bool check_chemin(void)
{
	if (!obstacle[CAPTEUR_HAUT_GAUCHE_45])
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
	}
}

