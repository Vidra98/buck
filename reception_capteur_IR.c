#include "reception_capteur_IR.h"
#include <sensors/proximity.h>

//en vue des diff�rentes valeurs al�atoires qui pouvait arriver sur les capteurs
//nous avons d�cid� d'interpreter l'arriv�e d'un obstacle comme 3 valeurs d'affil�e sur un capteur d�passant les valeurs seuil
#define MESURES_AFFILE			3
#define OBSTACLE_ENV_5CM		150
#define OBSTACLE_ENV_2CM		600
#define OBSTACLE_EN_CONTACT 	2000

static uint32_t val_calibrees[PROXIMITY_NB_CHANNELS];

static bool obstacle[PROXIMITY_NB_CHANNELS];

// Fonction qui renvoie une valeur calibr�e d'un capteur, pour le PI
// Est appel�e uniquement si n�cessaire
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

