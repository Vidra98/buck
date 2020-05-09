#ifndef ANIMATIONS_H
#define ANIMATIONS_H


#include <stdbool.h>


/* Pour indiquer que le robot a reçu une nouvelle commande et qu'il doit clignoter ses body leds
 *
 */
void set_nouvelle_commande(void);


/*
 * Allume les leds du dessus (leds rouges et rgb en rouge) pour indiquer la direction du son incident
 * Seule fonction d'animation appelée dans parcours car une fréquence de 2Hz n'était pas suffisant pour rafraichir les leds
 */
void set_tracking_leds(float angle);


/*
 * Pour indiquer si Buck a bien clignoté ses body leds avant de se mettre en marche
 */
bool get_animations_commande_validee(void);


/* Demarrage du Thread de waAnimations
 *
 */
void animations_start(void);

#endif
