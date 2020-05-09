#ifndef ANIMATIONS_H
#define ANIMATIONS_H

void set_nouvelle_commande(void);

// Allume une des 8 leds sur le haut de buck selon la direction du son incident
void set_tracking_leds(float angle);

int get_animations_commande_validee(void);
//demarage du thread d'animation
void animations_start(void);

#endif
