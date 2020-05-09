#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include <motors.h>
#include <audio/microphone.h>
#include <spi_comm.h>
#include <sensors/proximity.h>
#include <traitement_son.h>

#include "parcours.h"
#include "animations.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //Initialisation du bus de communication
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //initialistion des capteurs IR
    proximity_start();

    spi_comm_start();

    //démarrage des moteurs
    motors_init();

    //calibration des capteurs
    calibrate_ir();

    //Envoie les données du micro a la fonction process audio
    mic_start(&processAudioData);

    //Creation du thread responsable des mouvement
    parcours_start();

    //Creation du thread responsable des animations
    animations_start();

    /* Infinite loop. */
    while (1) {
    	wait_traitement_data();
    	traitement_data();
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;
void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
