#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>
#include <leds.h>
#include <spi_comm.h>
#include <sensors/proximity.h>
#include <traitement_son.h>

#include "parcours.h"
#include "animations.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}


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

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();

    //d�marrage des moteurs
    motors_init();

    //calibration des capteurs
    calibrate_ir();

    //Envoie les donn�es du micro a la fonction process audio
    mic_start(&processAudioData);

    parcours_start();
    animations_start();

    /* Infinite loop. */
    while (1) {
    	//systime_t time1,time2,time3;
    	//time1 = chVTGetSystemTime();
    	wait_traitement_data();
    	traitement_data();
    	//time3 = chVTGetSystemTime();
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
