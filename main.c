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

    //initialisation des modules de communication
    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();

    //démarrage des moteurs
    motors_init();

    //calibration des capteurs
    calibrate_ir();

    mic_start(&processAudioData);

    parcours_start();


    /* Infinite loop. */
    while (1) {
    	wait_traitement_data();
		//chprintf((BaseSequentialStream *) &SDU1, "\n hello main %d \n", 1);
    	clear_leds();
    	set_body_led(0);
    	traitement_data();
    	//waits 1 second
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
