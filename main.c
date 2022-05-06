#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"


#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <chprintf.h>

#include <pi_regulator.h>

#include <sensors/imu.h>
#include <i2c_bus.h>
#include <msgbus/messagebus.h>
#include <sensors/proximity.h>
#include <leds.h>
#include <choc.h>

extern bool arret;
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

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


void motors_same_speed(int16_t speed){
	right_motor_set_speed(speed);
	left_motor_set_speed(speed);
}

void calibration_prox(int prox1, int prox4, int prox5, int prox8){

		prox1 = get_prox(0);
		prox4 = get_prox(3);
		prox5 = get_prox(4);
		prox8 = get_prox(7);
		chprintf((BaseSequentialStream *)&SD3, "La distance 1 est %d", prox1);
		chprintf((BaseSequentialStream *)&SD3, "La distance 4 est %d", prox4);
		chprintf((BaseSequentialStream *)&SD3, "La distance 5 est %d", prox5);
		chprintf((BaseSequentialStream *)&SD3, "La distance 8 est %d", prox8);
		chThdSleepMilliseconds(500);
}

int main(void)
{
    /* System init */
    halInit();
    chSysInit();
    serial_start();
    arret = false;
    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    i2c_start();
    imu_start();
    motors_init();
    proximity_start();
    clear_leds();


    //wait to stabilize the epuck
    chThdSleepMilliseconds(1000);

   //beginning calibration with LED information

    set_led(LED3,1);
    set_led(LED5,1);
    set_led(LED7,1);
    chThdSleepMilliseconds(1000);


    calibrate_acc();
    set_led(LED3,0);
    calibrate_gyro();
    set_led(LED5,0);
    calibrate_ir();

    clear_leds();
    //end of calibration
    choc_start();
    pi_regulator_start();

//---------------------Pour calibrer la distance
//    int prox1 = 0;
//    int prox4 = 0;
//    int prox5 = 0;
//    int prox8 = 0;


    while(1){
//    	calibration_prox(prox1, prox4, prox5, prox8);
    }

}

//----------------------- STACK CHK FAIL -----------------------------

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
