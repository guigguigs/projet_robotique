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
#include <camera/po8030.h>
#include <chprintf.h>

#include <pi_regulator.h>
#include <process_image.h>

#include <sensors/imu.h>
#include <i2c_bus.h>
#include <msgbus/messagebus.h>
#include <sensors/proximity.h>
#include <leds.h>


#define NB_SAMPLES_OFFSET     200
#define MIN_VALUE 	0.1
#define SPEED 		200
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


void show_gravity(imu_msg_t *imu_values, int16_t speed){
	right_motor_set_speed(speed);
	left_motor_set_speed(speed);
//
//    float acc_x = imu_values->acceleration [X_AXIS];
//    float acc_y = imu_values->acceleration [Y_AXIS];
//    palSetPad(GPIOD, GPIOD_LED1);
//	palSetPad(GPIOD, GPIOD_LED3);
//	palSetPad(GPIOD, GPIOD_LED5);
//	palSetPad(GPIOD, GPIOD_LED7);
//    if(fabs(acc_x) > fabs(acc_y)){
//    	if(acc_x > 0){
//    		//palClearPad(GPIOD, GPIOD_LED7);
//    		right_motor_set_speed(0);
//    		left_motor_set_speed(0);
//    	}else{
//    		//palClearPad(GPIOD, GPIOD_LED3);
//    		right_motor_set_speed(0);
//    		left_motor_set_speed(0);
//    	}
//    }else{
//    	if(acc_y < MIN_VALUE && acc_y > -MIN_VALUE){
//    		right_motor_set_speed(0);
//    		left_motor_set_speed(0);
//    		return;
//    	}
//    	if(acc_y < 0){
//    		palClearPad(GPIOD, GPIOD_LED5);
//    		right_motor_set_speed(-speed);
//			left_motor_set_speed(-speed);
//    	}else{
//    		right_motor_set_speed(speed);
//    		left_motor_set_speed(speed);
//    		palClearPad(GPIOD, GPIOD_LED1);
//    	}
//    }
}

int main(void)
{
    /* System init */
    halInit();
    chSysInit();
    serial_start();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    i2c_start();
    imu_start();
    motors_init();

    proximity_start();
    clear_leds();
    pi_regulator_start();


    int16_t speed = 0;
    //to change the priority of the thread invoking the function. The main function in this case
    //chThdSetPriority(NORMALPRIO+2);

    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_msg_t imu_values;

    //wait 2 sec to be sure the e-puck is in a stable position
    chThdSleepMilliseconds(2000);
    //imu_compute_offset(imu_topic, NB_SAMPLES_OFFSET);
    calibrate_acc();
    calibrate_gyro();
    calibrate_ir();
    int prox_front = 0;
    int prox_back = 0;

    while(1){
    /*	prox_front = get_prox(3);
    	prox_back = get_prox(0);
    	//chprintf((BaseSequentialStream *)&SD3, "La distance est %d", prox);

    	//wait for new measures to be published
    	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

        speed = pi_regulator_int((imu_values.acc_raw[1]/10), 0);

    // chprintf((BaseSequentialStream *)&SD3, "la lumière est %d", prox_front);
        if(prox_front > DISTANCE_MIN && speed < 0){
        	clear_leds();
        	show_gravity(&imu_values, 0);
        	set_led(LED5,1);
        }else if(prox_back > DISTANCE_MIN && speed > 0){
        	clear_leds();
        	show_gravity(&imu_values, 0);
        	set_led(LED1,1);
        }else{
        	clear_leds();
        	show_gravity(&imu_values, speed);
        }
        chThdSleepMilliseconds(100);*/

    }

}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
