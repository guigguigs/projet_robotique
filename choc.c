/*
 * choc.c
 *
 *  Created on: 6 mai 2022
 *      Author: gvull
 */
#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <sensors/imu.h>
#include <i2c_bus.h>
#include <msgbus/messagebus.h>
#include <sensors/proximity.h>
#include <leds.h>
#include <choc.h>


bool detection_wall(int IR1, int IR4/*, int IR5, int IR8*/, int16_t acc){

	if(IR1 > MIN_DISTANCE_VALUE){
		if(acc < 0){
			return false;
		}else{
			return true;
		}
	}


	if(IR4 > MIN_DISTANCE_VALUE){
		if(acc > 0){
			return false;
		}else{
			return true;
		}
	}

/*
	if(IR8 > MIN_DISTANCE_VALUE){
		if(acc < 0){
			return false;
		}else{
			return true;
		}
	}


	if(IR5 > MIN_DISTANCE_VALUE){
		if(acc > 0){
			return false;
		}else{
			return true;
		}
	}*/
	return false;
}

static THD_WORKING_AREA(waChoc, 1024);
static THD_FUNCTION(Choc, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    messagebus_topic_t *proximity_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    proximity_msg_t prox_values;
    imu_msg_t imu_values;
    int8_t counter = 0;

    while(1){
        //wait for new measures to be published
        messagebus_topic_wait(proximity_topic, &prox_values, sizeof(prox_values));
        messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

		//we take only one value over five
		if(counter < 5){
			++counter;
		}else{

			counter = 0;

			//we take a tenth of the acceleration recieved to match the speed of the wheel
			if(detection_wall(prox_values.delta[0], prox_values.delta[3],/* prox_values.delta[4], prox_values.delta[7],*/ imu_values.acc_raw[1])){
				set_led(LED1,1);
				arret = true;
			}else{
				arret = false;
				clear_leds();
			}
		}

    }
}

void choc_start(void){
	chThdCreateStatic(waChoc, sizeof(waChoc), NORMALPRIO + 1, Choc, NULL);
}
