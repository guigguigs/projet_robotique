/*
 * obstacle.c
 *
 *  Created on: 6 mai 2022
 *      Author: Guillaume VULLIOUD & Gael ESCHBACH
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
#include <obstacle.h>


static bool arret = false;
static char cote = 'n';

void set_arret(bool a){
	arret = a;
}

bool get_arret(void){
	return arret;
}

void set_cote(char a){
	cote = a;
}

char get_cote(void){
	return cote;
}

bool detection_wall(int IR1, int IR4, int IR5, int IR8, int16_t acc){

	if(IR1 > MIN_DISTANCE_VALUE){
		if(acc < 0){
			return 0;
		}else{
			set_cote('f');
			return 1;
		}
	}

	else if(IR4 > MIN_DISTANCE_VALUE){
		if(acc > 0){
			return 0;
		}else{
			set_cote('b');
			return 1;
		}
	}

	else if(IR5 > MIN_DISTANCE_VALUE){
			if(acc > 0){
				return 0;
			}else{
				set_cote('b');
				return 1;
			}
		}


	else if(IR8 > MIN_DISTANCE_VALUE){
		if(acc < 0){
			return 0;
		}else{
			set_cote('f');
			return 1;
		}
	}



	return 0;
}

static THD_WORKING_AREA(waObstacle, 512);
static THD_FUNCTION(Obstacle, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    messagebus_topic_t *proximity_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    proximity_msg_t prox_values;
    imu_msg_t imu_values;

    while(1){
        //wait for new measures to be published
        messagebus_topic_wait(proximity_topic, &prox_values, sizeof(prox_values));
        messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));


		if(detection_wall(prox_values.delta[0], prox_values.delta[3], prox_values.delta[4], prox_values.delta[7], imu_values.acc_raw[1])){
			set_arret(true);
		}else{
			set_cote('n');
			set_arret(false);
		}
    }
}

void obstacle_start(void){
	chThdCreateStatic(waObstacle, sizeof(waObstacle), NORMALPRIO, Obstacle, NULL);
}
