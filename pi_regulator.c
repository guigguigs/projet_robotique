#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <sensors/imu.h>
#include <i2c_bus.h>
#include <msgbus/messagebus.h>
#include <pi_regulator.h>
#include <choc.h>


extern bool get_arret(void);
static bool no_speed = false;

void set_speed(bool a){
	no_speed = a;
}

bool get_speed(void){
	return no_speed;
}


int16_t pi_regulator_int(int16_t acc, int16_t goal){

	int16_t error = 0;
	float speed = 0;

	static int32_t sum_error = 0;
	static int32_t prev_error = 0;

	error = acc - goal;

	//disables the PID regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and
	//the acc is never exactly 0
	if(abs(error) < ERROR_THRESHOLD){
		sum_error = 0;				//we clear the integrate and derivative parts to improve efficiency
		prev_error = 0;
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error + KD * (error - prev_error);


	prev_error = error;

	//we set speed limit to avoid speeds that could lead
	//to something we don't want

	if(speed < - MOTOR_SPEED_LIMIT){
		speed = -MOTOR_SPEED_LIMIT;
	}else if( speed > MOTOR_SPEED_LIMIT){
		speed = MOTOR_SPEED_LIMIT;
	}else if(speed < MOTOR_MIN_SPEED && speed > 0){
		speed = MOTOR_MIN_SPEED;
	}else if(speed > -MOTOR_MIN_SPEED && speed < 0){
		speed = -MOTOR_MIN_SPEED;
	}

    return speed;
}

static THD_WORKING_AREA(waPiRegulator, 1024);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    int16_t speed = 0;
    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_msg_t imu_values;
    int8_t counter = 0;

    while(1){
    	//chprintf((BaseSequentialStream *)&SD3, "L'etat d'arret est %d", arret);

        //wait for new measures to be published
			messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

			if(get_arret()){
		    	motors_same_speed(0);
			}else{

				//we take only one value over five
				if(counter < 5){
					++counter;
				}else{

					counter = 0;

					//we take 1/8 of the acceleration recieved to match the speed of the wheel

					speed = pi_regulator_int(imu_values.acc_raw[1]/8, GOAL_ACC);
					motors_same_speed(speed);
					if(speed == 0){
						set_speed(true);
					}else{
						set_speed(false);
					}

			}
		}
    }

}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO + 1, PiRegulator, NULL);
}
