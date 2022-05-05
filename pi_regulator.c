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
#include <process_image.h>

//simple PI regulator implementation
int16_t pi_regulator(float acc, float goal){

	float error = 0;
	float speed = 0;


	static float sum_error = 0;

	error = acc - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;

    return (int16_t)speed;
}

int16_t pi_regulator_int(int16_t acc, int16_t goal){

	int16_t error = 0;
	float speed = 0;

	static int32_t sum_error = 0;
	static int32_t prev_error = 0;

	error = acc - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and
	//the camera is a bit noisy
	if(abs(error) < ERROR_THRESHOLD){
		sum_error = 0;
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error - KD * (error - prev_error);
	prev_error = error;
	if(speed < - MOTOR_SPEED_LIMIT){
		speed = -MOTOR_SPEED_LIMIT;
	}else if( speed > MOTOR_SPEED_LIMIT){
		speed = MOTOR_SPEED_LIMIT;
//	}else if(speed < 150 && speed > 0){
//		speed = 150;
//	}else if(speed < 0 && speed > -150){
//		speed = -150;
	}

    return speed;
}

static THD_WORKING_AREA(waPiRegulator, 1024);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;
    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_msg_t imu_values;
    int8_t counter = 0;
    int16_t values = 0;
    while(1){
        time = chVTGetSystemTime();
        //wait for new measures to be published
         messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
         if(counter < 5){
        	 //values += imu_values.acc_raw[1]/10;
        	 ++counter;
        	// chprintf((BaseSequentialStream *)&SD3, "La distance est %d", counter);

         }else{
         values = imu_values.acc_raw[1]/10; // / 5;
         counter = 0;
        //computes the speed to give to the motors
        //distance_cm is modified by the image processing thread
        speed = pi_regulator_int(values, GOAL_ACC);
        //computes a correction factor to let the robot rotate to be in front of the line
 //       speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));

        //if the line is nearly in front of the camera, don't rotate
//        if(abs(speed_correction) < ROTATION_THRESHOLD){
//        	speed_correction = 0;
//        }

        //applies the speed from the PI regulator
		right_motor_set_speed(speed);
		left_motor_set_speed(speed);}

        //100Hz
    //    chThdSleepUntilWindowed(time, time + MS2ST(100));
		//chThdSleepMilliseconds(80);
    }

}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO +1, PiRegulator, NULL);
}
